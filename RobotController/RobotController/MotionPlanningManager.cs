﻿using Caliburn.Micro;
using System;
using System.ComponentModel.Composition;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VisualComponents.Create3D;
using VisualComponents.UX.Shared;

namespace RobotController
{
    public class MotionPlanningManager
    {
        /// <summary>
        /// Initialize a motionPlanner instance for a specific robot. 
        /// This instance is stored in the dictonary of the class which manages and maintains the instances of all motionPlanners of all robots.
        /// </summary>
        /// <param name="robot"></param>The VC robot instance for which a motionPlanner instance should be created
        /// <param name="pathToRobotUrdfDescriptionFile"></param>The URDF description of the robot
        /// <param name="kinChainStart"></param>The name of the first part in the kinematic chain.
        /// <param name="kinChainEnd"></param>The name of the last part in the kinematic chain.
        /// <param name="pathToObstacleFile"></param>The path to an obstacle model file (e.g. stl format).
        public MotionPlan InitializeMotionPlanner(IRobot robot, String pathToRobotUrdfDescriptionFile, String kinChainStart, String kinChainEnd, String pathToObstacleFile)
        {
            MotionPlan motionPlan = new MotionPlan(pathToRobotUrdfDescriptionFile, kinChainStart, kinChainEnd);
            if (!motionPlan.isValid()) {
                Console.WriteLine("Failed to initialize motionPlan, unable to load robot urdf file \"" + pathToRobotUrdfDescriptionFile + "\" from " + kinChainStart + " to " + kinChainEnd + "!");
                return null;
            }
            MotionPlanRobotDescription description = motionPlan.getMotionPlanRobotDescription();
            
            Vector3 wpr = robot.Component.TransformationInWorld.GetWPR();

            description.setRobotPosition(robot.Component.TransformationInWorld.GetP().X / 1000,
                                        robot.Component.TransformationInWorld.GetP().Y / 1000,
                                        robot.Component.TransformationInWorld.GetP().Z / 1000);
            description.setRobotRotation(wpr.X, wpr.Y, wpr.Z);

            motionPlan.addObstacle(pathToObstacleFile);

            return motionPlan;
        }

        /*public void printToConsole(VectorOfDouble vecOfDouble, String name)
        {
            String buf = "VectorOfDouble "+name+" = new VectorOfDouble("+vecOfDouble.Count+");\r\n";
            foreach(double d in vecOfDouble)
            {
                
                buf += name+".Add("+ String.Format("{0:0.0000}", d)+");\r\n";
            }
            IoC.Get<IMessageService>().AppendMessage(buf, MessageLevel.Warning);
        }

        public void printIkCommand(Matrix m, String name)
        {
            Vector3 wpr = m.GetWPR();
            
            String ikCommand = "//"+name+"\r\ndescription.getIK("+String.Format("{0:0.0000}", (m.GetP().X / 1000))+","+
                String.Format("{0:0.0000}", (m.GetP().Y / 1000))+", "+
                String.Format("{0:0.0000}", (m.GetP().Z / 1000))+", "+
                wpr.X + ", " + wpr.Y + ", " + wpr.Z+");";
            
            IoC.Get<IMessageService>().AppendMessage(ikCommand, MessageLevel.Warning);
        }*/

        String rD(double d)
        {
            return String.Format("{0:0.00}", d);
        }
        String getPositionHashKey(String framePrefix, Vector3 position, Vector3 rotation, VectorOfDouble currentPositionJointAngles)
        {
            String pos = rD(position.X)+","+ rD(position.Y)+","+ rD(position.Z);
            String rot = rD(rotation.X) + "," + rD(rotation.Y) + "," + rD(rotation.Z);
            String joint = "";
            foreach (double d in currentPositionJointAngles)
            {
                joint += rD(d) + ",";
            }
            return framePrefix+"@"+pos + ","+rot+","+joint;
        }

        Dictionary<String, MotionPlan> motionBrain = new Dictionary<string, MotionPlan>();

        /// <summary>
        /// Trigger and receive a concrete motionPlan from startFrame to goalFrame.
        /// Obstacles must be added to the motionPlanner instance of the robot before.
        /// </summary>
        /// <param name="robot"></param>The robot for which a motion should be planned.
        /// <param name="motionPlan"></param>The preconfigured motionPlan which should be used.
        /// <param name="startFrame"></param>The frame where the motion should start.
        /// <param name="goalFrame"></param>The frame where the motion should end.
        /// <returns></returns>
        public VectorOfDoubleVector planMotion(IRobot robot, MotionPlan motionPlan, String startFrame, String goalFrame)
        {
            MotionPlanRobotDescription description = motionPlan.getMotionPlanRobotDescription();

            // we need the current position of the robot to enhance the result of the inverse kinematics
            VectorOfDouble currentPositionJointAngles = new VectorOfDouble(robot.RobotController.Joints.Count);

            if (robot.RobotController.Joints.Count == 7)
            {
                currentPositionJointAngles.Add(robot.RobotController.Joints[0].Value);
                currentPositionJointAngles.Add(robot.RobotController.Joints[1].Value);
                currentPositionJointAngles.Add(robot.RobotController.Joints[6].Value);
                currentPositionJointAngles.Add(robot.RobotController.Joints[2].Value);
                currentPositionJointAngles.Add(robot.RobotController.Joints[3].Value);
                currentPositionJointAngles.Add(robot.RobotController.Joints[4].Value);
                currentPositionJointAngles.Add(robot.RobotController.Joints[5].Value);
            } else
            {
                currentPositionJointAngles.Add(robot.RobotController.Joints[0].Value);
                currentPositionJointAngles.Add(robot.RobotController.Joints[1].Value);
                currentPositionJointAngles.Add(robot.RobotController.Joints[2].Value);
                currentPositionJointAngles.Add(robot.RobotController.Joints[3].Value);
                currentPositionJointAngles.Add(robot.RobotController.Joints[4].Value);
                currentPositionJointAngles.Add(robot.RobotController.Joints[5].Value);
            }




            //TODO: Make those frames global?
            IFeature startNode = robot.Component.FindFeature(startFrame);
            if (startNode == null)
            {
                IoC.Get<IMessageService>().AppendMessage("Start Frame \"" + startFrame + "\" was not found.", MessageLevel.Error);
                return null;
            }
            IFeature goalNode = robot.Component.FindFeature(goalFrame);
            if (goalNode == null)
            {
                IoC.Get<IMessageService>().AppendMessage("Goal Frame \"" + goalFrame + "\" was not found.", MessageLevel.Error);
                return null;
            }


            Matrix startPosition = robot.Component.RootNode.GetFeatureTransformationInWorld(startNode);
            Matrix goalPosition = robot.Component.RootNode.GetFeatureTransformationInWorld(goalNode);
            Vector3 startRotation = startPosition.GetWPR();
            Vector3 goalRotation = goalPosition.GetWPR();
            Vector3 robotPosition = robot.Component.TransformationInWorld.GetP();
            VectorOfDouble startJointAngles = currentPositionJointAngles;
            //IoC.Get<IMessageService>().AppendMessage("Goal Frame \"" + goalFrame + "\": [" + String.Format("{0:0.000}", (goalPosition.GetP().X / 1000)) + ", " + String.Format("{0:0.000}", (goalPosition.GetP().Y / 1000)) + ", " + String.Format("{0:0.000}", (goalPosition.GetP().Z / 1000)) + "] ["+ String.Format("{0:0.000}", goalRotation.X)+" "+ String.Format("{0:0.000}", goalRotation.Y)+" "+ String.Format("{0:0.000}", goalRotation.Z) + "]", MessageLevel.Warning);

            VectorOfDouble goalJointAngles = description.getIK(goalPosition.GetP().X / 1000,
                                                                goalPosition.GetP().Y / 1000,
                                                                goalPosition.GetP().Z / 1000,
                                                                goalRotation.X, goalRotation.Y, goalRotation.Z, startJointAngles, 0.5, "Distance");

            motionPlan.setStartPosition(startJointAngles);
            motionPlan.setGoalPosition(goalJointAngles);

            String startOut = "[", goalOut = "[";
            for (int i = 0; i < startJointAngles.Count; i++){
                startOut += String.Format("{0:0.000}", startJointAngles[i]) + " ";
            }
            for (int i = 0; i < goalJointAngles.Count; i++) {
                goalOut += String.Format("{0:0.000}", goalJointAngles[i]) + " ";
            }
            startOut += "]";
            goalOut += "]";

            motionPlan.setSolveTime(10.0);
            motionPlan.setStateValidityCheckingResolution(0.01);
            //motionPlan.setReportFirstExactSolution(true);
            motionPlan.setPlannerByString("RRTConnect");
            
            if (motionPlan.plan() > 0)
            {
                //IoC.Get<IMessageService>().AppendMessage("Found motion from " + startFrame + "@" + startOut + " to " + goalFrame + "@" + goalOut + ": ", MessageLevel.Warning);
                VectorOfDoubleVector plan = motionPlan.getLastResult();
                //motionPlan.showSetupInInspector();
                foreach (VectorOfDouble jointConfiguration in plan)
                {
                    String motionBuf = "[", sep = "";
                    foreach(double jointAngle in jointConfiguration)
                    {
                        motionBuf += sep + String.Format("{0:0.000}", jointAngle);
                        sep = ",";
                    }

                    //IoC.Get<IMessageService>().AppendMessage(motionBuf + "]", MessageLevel.Warning);
                }

                //IoC.Get<IMessageService>().AppendMessage("Found motion END", MessageLevel.Warning);
                return plan;
            }
            else
            {
                //motionPlan.showSetupInInspector();
                IoC.Get<IMessageService>().AppendMessage("Failed to find motion from " + startOut + " to " + goalOut + ": " + motionPlan.getLastPlanningError(), MessageLevel.Warning);
            }
            
            
            return null;
        }


        bool allZeroes(VectorOfDouble check)
        {
            foreach (double d in check)
            {
                if (d > 1E-10)
                {
                    return false;
                }
            }
            return true;
        }
    }
}
