using Caliburn.Micro;
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
        public Dictionary<IRobot, MotionPlan> MotionPlanners { get; }

        public MotionPlanningManager()
        {
            MotionPlanners = new Dictionary<IRobot, MotionPlan>();

        }

        /// <summary>
        /// Initialize a motionPlanner instance for a specific robot. 
        /// This instance is stored in the dictonary of the class which manages and maintains the instances of all motionPlanners of all robots.
        /// </summary>
        /// <param name="robot"></param>The VC robot instance for which a motionPlanner instance should be created
        /// <param name="pathToRobotUrdfDescriptionFile"></param>The URDF description of the robot
        /// <param name="kinChainStart"></param>The name of the first part in the kinematic chain.
        /// <param name="kinChainEnd"></param>The name of the last part in the kinematic chain.
        /// <param name="pathToObstacleFile"></param>The path to an obstacle model file (e.g. stl format).
        public void InitializeMotionPlanner(IRobot robot, String pathToRobotUrdfDescriptionFile, String kinChainStart, String kinChainEnd, String pathToObstacleFile)
        {
            if (robot.Component.GetProperty("motionStartTime") == null) { 
                robot.Component.CreateProperty(typeof(double), PropertyConstraintType.NotSpecified, "motionStartTime");
            }
            if (robot.Component.GetProperty("motionEndTime") == null)
            {
                robot.Component.CreateProperty(typeof(double), PropertyConstraintType.NotSpecified, "motionEndTime");
            }

            MotionPlan motionPlan = new MotionPlan();
            motionPlan.loadMotionPlanRobotDescription(pathToRobotUrdfDescriptionFile, kinChainStart, kinChainEnd);
            MotionPlanRobotDescription description = motionPlan.getMotionPlanRobotDescription();
            
            Vector3 wpr = robot.Component.TransformationInWorld.GetWPR();

            description.setRobotPosition(robot.Component.TransformationInWorld.GetP().X / 1000,
                                        robot.Component.TransformationInWorld.GetP().Y / 1000,
                                        robot.Component.TransformationInWorld.GetP().Z / 1000);
            description.setRobotRotation(wpr.X, wpr.Y, wpr.Z);

            motionPlan.addObstacle(pathToObstacleFile);
            if (!MotionPlanners.ContainsKey(robot))
            {
                MotionPlanners.Add(robot, motionPlan);
            }
        }

        /// <summary>
        /// Add additional obstacles to a motionPlanner instance of the specified robot.
        /// </summary>
        /// <param name="robot"></param>The robot for which the obstacle should be relevant.
        /// <param name="pathToObstacleFile"></param>The path to an obstacle model file (e.g. stl format).
        public void addObstacleToRobot(IRobot robot, String pathToObstacleFile)
        {
            MotionPlanners[robot].addObstacle(pathToObstacleFile);
        }


        /// <summary>
        /// Trigger and receive a concrete motionPlan from startFrame to goalFrame.
        /// Obstacles must be added to the motionPlanner instance of the robot before.
        /// </summary>
        /// <param name="robot"></param>The robot for which a motion should be planned.
        /// <param name="startFrame"></param>The frame where the motion should start.
        /// <param name="goalFrame"></param>The frame where the motion should end.
        /// <returns></returns>
        public VectorOfDoubleVector planMotion(IRobot robot, String startFrame, String goalFrame)
        {
            MotionPlanRobotDescription description = MotionPlanners[robot].getMotionPlanRobotDescription();

            VectorOfDouble vec = new VectorOfDouble(robot.Controller.Joints.Count);
            vec.Add(robot.Controller.Joints[0].Value);
            vec.Add(robot.Controller.Joints[1].Value);
            vec.Add(robot.Controller.Joints[6].Value);
            vec.Add(robot.Controller.Joints[2].Value);
            vec.Add(robot.Controller.Joints[3].Value);
            vec.Add(robot.Controller.Joints[4].Value);
            vec.Add(robot.Controller.Joints[5].Value);

            //TODO: Make those frames global?
            IFeature startNode = robot.Component.FindFeature(startFrame);
            IFeature goalNode = robot.Component.FindFeature(goalFrame);

            Matrix startPosition = robot.Component.RootNode.GetFeatureTransformationInWorld(startNode);
            Matrix goalPosition = robot.Component.RootNode.GetFeatureTransformationInWorld(goalNode);
            Vector3 startRotation = startPosition.GetWPR();
            Vector3 goalRotation = goalPosition.GetWPR();

            VectorOfDouble startJointAngles = description.getIK(startPosition.GetP().X / 1000,
                                                                startPosition.GetP().Y / 1000,
                                                                startPosition.GetP().Z / 1000,
                                                                startRotation.X, startRotation.Y, startRotation.Z);
            VectorOfDouble goalJointAngles = description.getIK(goalPosition.GetP().X / 1000,
                                                                goalPosition.GetP().Y / 1000,
                                                                goalPosition.GetP().Z / 1000,
                                                                goalRotation.X, goalRotation.Y, goalRotation.Z);
            
            MotionPlanners[robot].setStartPosition(startJointAngles);
            MotionPlanners[robot].setGoalPosition(goalJointAngles);

            MotionPlanners[robot].setSolveTime(10.0);
            MotionPlanners[robot].setStateValidityCheckingResolution(0.001);
            //motionPlan.setReportFirstExactSolution(true);
            MotionPlanners[robot].setPlannerByString("RRTConnect");

            if (MotionPlanners[robot].plan() >= 0)
            {
                //motionPlan.interpolatePath()
                return MotionPlanners[robot].getLastResult();
            }
            return null;
        }
    }
}
