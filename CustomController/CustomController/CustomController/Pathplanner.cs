using System;
using System.Collections.Generic;
using System.Linq;

using System.Threading;
using System.Text;
using System.Threading.Tasks;

using Caliburn.Micro;
using VisualComponents.Create3D;


// Robot Description: "C:\\Users\\Sinan\\Desktop\\rosi.plugin.pathplanner\\robot_descriptions\\urdf\\lbr_iiwa_14_r820.urdf"
// Obstacles: "C:/Users/Sinan/Desktop/rosi.plugin.pathplanner/cage-models/fleximir-model-even-less-detailed.stl"

namespace CustomController
{
    class Pathplanner
    {
        private MotionPlan motionPlan = null;

        private ReaderWriterLockSlim varRdWrLock = new ReaderWriterLockSlim();

        public MotionPlan InitializeMotionPlanner(IRobot robot, string robotDescription, List<string> obstacles)
        {
            motionPlan = new MotionPlan();
            motionPlan.loadMotionPlanRobotDescription(robotDescription, "base_link", "tool0");
            MotionPlanRobotDescription description = motionPlan.getMotionPlanRobotDescription();


            Vector3 wpr = robot.Component.TransformationInWorld.GetWPR();

            description.setRobotPosition(robot.Component.TransformationInWorld.GetP().X / 1000,
                                        robot.Component.TransformationInWorld.GetP().Y / 1000,
                                        robot.Component.TransformationInWorld.GetP().Z / 1000);
            description.setRobotRotation(wpr.X, wpr.Y, wpr.Z);

            if (obstacles != null)
            {
                foreach (string obstacle in obstacles)
                {
                    motionPlan.addObstacle(obstacle);
                }
            }

            return motionPlan;
        }

        public VectorOfDoubleVector planMotion(IRobot robot, String startFrame, String goalFrame)
        {
            MotionPlanRobotDescription description = motionPlan.getMotionPlanRobotDescription();

            VectorOfDouble vec = new VectorOfDouble(robot.Controller.Joints.Count);
            vec.Add(robot.Controller.Joints[0].Value);
            vec.Add(robot.Controller.Joints[1].Value);
            vec.Add(robot.Controller.Joints[6].Value);
            vec.Add(robot.Controller.Joints[2].Value);
            vec.Add(robot.Controller.Joints[3].Value);
            vec.Add(robot.Controller.Joints[4].Value);
            vec.Add(robot.Controller.Joints[5].Value);

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
            for (int i=0; i < startJointAngles.Count; i++)
            {
                IoC.Get<IMessageService>().AppendMessage(startJointAngles[i].ToString(), MessageLevel.Warning);
            }
            motionPlan.setStartPosition(startJointAngles);
            motionPlan.setGoalPosition(goalJointAngles);

            motionPlan.setSolveTime(10.0);
            motionPlan.setStateValidityCheckingResolution(0.001);
            motionPlan.setPlannerByString("RRTConnect");

            if (motionPlan.plan() >= 0)
            {
                return motionPlan.getLastResult();
            }
            return null;
        }
        
    }

    
}
