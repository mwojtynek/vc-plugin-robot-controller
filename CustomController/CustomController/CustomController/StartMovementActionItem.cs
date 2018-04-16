using Caliburn.Micro;
using System;
using System.ComponentModel.Composition;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VisualComponents.Create3D;
using VisualComponents.UX.Shared;
using System.IO;
using System.Threading;
using RosiTools.Collector;
using RosiTools.Printer;
using vcMotionPlannerCSharp;

namespace CustomController
{
    [Export(typeof(IActionItem))]
    public class StartMovementActionItem : ActionItem
    {
        [Import]
        private Lazy<IApplication> app = null;

        IMessageService ms = null;

        public StartMovementActionItem() : base("StartMovement")
        {
            ms = IoC.Get<IMessageService>();
            ms.AppendMessage("Constructor of StartMovement Action Item called", MessageLevel.Warning);
        }
        public static readonly string KinStart = "base_link";
        public static readonly string KinEnd = "tool0";
        class VCJobInfo
        {
            public string pythonState;
            public IRobot robot;

            public VCJobInfo(IRobot robot, string pythonState)
            {
                this.robot = robot;
                this.pythonState = pythonState;
            }
        }

        //SortedDictionary<String, List<MotionPlan>> previousJobResults = new SortedDictionary<string, List<MotionPlan>>();
        // to reuse exisiting planners, for increasing their exploration space...
        SortedDictionary<String, MotionPlanJob> jobBrain = new SortedDictionary<string, MotionPlanJob>();
        public override void Execute(PropertyCollection args)
        {
                if(args.Count < 6)
                {
                    ms.AppendMessage("Too few arguments were passed to StartMovementActionItem. [robotName, startFrameName, goalFrameName, maxAllowedCartesianSpeed, payload, stapleComponentName]", MessageLevel.Warning);
                    return;
                }
                String robotName = (String)args.GetByIndex(0).Value;
                ISimComponent robotParent = app.Value.World.FindComponent(robotName);
                IRobot robot = robotParent.GetRobot();


                String startFrameName = (String)args.GetByIndex(1).Value;
                String goalFrameName = (String)args.GetByIndex(2).Value;
                int maxAllowedCartesianSpeed = (int)args.GetByIndex(3).Value;
                String pythonState = (String)args.GetByIndex(4).Value;
                String stapleComponentName = (String)args.GetByIndex(5).Value;


                Printer.printTimed(robotName + " is planning " + pythonState);

                RobotSection parameter = ConfigReader.readSection(robotName);
                MotionPlanJob job;
                if (!jobBrain.TryGetValue(robotName, out job))
                {
                    job = new MotionPlanJob(robotName, parameter.urdfFile.Path, KinStart, KinEnd);
                    job.OnPlanDone += NotifyController;
                    jobBrain.Add(robotName, job);
                } else {
                    job.ClearObstacles();
                }
                job.DemandedSpeed = Double.Parse(parameter.velocity.Value);
                int decimals = 8;

                job.AddObstacle(parameter.obsFile.Path);

                Vector3 wpr = robot.Component.TransformationInWorld.GetWPR();

                job.SetRobotTransformation((robot.Component.TransformationInWorld.GetP().X / 1000.0).Floor(decimals),
                                            (robot.Component.TransformationInWorld.GetP().Y / 1000.0).Floor(decimals),
                                            (robot.Component.TransformationInWorld.GetP().Z / 1000.0).Floor(decimals),
                                            wpr.X.Floor(decimals), wpr.Y.Floor(decimals), wpr.Z.Floor(decimals));
                

            if (stapleComponentName != null && stapleComponentName != "")
                {
                    stapleConfig(stapleComponentName, job);
                }

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
            }
            else
            {
                currentPositionJointAngles.Add(robot.RobotController.Joints[0].Value);
                currentPositionJointAngles.Add(robot.RobotController.Joints[1].Value);
                currentPositionJointAngles.Add(robot.RobotController.Joints[2].Value);
                currentPositionJointAngles.Add(robot.RobotController.Joints[3].Value);
                currentPositionJointAngles.Add(robot.RobotController.Joints[4].Value);
                currentPositionJointAngles.Add(robot.RobotController.Joints[5].Value);
            }
            
            //TODO: Make those frames global?
            IFeature startNode = robot.Component.FindFeature(startFrameName);
            if (startNode == null)
            {
                IoC.Get<IMessageService>().AppendMessage("Start Frame \"" + startFrameName + "\" was not found.", MessageLevel.Error);
                return;
            }
            IFeature goalNode = robot.Component.FindFeature(goalFrameName);
            if (goalNode == null)
            {
                IoC.Get<IMessageService>().AppendMessage("Goal Frame \"" + goalFrameName + "\" was not found.", MessageLevel.Error);
                return;
            }


            Matrix startPosition = robot.Component.RootNode.GetFeatureTransformationInWorld(startNode);
            Matrix goalPosition = robot.Component.RootNode.GetFeatureTransformationInWorld(goalNode);
            Vector3 startRotation = startPosition.GetWPR();
            Vector3 goalRotation = goalPosition.GetWPR();
            Vector3 robotPosition = robot.Component.TransformationInWorld.GetP();
            VectorOfDouble startJointAngles = currentPositionJointAngles;

            VectorOfDouble goalCartPos = new VectorOfDouble();
            goalCartPos.Add((goalPosition.GetP().X / 1000.0).Floor(decimals));
            goalCartPos.Add((goalPosition.GetP().Y / 1000.0).Floor(decimals));
            goalCartPos.Add((goalPosition.GetP().Z / 1000.0).Floor(decimals));
            goalCartPos.Add(goalRotation.X.Floor(decimals));
            goalCartPos.Add(goalRotation.Y.Floor(decimals));
            goalCartPos.Add(goalRotation.Z.Floor(decimals));
            
            job.SetGoalStateAsCartesian(goalCartPos, startJointAngles);


            job.SetStartStateFromVector(startJointAngles);
            job.Goal_IK_Mode = "NonCollision";
            job.IK_Solve_Time = 15.0;

            job.SetSolveTime(30.0);

            job.SetStateValidityCheckingResolution(0.01);

            job.SetPlannerByString("RRTConnect");
            
            job.SetUserData(new VCJobInfo(robot, pythonState));
            
            MotionPlanJobExecutor.submit(job);

            /*VectorOfDoubleVector resultMotion = mpm.planMotion(robot, motionPlan, startFrameName, goalFrameName);
            if (resultMotion != null)
            {
                IBehavior beh = robot.Component.FindBehavior("MovementFinished");
                if (beh != null && beh is IStringSignal)
                {
                    IStringSignal movementFinished = (IStringSignal)robot.Component.FindBehavior("MovementFinished");
                    movementFinished.Value = ""; // empty string means no payload contained yet
                    CustomController sinanController = IoC.Get<ICollectorManager>().getInstance("CustomController", robotParent) as CustomController;
                    if (sinanController != null)
                    {
                        sinanController.moveAlongJointAngleList(pythonState, motionPlan);
                    }
                    else
                    {
                        ms.AppendMessage("Controller not found", MessageLevel.Warning);
                    }
                } else {
                    ms.AppendMessage("\"MovementFinished\" behavior was either null or not of type IStringSignal. Abort!", MessageLevel.Warning);
                }
            }*/

        }

        private void NotifyController(object sender, MotionPlanJobDoneEvent e)
        {
            try
            {
                VCJobInfo jobinfo = e.Job.GetUserData<VCJobInfo>();
                List<MotionPlan> value = null;

                /*                if(!previousJobResults.TryGetValue(jobinfo.robot.Component.Name, out value))
                                {
                                    value = new List<MotionPlan>();
                                    previousJobResults.Add(jobinfo.robot.Component.Name, value);
                                }
                                value.Add(e.Plan);*/
                if (!jobinfo.robot.IsValid) return;

                Printer.printTimed(jobinfo.robot.Component.Name + " planned Path with: " + e.Plan.getLastPlanningError() + " (" + e.Job.ResultCode.ToString() + ")");
                if (e.Job.ResultCode > 0)
                {
                    IBehavior beh = jobinfo.robot.Component.FindBehavior("MovementFinished");
                    if (beh != null && beh is IStringSignal)
                    {
                        IStringSignal movementFinished = (IStringSignal)jobinfo.robot.Component.FindBehavior("MovementFinished");
                        movementFinished.Value = ""; // empty string means no payload contained yet
                        CustomController sinanController = IoC.Get<ICollectorManager>().getInstance("CustomController", jobinfo.robot.Component) as CustomController;
                        if (sinanController != null)
                        {
                            sinanController.DemandedSpeed = e.Job.DemandedSpeed;
                            sinanController.MoveAlongJointAngleList(jobinfo.pythonState, e.Plan);
                        }
                        else
                        {
                            Printer.printTimed("Error: Controller not found!");
                        }
                    }
                    else
                    {
                        ms.AppendMessage("Error: \"MovementFinished\" behavior was either null or not of type IStringSignal. Abort!", MessageLevel.Warning);
                    }
                } else {
                    Printer.printTimed("Error: Failed to find movement! JobInfo: ", MessageLevel.Warning);
                    Printer.printTimed(e.Job.ToString());
                }
            }
            catch (Exception ee) {
                Printer.print(ee.Message + "\n" + ee.StackTrace);
            }
        }

        private void stapleConfig(String stapleComponentName, MotionPlanJob job)
        {
            StapleSection parameterStaple = ConfigReader.readStapleConfig();

            ISimComponent stapleComponent = app.Value.World.FindComponent(stapleComponentName);
            if (stapleComponentName != "" && stapleComponent == null){
                ms.AppendMessage("Failed to find staple component with name\"" + stapleComponentName + "\"! Planning of motion aborted...", MessageLevel.Warning);
                return;
            }

            String stapleFileFolder = parameterStaple.staplePath.Path;
            if (stapleComponent.GetProperty("stlID") == null){
                ms.AppendMessage("Component with name \"" + stapleComponentName + "\" has no property \"stlID\"! Planning of motion aborted...", MessageLevel.Warning);
                return;
            }
            String stlID = (String)stapleComponent.GetProperty("stlID").Value;
            String obstacleFilePath = stapleFileFolder + stlID + ".stl";
            if (!File.Exists(obstacleFilePath)){
                ms.AppendMessage("A path planning request requested stlID with \"" + stlID + "\" failed, because file \"" + obstacleFilePath + "\" does not exist...", MessageLevel.Warning);
                return;
            };
            if (stapleComponent.GetProperty("StackHeight") == null){
                ms.AppendMessage("Failed to find StackHeight property in component with name \"" + stapleComponentName + "\"! Planning of motion aborted!", MessageLevel.Warning);
                return;
            }

            Vector3 staplePosition = stapleComponent.TransformationInWorld.GetP();
            IDoubleProperty stackwidth = (IDoubleProperty)stapleComponent.GetProperty("StackWidth");
            IDoubleProperty stacklength = (IDoubleProperty)stapleComponent.GetProperty("StackLength");

            float translate_x = (float)staplePosition.X;// - (float)(stacklength.Value / 2.0);
            float translate_y = (float)staplePosition.Y;// - (float)(stackwidth.Value / 2.0);

            IDoubleProperty stackheight = (IDoubleProperty)stapleComponent.GetProperty("StackHeight");
            IDoubleProperty paletteHeight = (IDoubleProperty)stapleComponent.GetProperty("PaletteHeight");
            float translate_z = (float) (stackheight.Value + paletteHeight.Value);

            // setup staple obstacle
            job.AddObstacle(obstacleFilePath, translate_x / 1000.0f, translate_y / 1000.0f, translate_z / 1000.0f, 0.0f, 0.0f, 0.0f);
        }

    }
}
