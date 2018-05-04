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
        SortedDictionary<string, MotionPlan> jobBrain = new SortedDictionary<string, MotionPlan>();
        public override void Execute(PropertyCollection args)
        {
                if(args.Count < 7)
                {
                    ms.AppendMessage("Too few arguments were passed to StartMovementActionItem. [robotName, startFrameName, goalFrameName, maxAllowedCartesianSpeed, payload, stapleComponentName, Cacheable]", MessageLevel.Warning);
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
                bool cacheable = ((int)args.GetByIndex(6).Value)==1;
                //cacheable = false;

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

                RobotSection parameter = ConfigReader.readSection(robotName);
                string stapleCache = "";
                ISimComponent stapleComponent = app.Value.World.FindComponent(stapleComponentName);
                if (stapleComponent != null)
                {
                    stapleCache = stapleComponent.TransformationInWorld.GetP().ToString(); 
                }


            string cacheKey = getCacheKey(currentPositionJointAngles, pythonState, stapleComponentName, stapleCache, robotName);
                MotionPlan result;
                if(jobBrain.TryGetValue(cacheKey, out result)){
                    sendJobDoneEvent(robot, Double.Parse(parameter.velocity.Value), result, pythonState);
                    //Printer.printTimed(robotName + " loaded plan for " + pythonState);
                } else {
                    Printer.printTimed(robotName + " is planning " + pythonState);


                    MotionPlanJob job = new MotionPlanJob(robotName, parameter.urdfFile.Path, KinStart, KinEnd);
                    job.cacheable = cacheable;
                    job.cacheKey = cacheKey;

                    job.OnPlanDone += NotifyController;

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
                    job.Goal_IK_Mode = "Speed";
                    job.IK_Solve_Time = 0.5;

                    job.SetSolveTime(30.0);

                    job.SetStateValidityCheckingResolution(0.01);

                    job.SetPlannerByString("RRTConnect");

                    job.SetUserData(new VCJobInfo(robot, pythonState));

                    MotionPlanJobExecutor.submit(job);
                }
                
                
        }

        private string getCacheKey(VectorOfDouble currentPositionJointAngles, string goalFrameName, string stapleComponentName, string stapleCache, string robotName)
        {
            string cacheKey = robotName+"_"+stapleComponentName+"_"+stapleCache;
            foreach (double pos in currentPositionJointAngles)
            {
                cacheKey += Math.Round(pos)+"_";
            }
            cacheKey += goalFrameName;
            return cacheKey;
        }

        private void NotifyController(object sender, MotionPlanJobDoneEvent e)
        {
            try
            {
                VCJobInfo jobinfo = e.Job.GetUserData<VCJobInfo>();
                List<MotionPlan> value = null;
                
                if (!jobinfo.robot.IsValid) return;

                Printer.printTimed(jobinfo.robot.Component.Name + " planned Path with: " + e.Plan.getLastPlanningError() + " (" + e.Job.ResultCode.ToString() + ")");
                if (e.Job.ResultCode > 0)
                {
                    if (e.Job.cacheable)
                    {
                        jobBrain.Add(e.Job.cacheKey, e.Plan);
                    } else
                    {
                         Printer.printTimed(jobinfo.robot.Component.Name + " starts Motion to " + jobinfo.pythonState);
                    }
                    sendJobDoneEvent(jobinfo.robot, e.Job.DemandedSpeed, e.Plan, jobinfo.pythonState);
                }
                else {
                    Printer.printTimed("Error: Failed to find movement! JobInfo: ", MessageLevel.Warning);
                    Printer.printTimed(e.Job.ToString());
                }
            }
            catch (Exception ee) {
                Printer.print(ee.Message + "\n" + ee.StackTrace);
            }
        }

        private void sendJobDoneEvent(IRobot robot, double DemandedSpeed, MotionPlan mp, string pythonState)
        {
            IBehavior beh = robot.Component.FindBehavior("MovementFinished");
            if (beh != null && beh is IStringSignal)
            {
                IStringSignal movementFinished = (IStringSignal)robot.Component.FindBehavior("MovementFinished");
                movementFinished.Value = ""; // empty string means no payload contained yet
                CustomController sinanController = IoC.Get<ICollectorManager>().getInstance("CustomController", robot.Component) as CustomController;
                if (sinanController != null)
                {
                    sinanController.DemandedSpeed = DemandedSpeed;
                    sinanController.MoveAlongJointAngleList(pythonState, mp);
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
            double setPalette = paletteHeight.Value;
            if (job.cacheable)
            {
                // assume palette max height
                setPalette  = getMaximumPaletteHeight();
            }
            float translate_z = (float) (stackheight.Value + setPalette);

            // setup staple obstacle
            job.AddObstacle(obstacleFilePath, translate_x / 1000.0f, translate_y / 1000.0f, translate_z / 1000.0f, 0.0f, 0.0f, 0.0f);
        }

        private double getMaximumPaletteHeight()
        {
            return 2000.0; // TODO: FB
        }
    }
}
