using Caliburn.Micro;
using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel.Composition;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using VisualComponents.Create3D;
using VisualComponents.UX.Shared;

namespace RobotController
{
    [Export(typeof(IPlugin))]
    public class RobotController : IPlugin
    {
        private IApplication app = null;
        private static RobotController instance = null;
        private IRoutine customProgram = null;
        private IMessageService ms = null;
        private ReaderWriterLockSlim varRdWrLock = new ReaderWriterLockSlim();
        private ReaderWriterLockSlim jntRdWrLock = new ReaderWriterLockSlim();
        private VectorOfDoubleVector motionPlanResult = null;
        private double timeElapsedBase = 0.0;
        private MotionPlan motionPlan = null;
        private IRobot globalRobot = null;
        private IStatisticsManager statisticsManager = null;
        private ISamplingTimer timer = null;
        private double motionStartTime = 0.0;
        private double motionEndTime = 0.0;

        //Create Constructor for class
        [ImportingConstructor]
        public RobotController([Import(typeof(IApplication))] IApplication app)
        {
            this.app = app;
            instance = this;
            this.app.Simulation.SimulationStarted += SetInitialTime;
            this.app.Simulation.SimulationStarted += SimulationStarted;
            this.app.Simulation.SimulationStopped += SimulationStopped;
            this.app.Simulation.SimulationRun += SimulationRunLogger;
        }

        public static RobotController getInstance()
        {
            return instance;
        }

        public void Exit()
        {
        }


        /*
         * Get StatisticsManager instance, enable it and set an interval!
         */
        public void Initialize()
        {
            //IoC.Get<ISimulationService>().PropertyChanged += JointConfigurationChanged;
            ms = IoC.Get<IMessageService>();
            statisticsManager = this.app.StatisticsManager;
            statisticsManager.IsEnabled = true;
            statisticsManager.StatisticsInterval = 0.1;
            // Must be at least Warning, Info level is not printed
            ms.AppendMessage("DynamicRobotControl Plugin started initialization...", MessageLevel.Warning);
        }

        public void SimulationStarted(object sender, EventArgs e)
        {
            ms.AppendMessage("Simulation Started", MessageLevel.Warning);
            timer = statisticsManager.CreateTimer(RegularTick, 0.1);
            timer.StartStopTimer(true);
        }

        public void SimulationStopped(object sender, EventArgs e)
        {
            ms.AppendMessage("Simulation Stopped", MessageLevel.Warning);
            if (timer != null)
            {
                timer.StartStopTimer(false);
            }
        }

        public void RegularTick(object sender, EventArgs e)
        {
            ms.AppendMessage("Current Time: " + app.Simulation.Elapsed, MessageLevel.Warning);

            interpolatePlannedMotion(globalRobot);
        }

        public MotionPlan InitializeMotionPlanner(IRobot robot)
        {
            //TODO: Dirty hack!! Remove!
            globalRobot = robot;
            robot.RobotController.Heartbeat += HeartbeatLogger;
            motionPlan = new MotionPlan();
            motionPlan.loadMotionPlanRobotDescription("S:\\git\\rosi.plugin.pathplanner\\robot_descriptions\\urdf\\lbr_iiwa_14_r820.urdf", "base_link", "tool0");
            MotionPlanRobotDescription description = motionPlan.getMotionPlanRobotDescription();


            Vector3 wpr = robot.Component.TransformationInWorld.GetWPR();

            description.setRobotPosition(robot.Component.TransformationInWorld.GetP().X / 1000,
                                        robot.Component.TransformationInWorld.GetP().Y / 1000,
                                        robot.Component.TransformationInWorld.GetP().Z / 1000);
            description.setRobotRotation(wpr.X, wpr.Y, wpr.Z);

            motionPlan.addObstacle("S:/git/rosi.plugin.pathplanner/cage-models/fleximir-model-even-less-detailed.stl");

            return motionPlan;
        }

        public VectorOfDoubleVector planMotion(IRobot robot, String startFrame, String goalFrame, MotionPlan motionPlan)
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

            motionPlan.setStartPosition(startJointAngles);
            motionPlan.setGoalPosition(goalJointAngles);

            motionPlan.setSolveTime(10.0);
            motionPlan.setStateValidityCheckingResolution(0.001);
            //motionPlan.setReportFirstExactSolution(true);
            motionPlan.setPlannerByString("RRTConnect");

            if (motionPlan.plan() >= 0)
            {
                motionPlanResult = motionPlan.getLastResult();
                motionStartTime = app.Simulation.Elapsed;
                //motionPlan.interpolatePath()
                return motionPlan.getLastResult();
            }
            return null;
        }

        public void SetInitialTime(object sender, EventArgs e)
        {
            timeElapsedBase = app.Simulation.Elapsed;
        }

        public void SimulationRunLogger(object sender, EventArgs e)
        {
            ms.AppendMessage("Sender: " + sender.ToString(), MessageLevel.Warning);
        }

        public void HeartbeatLogger(object sender, HeartbeatEventArgs e)
        {
            ms.AppendMessage("Sender: " + sender.ToString() + " Time: " + e.Time, MessageLevel.Warning);
        }

        private double[] kukaSorted(VectorOfDouble jointAngleCollection)
        {
            double[] firstJointAngleCollectionSorted = new double[7];
            
            //VectorOfDouble firstJointAngleCollectionSorted = new VectorOfDouble(jointAngleCollection.Count);
            firstJointAngleCollectionSorted[0] = jointAngleCollection.ElementAt(0); //A1 0
            firstJointAngleCollectionSorted[1] = jointAngleCollection.ElementAt(1); //A2 1
            firstJointAngleCollectionSorted[2] = jointAngleCollection.ElementAt(3); //A3 6
            firstJointAngleCollectionSorted[3] = jointAngleCollection.ElementAt(4); //A4 2
            firstJointAngleCollectionSorted[4] = jointAngleCollection.ElementAt(5); //A5 3
            firstJointAngleCollectionSorted[5] = jointAngleCollection.ElementAt(6); //A6 4
            firstJointAngleCollectionSorted[6] = jointAngleCollection.ElementAt(2); //A7 5

            return firstJointAngleCollectionSorted;
        }
        private void interpolatePlannedMotion (IRobot robot)
        {
            if(motionPlanResult!= null)
            {
                double[] startJointAngleCollection = kukaSorted(motionPlanResult.First());

                IMotionTarget startMotionTarget = robot.RobotController.CreateTarget();
                startMotionTarget.CartesianSpeed = 1.0;
                startMotionTarget.AngularSpeed = 1.0;
                // JointSpeed Value from 0-100
                startMotionTarget.JointSpeed = 1.0;
                startMotionTarget.MotionType = MotionType.Joint;
                startMotionTarget.SetAllJointValues(startJointAngleCollection);
                startMotionTarget.UseJointValues = true;


                double[] goalJointAngleCollection = kukaSorted(motionPlanResult.Last());

                IMotionTarget goalMotionTarget = robot.RobotController.CreateTarget();
                goalMotionTarget.CartesianSpeed = 1.0;
                goalMotionTarget.AngularSpeed = 1.0;
                // JointSpeed Value from 0-100
                goalMotionTarget.JointSpeed = 1.0;
                goalMotionTarget.MotionType = MotionType.Joint;
                goalMotionTarget.SetAllJointValues(goalJointAngleCollection);
                goalMotionTarget.UseJointValues = true;
                
                //robot.RobotController.Evaluate();

                IMotionInterpolator motionInterpolator = robot.RobotController.CreateMotionInterpolator();
                motionInterpolator.AddTarget(startMotionTarget);
                motionInterpolator.AddTarget(goalMotionTarget);

                ms.AppendMessage("MotionInterpolator " + motionInterpolator.Targets.Count, MessageLevel.Warning);
                ms.AppendMessage("GetCycleTime at 0: " + motionInterpolator.GetCycleTimeAt(0), MessageLevel.Warning);
                ms.AppendMessage("GetCycleTime at 1: " + motionInterpolator.GetCycleTimeAt(1), MessageLevel.Warning);
                ms.AppendMessage("GetCycleTime at " + motionInterpolator.Targets.Count + ": " + motionInterpolator.GetCycleTimeAt(motionInterpolator.Targets.Count-1), MessageLevel.Warning);

                if (motionEndTime == 0.0)
                {
                    motionEndTime = motionInterpolator.GetCycleTimeAt(1);
                    ms.AppendMessage("motion will end at: " + motionEndTime, MessageLevel.Warning);
                }
                
                IMotionTarget refMotionTarget = robot.RobotController.CreateTarget();

                motionInterpolator.Interpolate(app.Simulation.Elapsed-motionStartTime, ref refMotionTarget);
                

                IMotionTester motionTester = robot.RobotController.GetMotionTester();
                motionTester.CurrentTarget = refMotionTarget;

                if (app.Simulation.Elapsed > motionEndTime)
                {
                    motionStartTime = 0.0;
                    motionEndTime = 0.0;
                    motionPlanResult = null;
                }

             }
        }

        public void JointConfigurationChanged(object sender, EventArgs e)
        {

            ISimulationService simulationService = (ISimulationService)sender;
            ms.AppendMessage("Simulation Service Loop: " + simulationService.Elapsed, MessageLevel.Warning);
            try
            {
                //ISimComponent robotComponent = (ISimComponent)sender;

                varRdWrLock.EnterWriteLock();
                if (motionPlanResult != null && motionPlanResult.Count > 0 && globalRobot.RobotController != null)
                {
                    //  app.WriteLine("Interpolating for " + (app.Simulation.Elapsed - timeElapsedBase));
                    double timeNow = app.Simulation.Elapsed - timeElapsedBase;

                    if (!motionPlan.isInterpolationDone(timeNow, 20))
                    {
                        VectorOfDouble result = motionPlan.interpolatePath(timeNow, 20.0);
                        List<double> tmp_jointList = new List<double>();
                        tmp_jointList.Add(result[0]);
                        tmp_jointList.Add(result[1]);
                        tmp_jointList.Add(result[3]);
                        tmp_jointList.Add(result[4]);
                        tmp_jointList.Add(result[5]);
                        tmp_jointList.Add(result[6]);
                        tmp_jointList.Add(result[2]);
                        globalRobot.RobotController.SetJointValues(tmp_jointList);
                        globalRobot.RobotController.InvalidateKinChains();
                    }
                }
            }
            finally
            {
                varRdWrLock.ExitWriteLock();
            }

            //try
            //{

            //    jntRdWrLock.EnterWriteLock();
            //    if (jointReader)
            //    {
            //        List<double> tmp_jointList = new List<double>();
            //        tmp_jointList.Add(jointList[0]);
            //        tmp_jointList.Add(jointList[1]);
            //        tmp_jointList.Add(jointList[3]);
            //        tmp_jointList.Add(jointList[4]);
            //        tmp_jointList.Add(jointList[5]);
            //        tmp_jointList.Add(jointList[6]);
            //        tmp_jointList.Add(jointList[2]);
            //        robot.RobotController.SetJointValues(tmp_jointList);
            //        robot.RobotController.SetJointValues(tmp_jointList);
            //        robot.RobotController.InvalidateKinChains();
            //        jointReader = false;
            //        //app.WriteLine("Angles from Update Event");
            //        //app.WriteLine("Joint [0] Degree: " + jointList[0] + " Radians: " + MathConverter.DegreeToRadian(jointList[0]));
            //        //app.WriteLine("Joint [1] Degree: " + jointList[1] + " Radians: " + MathConverter.DegreeToRadian(jointList[1]));                        
            //        //app.WriteLine("Joint [3] Degree: " + jointList[3] + " Radians: " + MathConverter.DegreeToRadian(jointList[3]));                        
            //        //app.WriteLine("Joint [4] Degree: " + jointList[4] + " Radians: " + MathConverter.DegreeToRadian(jointList[4]));                        
            //        //app.WriteLine("Joint [5] Degree: " + jointList[5] + " Radians: " + MathConverter.DegreeToRadian(jointList[5]));                        
            //        //app.WriteLine("Joint [6] Degree: " + jointList[6] + " Radians: " + MathConverter.DegreeToRadian(jointList[6]));                        
            //        //app.WriteLine("Joint [2] Degree: " + jointList[2] + " Radians: " + MathConverter.DegreeToRadian(jointList[2]));
            //    }
            //}
            //finally
            //{
            //    jntRdWrLock.ExitWriteLock();
            //}
        }

        [Export(typeof(IActionItem))]
        public class PlanMotion : ActionItem
        {
            [Import]
            private Lazy<IApplication> app = null;

            IMessageService ms = null;

            public PlanMotion() : base("PlanMotion")
            {
                ms = IoC.Get<IMessageService>();
                ms.AppendMessage("Constructor of PlanMotion Action Item called", MessageLevel.Warning);
            }

            public override void Execute(PropertyCollection args)
            {
                ms.AppendMessage("Executing PlanMotion...", MessageLevel.Warning);

                //TODO: Fix the hard index access or at least print out a message if input was wrong
                String robotName = (String)args.GetByIndex(0).Value;
                IRobot robot = app.Value.World.FindComponent(robotName).GetRobot();

                MotionPlan motionPlanInstance = RobotController.getInstance().InitializeMotionPlanner(robot);

                String startFrameName = (String)args.GetByIndex(1).Value;

                String goalFrameName = (String)args.GetByIndex(2).Value;

                VectorOfDoubleVector resultMotion = RobotController.getInstance().planMotion(robot, startFrameName, goalFrameName, motionPlanInstance);

                foreach (VectorOfDouble vector in resultMotion)
                {
                    String angles = "";
                    foreach (double angle in vector)
                    {
                        angles = angles + " , " + angle;
                    }
                    ms.AppendMessage("Angles: " + angles, MessageLevel.Warning);
                }
                ms.AppendMessage("Executed PlanMotion.", MessageLevel.Warning);
            }
        }
    }
}
