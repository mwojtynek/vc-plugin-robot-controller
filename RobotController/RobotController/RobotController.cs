using Caliburn.Micro;
using Rosi.Components.Sensors;
using SpeedAndSeparationMonitoring;
using System;
using System.Collections.Generic;
using System.ComponentModel.Composition;
using System.Linq;
using VisualComponents.Create3D;
using VisualComponents.UX.Shared;

namespace RobotController
{
    [Export(typeof(IPlugin))]
    public class RobotController : IPlugin
    {
        private const double TICK_INTERVAL = 0.1;
        private IApplication app = null;
        private IMessageService ms = null;
        private static RobotController instance = null;
        private IStatisticsManager statisticsManager = null;
        private ISamplingTimer timer = null;
        private MotionPlanningManager MotionPlanningManagerInstance { get; }
        private List<LaserScanner> laser_scanners = new List<LaserScanner>();
        private Dictionary<IRobot, RobotParameters> robotList = new Dictionary<IRobot, RobotParameters>();

        //Create Constructor for class
        [ImportingConstructor]
        public RobotController([Import(typeof(IApplication))] IApplication app)
        {
            this.app = app;
            instance = this;
            MotionPlanningManagerInstance = new MotionPlanningManager();
            this.app.Simulation.SimulationStarted += SimulationStarted;
            this.app.Simulation.SimulationStopped += SimulationStopped;
        }

        public static RobotController getInstance()
        {
            return instance;
        }

        public void Exit()
        {
        }

        public bool RegisterRobot(IRobot robot)
        {
            if (robotList.ContainsKey(robot))
            {
                return false;
            }
            else
            {
                robotList.Add(robot, new RobotParameters());
                robotList[robot].speedCalculator = new SpeedCalculator(30, 0);
                robotList[robot].seperationCalculator = new SeparationCalculator(1.0, 1.0, 10.0, 10.0, 0.1);
                robotList[robot].maxCartesianSpeed = 10.0;
                return true;
            }
        }

        public void addMotionPlan(IRobot robot, MotionPlan motionPlan)
        {
            try
            {
                robotList[robot].motionPlan = motionPlan;
            } catch(KeyNotFoundException e)
            {
                ms.AppendMessage("Motion plan could not be added, maybe robot was not registred?", MessageLevel.Warning);
                ms.AppendMessage(e.ToString(), MessageLevel.Error);
            }
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
            statisticsManager.StatisticsInterval = TICK_INTERVAL;
            // Must be at least Warning, Info level is not printed
            ms.AppendMessage("DynamicRobotControl Plugin started initialization...", MessageLevel.Warning);

            app.World.ComponentAdded += World_ComponentAdded;
            app.World.ComponentRemoving += World_ComponentRemoving;
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
            //ms.AppendMessage("Current Time: " + app.Simulation.Elapsed, MessageLevel.Warning);

            foreach (IRobot robot in robotList.Keys)
            {
                interpolatePlannedMotion(robot);
                calculateCurrentRobotSpeed(robot, robot.RobotController.ToolCenterPoint);
            }

            foreach(LaserScanner laserScanner in laser_scanners)
            {
                laserScanner.Scan();
            }
        }

        private void calculateCurrentRobotSpeed(IRobot robot, Matrix currentTcpWorldPosition)
        {
            //Distance between last and current position
            if (!robotList[robot].lastTcpWorldPosition.Equals(Matrix.Zero))
            {
                double distance = (robotList[robot].lastTcpWorldPosition.GetP() - robot.RobotController.ToolCenterPoint.GetP()).Length;
                //[mm/s]
                robotList[robot].currentCartesianSpeed = distance * 1 / TICK_INTERVAL;
            }

            robotList[robot].lastTcpWorldPosition = currentTcpWorldPosition;
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

        private IMotionTarget createIMotionTargetForJointAngleConfiguration(IRobot robot, double[] jointAngleCollection, MotionType motionType, double cartesianSpeed)
        {
            IMotionTarget motionTarget = robot.RobotController.CreateTarget();

            if(motionType == MotionType.Joint)
            {
                // JointSpeed Value from 0-100
                if (robotList[robot].allowedCartesianSpeed > cartesianSpeed)
                {
                    motionTarget.JointSpeed = robotList[robot].maxCartesianSpeed / robotList[robot].allowedCartesianSpeed;
                }
                else
                {
                    motionTarget.JointSpeed = robotList[robot].maxCartesianSpeed / cartesianSpeed;
                }
                ms.AppendMessage("SetJoint Speed for motion to: " + motionTarget.JointSpeed, MessageLevel.Warning);
            }
            else if (motionType == MotionType.Linear)
            {
                if(robotList[robot].allowedCartesianSpeed > cartesianSpeed)
                {
                    motionTarget.CartesianSpeed = robotList[robot].allowedCartesianSpeed;
                }
                else
                {
                    motionTarget.CartesianSpeed = cartesianSpeed;
                }
                ms.AppendMessage("SetCartesian Speed for motion to: " + motionTarget.CartesianSpeed, MessageLevel.Warning);
            }
            motionTarget.MotionType = motionType;
            motionTarget.SetAllJointValues(jointAngleCollection);
            motionTarget.UseJointValues = true;

            return motionTarget;
        }

        private void interpolatePlannedMotion (IRobot robot)
        {
            if (robotList[robot].motionPlan.getLastResult() != null)
            {
                IMotionInterpolator motionInterpolator = robot.RobotController.CreateMotionInterpolator();

                foreach (VectorOfDouble jointAngleCollection in robotList[robot].motionPlan.getLastResult())
                {
                    IMotionTarget motionTarget = createIMotionTargetForJointAngleConfiguration(robot, kukaSorted(jointAngleCollection), MotionType.Linear, robotList[robot].maxCartesianSpeed);
                    motionInterpolator.AddTarget(motionTarget);
                }

                //ms.AppendMessage("GetCycleTime at " + motionInterpolator.Targets.Count + ": " + motionInterpolator.GetCycleTimeAt(motionInterpolator.Targets.Count-1), MessageLevel.Warning);

                if (robotList[robot].currentMotionEndTime == 0.0)
                {
                    robotList[robot].currentMotionStartTime = app.Simulation.Elapsed;
                    robotList[robot].currentMotionEndTime = motionInterpolator.GetCycleTimeAt(robotList[robot].motionPlan.getLastResult().Count-1);
                    ms.AppendMessage("Motion for robot " + robot.Name + " will end at: " + motionInterpolator.GetCycleTimeAt(robotList[robot].motionPlan.getLastResult().Count - 1), MessageLevel.Warning);
                }
                
                IMotionTarget refMotionTarget = robot.RobotController.CreateTarget();

                motionInterpolator.Interpolate(app.Simulation.Elapsed - robotList[robot].currentMotionStartTime, ref refMotionTarget);

                IMotionTester motionTester = robot.RobotController.GetMotionTester();
                motionTester.CurrentTarget = refMotionTarget;

                if (app.Simulation.Elapsed > robotList[robot].currentMotionEndTime)
                {

                    robotList[robot].currentMotionStartTime = 0.0;
                    robotList[robot].currentMotionEndTime = 0.0;
                    robotList[robot].motionPlan.invalidatePlanner();
                }

             }
        }

        void OutputOnHumanDetected(object sender, LaserScannerHumanDetectedEventArgs args)
        {
            ms.AppendMessage("Detected Human with moveSpeed: " +args.MoveSpeed, MessageLevel.Warning);
            try
            {
                IRobot robot = args.Robot.GetRobot();
                robotList[robot].allowedCartesianSpeed = robotList[robot].speedCalculator.GetAllowedVelocity(BodyPart.Chest, args.MoveSpeed, 10.0);
                ms.AppendMessage("Allowed Speed from SSM: " + robotList[robot].allowedCartesianSpeed, MessageLevel.Warning);
                robotList[robot].currentSeperationDistance = robotList[robot].seperationCalculator.GetSeparationDistance(args.MoveSpeed, robotList[robot].currentCartesianSpeed);
                ms.AppendMessage("SeperationDistance: " + robotList[robot].currentSeperationDistance, MessageLevel.Warning);
            } catch (NullReferenceException e)
            {
                ms.AppendMessage("Laser Scanner sent event without robot component", MessageLevel.Error);
                ms.AppendMessage(e.ToString(), MessageLevel.Error);
            }

        }
        
        //TODO: Change the registration of laser scanners
        void World_ComponentRemoving(object sender, ComponentRemovingEventArgs args)
        {
            LaserScanner found = null;
            foreach (LaserScanner laser_scanner in laser_scanners)
            {
                if (laser_scanner.GetComponent().Equals(args.Component))
                {
                    // found
                    found = laser_scanner;
                }
            }
            if (found != null)
            {
                ms.AppendMessage("Removing LaserScanner from RobotController!", MessageLevel.Warning);
                found.OnHumanDetected -= OutputOnHumanDetected;
                laser_scanners.Remove(found);
                ms.AppendMessage("LaserScanners: " + laser_scanners.Count(), MessageLevel.Warning);
            }
        }
        void World_ComponentAdded(object sender, ComponentAddedEventArgs args)
        {
            IProperty prop = args.Component.GetProperty("LaserScanner");
            if (prop != null && (prop.Value is bool) && (bool)prop.Value)
            {
                ms.AppendMessage("Adding LaserScanner from RobotController!", MessageLevel.Warning);
                LaserScanner ls = new LaserScanner(args.Component, app);
                ls.OnHumanDetected += OutputOnHumanDetected;
                laser_scanners.Add(ls);
                ms.AppendMessage("LaserScanners: " + laser_scanners.Count(), MessageLevel.Warning);
            }
        }
        

    }
}
