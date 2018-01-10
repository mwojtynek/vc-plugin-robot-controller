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
        private List<ILaserScanner> laser_scanners = new List<ILaserScanner>();
        private Dictionary<IRobot, RobotParameters> robotList = new Dictionary<IRobot, RobotParameters>();
        private MotionInterpolation MotionInterpolationInstance = null;

        //Create Constructor for class
        [ImportingConstructor]
        public RobotController([Import(typeof(IApplication))] IApplication app)
        {
            this.app = app;
            instance = this;
            MotionPlanningManagerInstance = new MotionPlanningManager();
            MotionInterpolationInstance = new MotionInterpolation();
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
                robotList.Remove(robot);
            }
            robotList.Add(robot, new RobotParameters());
            robotList[robot].speedCalculator = new SpeedCalculator(30, 0);
            robotList[robot].seperationCalculator = new SeparationCalculator(1.0, 1.0, 10.0, 10.0, 0.1);
            robotList[robot].maxCartesianSpeed = 240.0;
            robotList[robot].allowedCartesianSpeed = 240.0;
            
            return true;
        }

        public void AddMotionPlan(IRobot robot, MotionPlan motionPlan)
        {
            try
            {
                robotList[robot].motionPlan = motionPlan;
                //if (robotList[robot].motionInterpolator == null)
                //{
                    robotList[robot].motionInterpolator = robot.RobotController.CreateMotionInterpolator();
                    robotList[robot].motionList = new SortedList<double, IMotionTarget>();
                    robotList[robot].motionTester = robot.RobotController.GetMotionTester();
                    MotionInterpolationInstance.CalculateInterpolation(robot, ref robotList, 1.0, app.Simulation.Elapsed);
                //}
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
            timer = statisticsManager.CreateTimer(RegularTick, TICK_INTERVAL);
            timer.StartStopTimer(true);
        }

        public void SimulationStopped(object sender, EventArgs e)
        {
            ms.AppendMessage("Simulation Stopped", MessageLevel.Warning);
            if (timer != null)
            {
                timer.StartStopTimer(false);
                foreach (IRobot robot in robotList.Keys) {
                    robotList[robot].motionInterpolator = null;
                    robotList[robot].currentCartesianSpeed = 0.0;
                    robotList[robot].currentMotionStartTime = 0.0;
                    robotList[robot].currentMotionEndTime = 0.0;
                    robotList[robot].currentTarget = null;
                    robotList[robot].motionList = null;
                    robotList[robot].motionPlan = null;
                    robotList[robot].motionTester = null;
                    robotList[robot].seperationCalculator = null;
                    robotList[robot].speedCalculator = null;
                }
            }
        }

        public void RegularTick(object sender, EventArgs e)
        {
            //ms.AppendMessage("Current Time: " + app.Simulation.Elapsed, MessageLevel.Warning);

            foreach (IRobot robot in robotList.Keys)
            {
                MotionInterpolationInstance.InterpolatePlannedMotion(robot, ref robotList, app.Simulation.Elapsed);
                MotionInterpolationInstance.CalculateCurrentRobotSpeed(robot, ref robotList, robot.RobotController.ToolCenterPoint, TICK_INTERVAL);
            }

            foreach(ILaserScanner laserScanner in laser_scanners)
            {
                laserScanner.Scan();
            }
        }

        

        void OutputOnHumanDetected(object sender, LaserScannerHumanDetectedEventArgs args)
        {
            //ms.AppendMessage("Detected Human with moveSpeed: " +args.MoveSpeed, MessageLevel.Warning);
            try
            {
                IRobot robot = args.Robot;
                if (robotList.ContainsKey(robot))
                {
                    robotList[robot].allowedCartesianSpeed = robotList[robot].speedCalculator.GetAllowedVelocity(BodyPart.Chest, args.MoveSpeed, 1.0);
                    ms.AppendMessage("Allowed Speed from SSM: " + robotList[robot].allowedCartesianSpeed, MessageLevel.Warning);

                    robotList[robot].currentSeperationDistance = robotList[robot].seperationCalculator.GetSeparationDistance(args.MoveSpeed, robotList[robot].currentCartesianSpeed);
                    //ms.AppendMessage("SeperationDistance: " + robotList[robot].currentSeperationDistance, MessageLevel.Warning);

                    robotList[robot].closestDistanceToHuman = (args.HumanPosition - robot.RobotController.ToolCenterPoint.GetP()).Length;
                    //ms.AppendMessage("MeasuredDistance: " + robotList[robot].closestDistanceToHuman, MessageLevel.Warning);
                }
            } catch (NullReferenceException e)
            {
                ms.AppendMessage("Laser Scanner sent event without robot component", MessageLevel.Error);
                ms.AppendMessage(e.ToString(), MessageLevel.Error);
            } catch (KeyNotFoundException e)
            {
                ms.AppendMessage("Key not found", MessageLevel.Error);
                ms.AppendMessage(e.ToString(), MessageLevel.Error);

            }


        }
        
        //TODO: Change the registration of laser scanners
        void World_ComponentRemoving(object sender, ComponentRemovingEventArgs args)
        {
            ILaserScanner found = null;
            foreach (ILaserScanner laser_scanner in laser_scanners)
            {
                if (laser_scanner.Component.Equals(args.Component))
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
                LaserScanner_Virtual ls = new LaserScanner_Virtual(args.Component, app);
                ls.OnHumanDetected += OutputOnHumanDetected;
                laser_scanners.Add(ls);
                ms.AppendMessage("LaserScanners: " + laser_scanners.Count(), MessageLevel.Warning);
            }
        }
        

    }
}
