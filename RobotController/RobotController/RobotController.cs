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
        private const double TICK_INTERVAL = 1.0/30.0;
        private IApplication app = null;
        private IMessageService ms = null;
        private static RobotController instance = null;
        private IStatisticsManager statisticsManager = null;
        private ISamplingTimer timer = null;
        private MotionPlanningManager MotionPlanningManagerInstance { get; }
        private List<ILaserScanner> laser_scanners = new List<ILaserScanner>();
        private Dictionary<IRobot, RobotParameters> robotList = new Dictionary<IRobot, RobotParameters>();
        private MotionInterpolation MotionInterpolationInstance = null;
        private ISimComponent human = null;
        private IProperty humanAngleIndicatorZRotation = null;

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

        /// <summary>
        /// The provided robot is added to the internal list which is maintained by this controller plugin.
        /// Initialization of variables and speed and separation monitoring happens.
        /// </summary>
        /// <param name="robot"></param>The robot that should be registered.
        /// <returns></returns>Returns true if registration was successful and false otherwise.
        public bool RegisterRobot(IRobot robot)
        {
            if (robotList.ContainsKey(robot))
            {
                robotList.Remove(robot);
            }
            robotList.Add(robot, new RobotParameters());
            robotList[robot].speedCalculator = new SpeedCalculator(30, 0);
            robotList[robot].seperationCalculator = new SeparationCalculator(1.0, 1.0, 10.0, 10.0, 0.1);
            robotList[robot].maxCartesianSpeed = 480.0;
            robotList[robot].allowedCartesianSpeed = 480.0;

            VisualizeSeperationDistance(robot, 500.0);

            return true;
        }

        /// <summary>
        /// Visualization of Separation Distance. A cylinder feature is added to the robot if not already present.
        /// Initial values are set.
        /// </summary>
        /// <param name="robot"></param>The robot for which the separation distance should be visualized.
        /// <param name="initialRadius"></param>The initial radius for the cylinder.
        private void VisualizeSeperationDistance(IRobot robot, double initialRadius)
        {
            if (robot.Component.FindFeature("SeparationVisualization") == null)
            {
                ITransformFeature transformFeature = robot.Component.RootNode.RootFeature.CreateFeature<ITransformFeature>();
                transformFeature.GetProperty("Expression").Value = "Tz(-" + robot.Component.TransformationInWorld.Pz + ").Ty(" 
                    + robot.Component.FindNode("mountplate").TransformationInWorld.Py + ").Tx(" + robot.Component.FindNode("mountplate").TransformationInWorld.Px + ")";
                transformFeature.SetName("SeparationVisualizationTransformation");

                ICylinderFeature seperationVisualization = robot.Component.FindFeature("SeparationVisualizationTransformation").CreateFeature<ICylinderFeature>();
                // true would remove the top and bottom of the cylinder, but backfaces of the inside of the cylinder are not rendered
                //seperationVisualization.GetProperty("Caps").Value = false; 
                seperationVisualization.GetProperty("Height").Value = "3000.0";
                seperationVisualization.GetProperty("Sections").Value = "36.0";
                seperationVisualization.GetProperty("Radius").Value = initialRadius.ToString();
                seperationVisualization.GetProperty("Material").Value = app.FindMaterial("transp_yellow", false);
                seperationVisualization.SetName("SeparationVisualization");
            } 
        }

        /// <summary>
        /// Simple function to update the size of the cylinder which visualizes the current separation distance.
        /// </summary>
        /// <param name="robot"></param>The robot for which the update should be made.
        private void UpdateVisualizationDistance(IRobot robot)
        {
            if (robot.Component != null && robot.Component.FindFeature("SeparationVisualization") != null)
            {
                ITransformFeature transformFeature = (ITransformFeature) robot.Component.FindFeature("SeparationVisualizationTransformation");
                transformFeature.GetProperty("Expression").Value = "Tz(-" + robot.Component.TransformationInWorld.Pz + ").Ty("
                    + (robot.Component.TransformationInWorld.Py + robot.Component.FindNode("mountplate").TransformationInWorld.Py) + ").Tx(" + (robot.Component.TransformationInWorld.Px + robot.Component.FindNode("mountplate").TransformationInWorld.Px) + ")";
                transformFeature.SetName("SeparationVisualizationTransformation");

                ICylinderFeature cylinder = (ICylinderFeature) robot.Component.FindFeature("SeparationVisualization");
                cylinder.GetProperty("Radius").Value = 
                    robotList[robot].currentSeperationDistance.ToString();
                cylinder.Rebuild();
            } else
            {
                ms.AppendMessage("UpdateVisualizationDistance: Failed to find robot component!", MessageLevel.Warning);
            }
        }

        public void setMaxAllowedCartesianSpeed(IRobot robot, int maxspeed)
        {
            RobotParameters param = robotList[robot];
            param.allowedCartesianSpeed = maxspeed;
            if(param.motionPlan != null)
            {
                param.motionPlan.getMotionInterpolator().setCartesianSpeedLimit(maxspeed);
            }
        }

        /// <summary>
        /// Adds a new motion plan for a specific robot which would come from path planning.
        /// This motion plan contains at least two or more (the intermediate) joint angle configuration to reach the goal position.
        /// Furthermore motion interpolator and motion tester are initialized and the motion plan from path planning
        /// is subdived.
        /// </summary>
        /// <param name="robot"></param>The robot to which the motion plan belongs and for which it should be saved.
        /// <param name="motionPlan"></param>
        public void AddMotionPlan(IRobot robot, String payloadOnFinishMovement, MotionPlan motionPlan)
        {
            try
            {
                robotList[robot].motionPlan = motionPlan;
                robotList[robot].currentMotionStartTime = app.Simulation.Elapsed;
                robotList[robot].payloadOnFinishMovement = payloadOnFinishMovement;
                
                ms.AppendMessage("New motion plan ("+ motionPlan.getLastResult().Count + ") set for robot "+robot.Name+" starting at "+app.Simulation.Elapsed, MessageLevel.Warning);
            }
            catch (KeyNotFoundException e)
            {
                ms.AppendMessage("Motion plan could not be added, maybe robot was not registred?", MessageLevel.Warning);
                ms.AppendMessage(e.ToString(), MessageLevel.Error);
            }
        }

        /// <summary>
        /// Plugin Initialization. Only called once at program start.
        /// </summary>
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

        /// <summary>
        /// Initialization at Simulation startup. Each time the play button is pressed, this is triggered.
        /// </summary>
        /// <param name="sender"></param>The source of the simulation start event.
        /// <param name="e"></param>The simulation start event.
        public void SimulationStarted(object sender, EventArgs e)
        {
            ms.AppendMessage("Simulation Started", MessageLevel.Warning);
            timer = statisticsManager.CreateTimer(RegularTick, TICK_INTERVAL);
            timer.StartStopTimer(true);
            if(app.World.FindComponent("WorksHuman") != null)
            {
                human = app.World.FindComponent("WorksHuman");
                if(human.GetProperty("AngleIndicatorZRotation") != null)
                {
                    humanAngleIndicatorZRotation = human.GetProperty("AngleIndicatorZRotation");
                }
            }
        }

        /// <summary>
        /// Cleanup for the reset of the simulation. Everything that has to be cleared for the next run,
        /// should be cleared.
        /// </summary>
        /// <param name="sender"></param>The source of the simulation stopped event.
        /// <param name="e"></param>The simulation stopped event.
        public void SimulationStopped(object sender, EventArgs e)
        {
            ms.AppendMessage("Simulation Stopped", MessageLevel.Warning);
            if (timer != null)
            {
                timer.StartStopTimer(false);
                foreach (IRobot robot in robotList.Keys) {
                    robotList[robot].currentCartesianSpeed = 0.0;
                    robotList[robot].currentMotionStartTime = 0.0;
                    robotList[robot].motionPlan = null;
                    //robotList[robot].seperationCalculator = null;
                    //robotList[robot].speedCalculator = null;
                }
            }
        }

        /// <summary>
        /// The general update loop which has to trigger everything that should be computed for each iteration.
        /// </summary>
        /// <param name="sender"></param>The source of the tick event.
        /// <param name="e"></param>The tick event itself.
        public void RegularTick(object sender, EventArgs e)
        {
            foreach (ILaserScanner laserScanner in laser_scanners)
            {
                laserScanner.Scan();
            }

            foreach (IRobot robot in robotList.Keys)
            {
                UpdateVisualizationDistance(robot);
                //MotionInterpolationInstance.InterpolatePlannedMotion(robot, ref robotList, app.Simulation.Elapsed);

                MotionInterpolationInstance.CalculateCurrentRobotSpeed(robot, ref robotList, TICK_INTERVAL, human.TransformationInWorld.GetP()); //robotList[robot].closestHumanWorldPosition

                //MotionInterpolationInstance.CalculateCurrentRobotSpeed(robot, ref robotList, TICK_INTERVAL, app.World.FindComponent("WorksHuman").TransformationInWorld.GetP()); //robotList[robot].closestHumanWorldPosition

                RobotParameters param = robotList[robot];
                if (param.motionPlan == null)
                    continue;
                MotionInterpolator mp = param.motionPlan.getMotionInterpolator();
                if (param.motionPlan != null && !param.motionPlan.getMotionInterpolator().motionDone()){
                    VectorOfDouble result = param.motionPlan.getMotionInterpolator().interpolate_tick(TICK_INTERVAL);

                    robot.RobotController.InvalidateKinChains();
                    robot.RobotController.SetJointValues(MotionInterpolation.KukaSorted(result));
                } else {
                    // set movement done!
                    IBehavior movementFinished = (IBehavior)robot.Component.FindBehavior("MovementFinished");
                    if(movementFinished is IStringSignal)
                    {
                        IStringSignal movementFinishedStringSignal = (IStringSignal)movementFinished;
                        
                        if (!String.Equals(movementFinishedStringSignal.Value, robotList[robot].payloadOnFinishMovement))
                        {
                            movementFinishedStringSignal.Value = robotList[robot].payloadOnFinishMovement;
                        }
                    } else {
                        ms.AppendMessage("\"MovementFinished\" behavior was null or not of type IStringSignal. Abort!", MessageLevel.Warning);
                    }

                }
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
                    robotList[robot].closestHumanWorldPosition = args.HumanPosition;
                    robotList[robot].angleToHuman = args.Angle;

                    double humanDistance = Math.Abs(Vector3.Subtract(robotList[robot].closestHumanWorldPosition, robot.Component.TransformationInWorld.GetP()).Length);
                    if (humanDistance < robotList[robot].currentSeperationDistance)
                    {
                        robotList[robot].allowedCartesianSpeed = 0.0;
                    } else
                    {
                        robotList[robot].allowedCartesianSpeed = robotList[robot].speedCalculator.GetAllowedVelocity(BodyPart.Chest, args.MoveSpeed, 1.0);
                    }
                    if(robotList[robot].motionPlan != null)
                    {
                        robotList[robot].motionPlan.getMotionInterpolator().setCartesianSpeedLimit(robotList[robot].allowedCartesianSpeed);
                        ms.AppendMessage("Allowed Cartesian Speed: " + robotList[robot].allowedCartesianSpeed, MessageLevel.Error);
                    }

                    humanAngleIndicatorZRotation.Value = args.Angle * (180 / Math.PI) + human.TransformationInWorld.GetAxisAngle().W;
                    
                    
                    ms.AppendMessage("Angle from Human to robot: " + (args.Angle * (180 / Math.PI))+ " humanAngleIndicatorZRotation.Value: "+ humanAngleIndicatorZRotation.Value, MessageLevel.Warning);
                    //ms.AppendMessage("Allowed Speed from SSM: " + robotList[robot].allowedCartesianSpeed, MessageLevel.Warning);

                    //Gewichtetes Update der Separation Daten um Ausschläge ("Sensorrauschen") zu vermeiden
                    robotList[robot].currentSeperationDistance = 0.2 * robotList[robot].seperationCalculator.GetSeparationDistance(args.MoveSpeed, robotList[robot].currentCartesianSpeed)
                        + 0.8 * robotList[robot].oldSeparationDistance;

                    //ms.AppendMessage(app.Simulation.Elapsed + ";" + args.MoveSpeed + ";" + robotList[robot].currentCartesianSpeed + ";" + robotList[robot].currentSeperationDistance, MessageLevel.Error);

                    robotList[robot].oldSeparationDistance = robotList[robot].currentSeperationDistance;

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
