using Caliburn.Micro;
using Rosi.Components.Sensors;
using RosiTools.PluginManager;
using SpeedAndSeparationMonitoring;
using System;
using System.Collections.Generic;
using System.ComponentModel.Composition;
using System.Linq;
using System.Threading;
using VisualComponents.Create3D;
using VisualComponents.UX.Shared;

namespace RobotController
{
    [Export(typeof(IPlugin))]
    public class RobotController : IPlugin
    {
        private const double TICK_INTERVAL = 1.0;
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

        ReaderWriterLockSlim robotListLock = new ReaderWriterLockSlim();

        //Create Constructor for class
        [ImportingConstructor]
        public RobotController([Import(typeof(IApplication))] IApplication app)
        {
            this.app = app;
            instance = this;

            MotionPlanningManagerInstance = new MotionPlanningManager();
            MotionInterpolationInstance = new MotionInterpolation();

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
            try
            {
                robotListLock.EnterWriteLock();
                if (robotList.ContainsKey(robot))
                {
                    robotList.Remove(robot);
                }
                robotList.Add(robot, new RobotParameters());
                robotList[robot].speedCalculator = new SpeedCalculator(30, 0);
                robotList[robot].seperationCalculator = new SeparationCalculator(1.0, 1.0, 10.0, 10.0, 0.1);
                robotList[robot].maxCartesianSpeed = 480.0;
                robotList[robot].allowedCartesianSpeed = 480.0;
            }
            finally
            {
                robotListLock.ExitWriteLock();
            }

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
            if (robot != null && robot.IsValid && app.World.FindComponent("SeparationVisualization_" + robot.Name) == null)
            {
                ISimComponent component = app.World.CreateComponent("SeparationVisualization_" + robot.Name);
                ISimNode node = robot.Component.FindNode("mountplate");

                Matrix matrix = component.TransformationInReference;
                matrix.SetP(new Vector3(node.TransformationInWorld.Px, node.TransformationInWorld.Py, 0.0));

                component.TransformationInReference = matrix;

                ICylinderFeature seperationVisualization = component.RootNode.RootFeature.CreateFeature<ICylinderFeature>();
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
            if (robot != null && robot.IsValid && app.World.FindComponent("SeparationVisualization_" + robot.Name) != null)
            {
                ISimComponent comp = app.World.FindComponent("SeparationVisualization_" + robot.Name);
                ISimNode node = robot.Component.FindNode("mountplate");

                Matrix matrix = comp.TransformationInReference;
                matrix.SetP(new Vector3(node.TransformationInWorld.Px, node.TransformationInWorld.Py, 0.0));

                comp.TransformationInReference = matrix;
                
                ICylinderFeature cylinder = (ICylinderFeature) app.World.FindComponent("SeparationVisualization_" + robot.Name).FindFeature("SeparationVisualization");
                if(robotList[robot].currentSeperationDistance <= 100)
                {
                    cylinder.GetProperty("Radius").Value = "100";
                } else
                {
                    cylinder.GetProperty("Radius").Value =
                        robotList[robot].currentSeperationDistance.ToString();                    
                }
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
            if (param.motionPlan != null)
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
                robotListLock.EnterWriteLock();
                try
                {
                    robotList[robot].motionPlan = motionPlan;
                    robotList[robot].currentMotionStartTime = app.Simulation.Elapsed;
                    robotList[robot].payloadOnFinishMovement = payloadOnFinishMovement;

                    ms.AppendMessage("New motion plan (" + motionPlan.getLastResult().Count + ") set for robot " + robot.Name + " starting at " + app.Simulation.Elapsed, MessageLevel.Warning);
                }
                catch (KeyNotFoundException e)
                {
                    ms.AppendMessage("Motion plan could not be added, maybe robot was not registred?", MessageLevel.Warning);
                    ms.AppendMessage(e.ToString(), MessageLevel.Error);
                }
            }
            finally
            {
                robotListLock.ExitWriteLock();
            }
        }

        /// <summary>
        /// Plugin Initialization. Only called once at program start.
        /// </summary>
        public void Initialize()
        {
            if (!(IoC.Get<IPluginManager>().queryPlugin(PluginCategory.Controller))) {
                return;
            }
            ConfigReader.init();


            this.app.Simulation.SimulationStarted += SimulationStarted;
            this.app.Simulation.SimulationStopped += SimulationStopped;
            //IoC.Get<ISimulationService>().PropertyChanged += JointConfigurationChanged;
            ms = IoC.Get<IMessageService>();
            statisticsManager = this.app.StatisticsManager;
            statisticsManager.IsEnabled = true;
            statisticsManager.StatisticsInterval = TICK_INTERVAL;
            // Must be at least Warning, Info level is not printed
            ms.AppendMessage("DynamicRobotControl Plugin started initialization...", MessageLevel.Warning);

            app.World.ComponentAdded += World_ComponentAdded;
            app.World.ComponentRemoving += World_ComponentRemoving;
            IoC.Get<ISimulationService>().PropertyChanged += SimulationPropertyChanged;
        }

        /// <summary>
        /// Initialization at Simulation startup. Each time the play button is pressed, this is triggered.
        /// </summary>
        /// <param name="sender"></param>The source of the simulation start event.
        /// <param name="e"></param>The simulation start event.
        public void SimulationStarted(object sender, EventArgs e)
        {
            ms.AppendMessage("Simulation Started", MessageLevel.Warning);
            //timer = statisticsManager.CreateTimer(RegularTick, TICK_INTERVAL);
            //timer.StartStopTimer(true);

            if (app.World.FindComponent("WorksHuman") != null)
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
                try
                {
                    robotListLock.EnterWriteLock();

                    foreach (IRobot robot in robotList.Keys)
                    {
                        robotList[robot].currentCartesianSpeed = 0.0;
                        robotList[robot].currentMotionStartTime = 0.0;
                        robotList[robot].motionPlan = null;
                        robotList[robot].LastTimeElapsed = 0.0;
                        //robotList[robot].seperationCalculator = null;
                        //robotList[robot].speedCalculator = null;
                    }
                }
                finally
                {
                    robotListLock.ExitWriteLock();
                }
            }
        }


        //double lastTime = 0.0;
        public void SimulationPropertyChanged(object sender, EventArgs e)
        {
            //double deltaTime = app.Simulation.Elapsed - lastTime;
            //lastTime = app.Simulation.Elapsed;
            
            foreach (ILaserScanner laserScanner in laser_scanners)
            {
                laserScanner.Scan();
            }

            double appElapsed = app.Simulation.Elapsed;
            try
            {
                robotListLock.EnterReadLock();
                foreach (IRobot robot in robotList.Keys)
                {
                    if (!robot.IsValid) continue;
                    UpdateVisualizationDistance(robot);
                    double deltaTime = appElapsed - robotList[robot].LastTimeElapsed;

                    MotionInterpolationInstance.CalculateCurrentRobotSpeed(robot, ref robotList, TICK_INTERVAL, human.TransformationInWorld.GetP()); //robotList[robot].closestHumanWorldPosition
                    RobotParameters param = robotList[robot];
                    if (param.motionPlan == null)
                        continue;
                    MotionInterpolator mp = param.motionPlan.getMotionInterpolator();
                    if (param.motionPlan != null && !param.motionPlan.getMotionInterpolator().motionDone())
                    {
                        VectorOfDouble result = param.motionPlan.getMotionInterpolator().interpolate_tick(deltaTime);

                        robot.RobotController.InvalidateKinChains();

                        if (robot.RobotController.Joints.Count == 7)
                        {
                            robot.RobotController.SetJointValues(MotionInterpolation.KukaSorted(result));
                        }
                        else
                        {
                            double[] firstJointAngleCollectionSorted = new double[result.Count];
                            firstJointAngleCollectionSorted[0] = result.ElementAt(0); //A1
                            firstJointAngleCollectionSorted[1] = result.ElementAt(1); //A2
                            firstJointAngleCollectionSorted[2] = result.ElementAt(2); //A3
                            firstJointAngleCollectionSorted[3] = result.ElementAt(3); //A4
                            firstJointAngleCollectionSorted[4] = result.ElementAt(4); //A5
                            firstJointAngleCollectionSorted[5] = result.ElementAt(5); //A6
                            robot.RobotController.SetJointValues(firstJointAngleCollectionSorted);
                        }

                    }
                    else
                    {
                        // set movement done!
                        IBehavior movementFinished = (IBehavior)robot.Component.FindBehavior("MovementFinished");
                        if (movementFinished is IStringSignal)
                        {
                            IStringSignal movementFinishedStringSignal = (IStringSignal)movementFinished;

                            if (!String.Equals(movementFinishedStringSignal.Value, robotList[robot].payloadOnFinishMovement))
                            {
                                movementFinishedStringSignal.Value = robotList[robot].payloadOnFinishMovement;
                            }
                        }
                        else
                        {
                            ms.AppendMessage("\"MovementFinished\" behavior was null or not of type IStringSignal. Abort!", MessageLevel.Warning);
                        }

                    }
                    robotList[robot].LastTimeElapsed = appElapsed;
                }
            }
            finally
            {
                robotListLock.ExitReadLock();
            }

        }

        void OutputOnHumanDetected(object sender, LaserScannerHumanDetectedEventArgs args)
        {
            //ms.AppendMessage("Detected Human with moveSpeed: " +args.MoveSpeed, MessageLevel.Warning);
            try
            {
                robotListLock.EnterWriteLock();
                IRobot robot = args.Robot;
                if (robotList.ContainsKey(robot) && robot.IsValid)
                {
                    robotList[robot].closestHumanWorldPosition = args.HumanPosition;
                    robotList[robot].angleToHuman = args.Angle;

                    //Gewichtetes Update der Separation Daten um Ausschläge ("Sensorrauschen") zu vermeiden
                    robotList[robot].currentSeperationDistance = 0.2 * robotList[robot].seperationCalculator.GetSeparationDistance(args.MoveSpeed, robotList[robot].currentCartesianSpeed)
                        + 0.8 * robotList[robot].oldSeparationDistance;


                    double humanDistance = Math.Abs(Vector3.Subtract(robotList[robot].closestHumanWorldPosition, robot.Component.TransformationInWorld.GetP()).Length);
                    if (humanDistance < robotList[robot].currentSeperationDistance)
                    {
                        /*setMaxAllowedCartesianSpeed(robot, 0); //robotList[robot].allowedCartesianSpeed = 0.0;
                    } else
                    {*/
                        int result = Convert.ToInt32(robotList[robot].speedCalculator.GetAllowedVelocity(BodyPart.Chest, args.MoveSpeed, 1.0));
                        ms.AppendMessage("Setting robot cartesian speed limit to " + result, MessageLevel.Warning);
                        setMaxAllowedCartesianSpeed(robot, result);
                    } else
                    {
                        setMaxAllowedCartesianSpeed(robot, 400);
                        ms.AppendMessage("Setting robot cartesian speed limit to 400", MessageLevel.Warning);
                    }
                    if (robotList[robot].motionPlan != null)
                    {
                        robotList[robot].motionPlan.getMotionInterpolator().setCartesianSpeedLimit(robotList[robot].allowedCartesianSpeed);
                        //ms.AppendMessage("Allowed Cartesian Speed: " + robotList[robot].allowedCartesianSpeed, MessageLevel.Error);
                    }

                    //humanAngleIndicatorZRotation.Value = args.Angle * (180 / Math.PI) + human.TransformationInWorld.GetAxisAngle().W;
                    
                    
                    //ms.AppendMessage("Angle from Human to robot: " + (args.Angle * (180 / Math.PI))+ " humanAngleIndicatorZRotation.Value: "+ humanAngleIndicatorZRotation.Value, MessageLevel.Warning);
                    //ms.AppendMessage("Allowed Speed from SSM: " + robotList[robot].allowedCartesianSpeed, MessageLevel.Warning);

                   
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
            } finally
            {
                robotListLock.ExitWriteLock();
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
                ls.OnHumanLost += OutputOnHumanLost;
                laser_scanners.Add(ls);
                ms.AppendMessage("LaserScanners: " + laser_scanners.Count(), MessageLevel.Warning);
            }
        }

        
        private void OutputOnHumanLost(object sender, LaserScannerHumanLostEventArgs e)
        {
            if(e.HumansLeftCount == 0)
            {
                try
                {
                    robotListLock.EnterWriteLock();
                    if (robotList.ContainsKey(e.Robot))
                    {
                        // we can reset the cartesian speed of the robot
                        ms.AppendMessage(e.Robot.Name + " restored cartesian speed to " + robotList[e.Robot].maxCartesianSpeed + "!", MessageLevel.Warning);
                        robotList[e.Robot].allowedCartesianSpeed = robotList[e.Robot].maxCartesianSpeed;
                    }
                    else
                    {
                        ms.AppendMessage("OutputOnHumanLost failed to find referenced robot!", MessageLevel.Warning);
                    }
                }
                finally
                {
                    robotListLock.ExitWriteLock();
                }

            }
        }
    }
}
