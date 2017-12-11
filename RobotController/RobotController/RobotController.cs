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
        private IMessageService ms = null;
        private static RobotController instance = null;
        private double timeElapsedBase = 0.0;
        private IStatisticsManager statisticsManager = null;
        private ISamplingTimer timer = null;
        public MotionPlanningManager MotionPlanningManagerInstance { get; }

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

            foreach (IRobot robot in MotionPlanningManagerInstance.MotionPlanners.Keys)
            {
                interpolatePlannedMotion(robot);
            }
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
            
            if (MotionPlanningManagerInstance.MotionPlanners[robot].getLastResult() != null)
            {
                double[] startJointAngleCollection = kukaSorted(MotionPlanningManagerInstance.MotionPlanners[robot].getLastResult().First());

                IMotionTarget startMotionTarget = robot.RobotController.CreateTarget();
                startMotionTarget.CartesianSpeed = 1.0;
                startMotionTarget.AngularSpeed = 1.0;
                // JointSpeed Value from 0-100
                startMotionTarget.JointSpeed = 1.0;
                startMotionTarget.MotionType = MotionType.Joint;
                startMotionTarget.SetAllJointValues(startJointAngleCollection);
                startMotionTarget.UseJointValues = true;
                
                double[] goalJointAngleCollection = kukaSorted(MotionPlanningManagerInstance.MotionPlanners[robot].getLastResult().Last());

                IMotionTarget goalMotionTarget = robot.RobotController.CreateTarget();
                goalMotionTarget.CartesianSpeed = 1.0;
                goalMotionTarget.AngularSpeed = 1.0;
                // JointSpeed Value from 0-100
                goalMotionTarget.JointSpeed = 1.0;
                goalMotionTarget.MotionType = MotionType.Joint;
                goalMotionTarget.SetAllJointValues(goalJointAngleCollection);
                goalMotionTarget.UseJointValues = true;
                
                IMotionInterpolator motionInterpolator = robot.RobotController.CreateMotionInterpolator();
                motionInterpolator.AddTarget(startMotionTarget);
                motionInterpolator.AddTarget(goalMotionTarget);

                //ms.AppendMessage("GetCycleTime at " + motionInterpolator.Targets.Count + ": " + motionInterpolator.GetCycleTimeAt(motionInterpolator.Targets.Count-1), MessageLevel.Warning);

                if (robot.Component.GetProperty("motionEndTime") != null && robot.Component.GetProperty("motionEndTime").Equals("0.0"))
                {
                    robot.Component.SetPropertyValues(Enumerable.Repeat<string>("motionStartTime", 1), Enumerable.Repeat<string>(app.Simulation.Elapsed.ToString(), 1));
                    robot.Component.SetPropertyValues(Enumerable.Repeat<string>("motionEndTime", 1), Enumerable.Repeat<string>(motionInterpolator.GetCycleTimeAt(1).ToString(), 1));
                    ms.AppendMessage("Motion for robot " + robot.Name + " will end at: " + motionInterpolator.GetCycleTimeAt(1), MessageLevel.Warning);
                }
                
                IMotionTarget refMotionTarget = robot.RobotController.CreateTarget();

                motionInterpolator.Interpolate(app.Simulation.Elapsed- (double)robot.Component.GetProperty("motionStartTime").Value, ref refMotionTarget);
                

                IMotionTester motionTester = robot.RobotController.GetMotionTester();
                motionTester.CurrentTarget = refMotionTarget;

                if (app.Simulation.Elapsed > (double)robot.Component.GetProperty("motionEndTime").Value)
                {
                    
                    robot.Component.SetPropertyValues(Enumerable.Repeat<string>("motionStartTime", 1), Enumerable.Repeat<string>("0.0", 1));
                    robot.Component.SetPropertyValues(Enumerable.Repeat<string>("motionEndTime", 1), Enumerable.Repeat<string>("0.0", 1));
                    MotionPlanningManagerInstance.MotionPlanners[robot].invalidatePlanner();
                }

             }
        }

    }
}
