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
    [Export(typeof(IActionItem))]
    public class StartMovementActionItem : ActionItem
    {
        [Import]
        private Lazy<IApplication> app = null;

        IMessageService ms = null;
        IRobot robot = null;
        MotionPlanningManager mpm = null;
        MotionPlan motionPlan = null;

        public StartMovementActionItem() : base("StartMovement")
        {
            ms = IoC.Get<IMessageService>();
            ms.AppendMessage("Constructor of StartMovement Action Item called", MessageLevel.Warning);
        }

        Dictionary<ISimComponent, MotionPlan> motionPlanCollection = new Dictionary<ISimComponent, MotionPlan>();
        public override void Execute(PropertyCollection args)
        {
            //TODO: Fix the hard index access or at least print out a message if input was wrong
            String robotName = (String)args.GetByIndex(0).Value;
            ISimComponent robotParent = app.Value.World.FindComponent(robotName);
            robot = robotParent.GetRobot();

            String startFrameName = (String)args.GetByIndex(1).Value;
            String goalFrameName = (String)args.GetByIndex(2).Value;
            int maxAllowedCartesianSpeed = (int)args.GetByIndex(3).Value;
            String payload = (String)args.GetByIndex(4).Value;

            RobotSection parameter = ConfigReader.readSection(robotName);

            RobotController.getInstance().setMaxAllowedCartesianSpeed(robot, maxAllowedCartesianSpeed);
            motionPlanCollection.Clear();
            if (!motionPlanCollection.TryGetValue(robotParent, out motionPlan))
            {
                mpm = new MotionPlanningManager();
                motionPlan = mpm.InitializeMotionPlanner(robot,
                                                        //RobotParameters.UrdfFile,
                                                        parameter.urdfFile.Path,
                                                        RobotParameters.KinStart, RobotParameters.KinEnd,
                                                        parameter.obsFile.Path
                                                        //RobotParameters.obstacleModelFile
                                                        );
                motionPlanCollection.Add(robotParent, motionPlan);
                ms.AppendMessage("Created new motionPlan for " + robotName, MessageLevel.Warning);
            }
            VectorOfDoubleVector resultMotion = mpm.planMotion(robot, motionPlan, startFrameName, goalFrameName);
            if (resultMotion != null)
            {
                IBehavior beh = robot.Component.FindBehavior("MovementFinished");
                if (beh != null && beh is IStringSignal)
                {
                    IStringSignal movementFinished = (IStringSignal)robot.Component.FindBehavior("MovementFinished");
                    movementFinished.Value = ""; // empty string means no payload contained yet

                    MotionInterpolator inp = motionPlan.getMotionInterpolator();
                    inp.setMaxJointAcceleration(6.5);
                    inp.setMaxJointVelocity(75.0);

                    RobotController.getInstance().AddMotionPlan(robot, payload, motionPlan);
                }
                else
                {
                    ms.AppendMessage("\"MovementFinished\" behavior was either null or not of type IStringSignal. Abort!", MessageLevel.Warning);
                }
            }
        }

    }
}
