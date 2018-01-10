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
        int counter = 1;

        public StartMovementActionItem() : base("StartMovement")
        {
            ms = IoC.Get<IMessageService>();
            ms.AppendMessage("Constructor of StartMovement Action Item called", MessageLevel.Warning);
        }
        
        public override void Execute(PropertyCollection args)
        {
            //ms.AppendMessage("Executing StartMovement with counter value" + counter, MessageLevel.Warning);

            //TODO: Fix the hard index access or at least print out a message if input was wrong
            String robotName = (String)args.GetByIndex(0).Value;
            robot = app.Value.World.FindComponent(robotName).GetRobot();

            mpm = new MotionPlanningManager();

            motionPlan = mpm.InitializeMotionPlanner(robot,
                "S:\\git\\rosi.plugin.pathplanner\\robot_descriptions\\urdf\\lbr_iiwa_14_r820.urdf",
                "base_link",
                "tool0",
                "S:/git/rosi.plugin.pathplanner/cage-models/fleximir-model-even-less-detailed.stl");
            
            if (counter == 5)
            {
                VectorOfDoubleVector resultMotion = mpm.planMotion(robot, motionPlan, counter.ToString(), "1");
                counter = 1;
            }
            else
            {
                VectorOfDoubleVector resultMotion = mpm.planMotion(robot, motionPlan, counter.ToString(), (counter + 1).ToString());
                counter++;
            }

            IBooleanSignal movementFinished = (IBooleanSignal)robot.Component.FindBehavior("MovementFinished");
            movementFinished.Value = false;

            RobotController.getInstance().AddMotionPlan(robot, motionPlan);

            //ms.AppendMessage("Executed StartMovement.", MessageLevel.Warning);
        }
        
    }
}
