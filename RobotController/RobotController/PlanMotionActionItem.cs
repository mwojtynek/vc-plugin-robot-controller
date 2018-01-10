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
    public class PlanMotionActionItem : ActionItem
    {
        [Import]
        private Lazy<IApplication> app = null;

        IMessageService ms = null;
        

        public PlanMotionActionItem() : base("StartMovement")
        {
            ms = IoC.Get<IMessageService>();
            ms.AppendMessage("Constructor of PlanMotion Action Item called", MessageLevel.Warning);
        }
        //public PlanMotionActionItem() : base("PlanMotion")
        //{
        //    ms = IoC.Get<IMessageService>();
        //    ms.AppendMessage("Constructor of PlanMotion Action Item called", MessageLevel.Warning);
        //}

        public override void Execute(PropertyCollection args)
        {
            ms.AppendMessage("Executing PlanMotion...", MessageLevel.Warning);

            //TODO: Fix the hard index access or at least print out a message if input was wrong
            String robotName = (String)args.GetByIndex(0).Value;
            IRobot robot = app.Value.World.FindComponent(robotName).GetRobot();

            MotionPlanningManager mpm = new MotionPlanningManager();

            MotionPlan motionPlan = mpm.InitializeMotionPlanner(robot,
                "S:\\git\\rosi.plugin.pathplanner\\robot_descriptions\\urdf\\lbr_iiwa_14_r820.urdf",
                "base_link",
                "tool0",
                "S:/git/rosi.plugin.pathplanner/cage-models/fleximir-model-even-less-detailed.stl");

            RobotController.getInstance().AddMotionPlan(robot, motionPlan);
                
            String startFrameName = args.GetByIndex(1).Value.ToString();

            String goalFrameName = args.GetByIndex(2).Value.ToString();

            VectorOfDoubleVector resultMotion = mpm.planMotion(robot, motionPlan,  startFrameName, goalFrameName);

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
