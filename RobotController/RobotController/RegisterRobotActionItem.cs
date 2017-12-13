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
    public class RegisterRobotActionItem : ActionItem
    {
        [Import]
        private Lazy<IApplication> app = null;

        IMessageService ms = null;

        public RegisterRobotActionItem() : base("RegisterRobot")
        {
            ms = IoC.Get<IMessageService>();
            ms.AppendMessage("Constructor of RegisterRobot Action Item called", MessageLevel.Warning);
        }

        public override void Execute(PropertyCollection args)
        {
            ms.AppendMessage("Executing RegisterRobot...", MessageLevel.Warning);

            //TODO: Fix the hard index access or at least print out a message if input was wrong
            String robotName = (String)args.GetByIndex(0).Value;
            IRobot robot = app.Value.World.FindComponent(robotName).GetRobot();

            if (robot != null)
            {
                if (RobotController.getInstance().RegisterRobot(robot))
                {
                    ms.AppendMessage("Robot registered at RobotController instance.", MessageLevel.Warning);
                } else
                {
                    ms.AppendMessage("Robot Registration failed!", MessageLevel.Warning);
                }
            }
        }
    }
}
