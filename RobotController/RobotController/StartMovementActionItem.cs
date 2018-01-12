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

		Dictionary<String, MotionPlan> motionPlanCollection = new Dictionary<string, MotionPlan>();
		public override void Execute(PropertyCollection args)
		{
			//ms.AppendMessage("Executing StartMovement with counter value" + counter, MessageLevel.Warning);

			//TODO: Fix the hard index access or at least print out a message if input was wrong
			String robotName = (String)args.GetByIndex(0).Value;
			robot = app.Value.World.FindComponent(robotName).GetRobot();

			String startFrameName, goalFrameName;
			if (counter == 5)
			{
				startFrameName = counter.ToString();
				goalFrameName = "1";
				counter = 1;
			}
			else
			{
				startFrameName = counter.ToString();
				goalFrameName = (counter + 1).ToString();
				counter++;
			}

			String accessKey = startFrameName + ":" + goalFrameName;
			if (!motionPlanCollection.TryGetValue(accessKey, out motionPlan))
			{
				mpm = new MotionPlanningManager();

				motionPlan = mpm.InitializeMotionPlanner(robot,
																								RobotParameters.UrdfFile,
																								RobotParameters.KinStart, RobotParameters.KinEnd,
																								RobotParameters.obstacleModelFile);

				VectorOfDoubleVector resultMotion = mpm.planMotion(robot, motionPlan, startFrameName, goalFrameName);
				motionPlanCollection.Add(accessKey, motionPlan);
			}


			IBooleanSignal movementFinished = (IBooleanSignal)robot.Component.FindBehavior("MovementFinished");
			movementFinished.Value = false;

			RobotController.getInstance().AddMotionPlan(robot, motionPlan);

			//ms.AppendMessage("Executed StartMovement.", MessageLevel.Warning);
		}

	}
}
