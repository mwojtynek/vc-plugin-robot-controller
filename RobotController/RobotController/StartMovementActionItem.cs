using Caliburn.Micro;
using System;
using System.ComponentModel.Composition;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VisualComponents.Create3D;
using VisualComponents.UX.Shared;
using System.IO;

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
            if(args.Count < 6)
            {
                ms.AppendMessage("Too few arguments were passed to StartMovementActionItem. [robotName, startFrameName, goalFrameName, maxAllowedCartesianSpeed, payload, stapleComponentName]", MessageLevel.Warning);
            }
            //TODO: Fix the hard index access or at least print out a message if input was wrong
            String robotName = (String)args.GetByIndex(0).Value;
            ISimComponent robotParent = app.Value.World.FindComponent(robotName);
            robot = robotParent.GetRobot();

            String startFrameName = (String)args.GetByIndex(1).Value;
            String goalFrameName = (String)args.GetByIndex(2).Value;
            int maxAllowedCartesianSpeed = (int)args.GetByIndex(3).Value;
            String payload = (String)args.GetByIndex(4).Value;
            //String stapleComponentName = "StapleForRightIiwa"
            String stapleComponentName = (String)args.GetByIndex(5).Value;
            

            RobotSection parameter = ConfigReader.readSection(robotName);
            StapleSection parameterStaple = ConfigReader.readStapleConfig();
            
            ISimComponent stapleComponent = app.Value.World.FindComponent(stapleComponentName);
            if(stapleComponentName != "" && stapleComponent == null)
            {
                ms.AppendMessage("Failed to find staple component with name\""+stapleComponentName+"\"! Planning of motion aborted...", MessageLevel.Warning);
                return;
            }
            
            String stapleFileFolder = parameterStaple.staplePath.Path;
            if(stapleComponent.GetProperty("stlID") == null)
            {
                ms.AppendMessage("Component with name \""+stapleComponentName+"\" has no property \"stlID\"! Planning of motion aborted...", MessageLevel.Warning);
                return;
            }
            String stlID = (String) stapleComponent.GetProperty("stlID").Value;
            String obstacleFilePath = stapleFileFolder + stlID + ".stl";
            if (!File.Exists(obstacleFilePath))
            {
                ms.AppendMessage("A path planning request requested stlID with \"" + stlID + "\" failed, because file \""+obstacleFilePath+"\" does not exist...", MessageLevel.Warning);
                return;
            };
            if (stapleComponent.GetProperty("StackHeight") == null) {
                ms.AppendMessage("Failed to find StackHeight property in component with name \""+stapleComponentName+"\"! Planning of motion aborted!", MessageLevel.Warning);
                return;
            }


            RobotController.getInstance().setMaxAllowedCartesianSpeed(robot, maxAllowedCartesianSpeed);
            motionPlanCollection.Clear();
            if (!motionPlanCollection.TryGetValue(robotParent, out motionPlan))
            {
                mpm = new MotionPlanningManager();
                motionPlan = mpm.InitializeMotionPlanner(robot,
                                                        parameter.urdfFile.Path,
                                                        RobotParameters.KinStart, RobotParameters.KinEnd,
                                                        parameter.obsFile.Path);
                motionPlanCollection.Add(robotParent, motionPlan);
                ms.AppendMessage("Created new motionPlan for " + robotName, MessageLevel.Warning);
            }

            Vector3 staplePosition = stapleComponent.TransformationInWorld.GetP();
            IDoubleProperty stackwidth = (IDoubleProperty)stapleComponent.GetProperty("StackWidth");
            IDoubleProperty stacklength = (IDoubleProperty)stapleComponent.GetProperty("StackLength");

            float translate_x = (float) staplePosition.X - (float) (stacklength.Value / 2.0);
            float translate_y = (float) staplePosition.Y - (float)(stackwidth.Value / 2.0);

            IDoubleProperty stackheight = (IDoubleProperty) stapleComponent.GetProperty("StackHeight");
            float translate_z = (float) stackheight.Value;

            // setup staple obstacle
            int obstacleId = motionPlan.addObstacle(obstacleFilePath, translate_x/1000.0f, translate_y / 1000.0f, translate_z / 1000.0f, 0.0f, 0.0f, 0.0f);
            //motionPlan.showSetupInInspector();

            VectorOfDoubleVector resultMotion = mpm.planMotion(robot, motionPlan, startFrameName, goalFrameName);
            if (resultMotion != null)
            {
                IBehavior beh = robot.Component.FindBehavior("MovementFinished");
                if (beh != null && beh is IStringSignal)
                {
                    IStringSignal movementFinished = (IStringSignal)robot.Component.FindBehavior("MovementFinished");
                    movementFinished.Value = ""; // empty string means no payload contained yet

                    MotionInterpolator inp = motionPlan.getMotionInterpolator();
                    inp.setMaxJointAcceleration(7.5);
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
