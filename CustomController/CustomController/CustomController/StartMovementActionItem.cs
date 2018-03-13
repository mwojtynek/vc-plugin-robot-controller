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
using System.Threading;


namespace CustomController
{
    [Export(typeof(IActionItem))]
    public class StartMovementActionItem : ActionItem
    {
        [Import]
        private Lazy<IApplication> app = null;

        IMessageService ms = null;
        IRobot robot = null;
        MotionPlan motionPlan = null;


        public StartMovementActionItem() : base("StartMovement")
        {
            ms = IoC.Get<IMessageService>();
            ms.AppendMessage("Constructor of StartMovement Action Item called", MessageLevel.Warning);
        }

        MotionPlanningManager mpm;
        public override void Execute(PropertyCollection args)
        {
                if(args.Count < 6)
                {
                    ms.AppendMessage("Too few arguments were passed to StartMovementActionItem. [robotName, startFrameName, goalFrameName, maxAllowedCartesianSpeed, payload, stapleComponentName]", MessageLevel.Warning);
                    return;
                }
            ms.AppendMessage("Starting MotionPlanner...", MessageLevel.Warning);
                String robotName = (String)args.GetByIndex(0).Value;
                ISimComponent robotParent = app.Value.World.FindComponent(robotName);
                robot = robotParent.GetRobot();

                String startFrameName = (String)args.GetByIndex(1).Value;
                String goalFrameName = (String)args.GetByIndex(2).Value;
                int maxAllowedCartesianSpeed = (int)args.GetByIndex(3).Value;
                String pythonState = (String)args.GetByIndex(4).Value;
                String stapleComponentName = (String)args.GetByIndex(5).Value;


                RobotSection parameter = ConfigReader.readSection(robotName);

                mpm = new MotionPlanningManager();
                motionPlan = mpm.InitializeMotionPlanner(robot,
                                                        parameter.urdfFile.Path,
                                                        RobotParameters.KinStart, RobotParameters.KinEnd,
                                                        parameter.obsFile.Path);
                if(stapleComponentName != null && stapleComponentName != "")
                {
                    stapleConfig(stapleComponentName);
                }
            
            
                VectorOfDoubleVector resultMotion = mpm.planMotion(robot, motionPlan, startFrameName, goalFrameName);
                if (resultMotion != null)
                {
                    IBehavior beh = robot.Component.FindBehavior("MovementFinished");
                    if (beh != null && beh is IStringSignal)
                    {
                        IStringSignal movementFinished = (IStringSignal)robot.Component.FindBehavior("MovementFinished");
                        movementFinished.Value = ""; // empty string means no payload contained yet
                        CustomController sinanController = IoC.Get<ICustomController>().getController(robotName);
                        sinanController.moveAlongJointAngleList(pythonState, motionPlan.getLastResult());
                    } else {
                        ms.AppendMessage("\"MovementFinished\" behavior was either null or not of type IStringSignal. Abort!", MessageLevel.Warning);
                    }
                }

        }

        private void stapleConfig(String stapleComponentName)
        {
            StapleSection parameterStaple = ConfigReader.readStapleConfig();

            ISimComponent stapleComponent = app.Value.World.FindComponent(stapleComponentName);
            if (stapleComponentName != "" && stapleComponent == null){
                ms.AppendMessage("Failed to find staple component with name\"" + stapleComponentName + "\"! Planning of motion aborted...", MessageLevel.Warning);
                return;
            }

            String stapleFileFolder = parameterStaple.staplePath.Path;
            if (stapleComponent.GetProperty("stlID") == null){
                ms.AppendMessage("Component with name \"" + stapleComponentName + "\" has no property \"stlID\"! Planning of motion aborted...", MessageLevel.Warning);
                return;
            }
            String stlID = (String)stapleComponent.GetProperty("stlID").Value;
            String obstacleFilePath = stapleFileFolder + stlID + ".stl";
            if (!File.Exists(obstacleFilePath)){
                ms.AppendMessage("A path planning request requested stlID with \"" + stlID + "\" failed, because file \"" + obstacleFilePath + "\" does not exist...", MessageLevel.Warning);
                return;
            };
            if (stapleComponent.GetProperty("StackHeight") == null){
                ms.AppendMessage("Failed to find StackHeight property in component with name \"" + stapleComponentName + "\"! Planning of motion aborted!", MessageLevel.Warning);
                return;
            }

            Vector3 staplePosition = stapleComponent.TransformationInWorld.GetP();
            IDoubleProperty stackwidth = (IDoubleProperty)stapleComponent.GetProperty("StackWidth");
            IDoubleProperty stacklength = (IDoubleProperty)stapleComponent.GetProperty("StackLength");

            float translate_x = (float)staplePosition.X - (float)(stacklength.Value / 2.0);
            float translate_y = (float)staplePosition.Y - (float)(stackwidth.Value / 2.0);

            IDoubleProperty stackheight = (IDoubleProperty)stapleComponent.GetProperty("StackHeight");
            float translate_z = (float)stackheight.Value;

            // setup staple obstacle
            int obstacleId = motionPlan.addObstacle(obstacleFilePath, translate_x / 1000.0f, translate_y / 1000.0f, translate_z / 1000.0f, 0.0f, 0.0f, 0.0f);
        }

    }
}
