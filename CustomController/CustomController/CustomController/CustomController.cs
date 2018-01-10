using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using VisualComponents.Create3D;
using Caliburn.Micro;

using Debugger;

namespace CustomController
{
    public partial class CustomController
    {
        public VisualRobotManipulator manip;
        
        private IMotionTarget kinematics;
        Vector deltaJoints;
        double lastDistance;
        double lastSpeed = 400;

        private double maximumSpeed = 400;

        // Dieser Block sollte so irgendwann verschwinden
        Vector startJoints;
        Vector goalJoints;
        bool reset = false;
        bool finished = false;
        private double demandedSpeed = 400;
        
        
        private IApplication _app;
        
        private bool killed = false;

        private ISimulationTicker ticker;

        public CustomController(IApplication app, VisualRobotManipulator manip)
        {
            try
            {
                _app = app;
                this.manip = manip;

                kinematics = manip.robot.RobotController.CreateTarget();
                kinematics.UseJointValues = false;
                
                setFromStartFrame();
                setFromGoalFrame();
                manip.robot.Component.FindFeature("startFrame").TransformationChanged += (e, a) => { setFromStartFrame(); };
                manip.robot.Component.FindFeature("goalFrame").TransformationChanged += (e, a) => { setFromGoalFrame(); };
            } catch(Exception e)
            {
                debug(e.StackTrace);
            }

            if (useSSM) {
                initSSM();
            }

            // TODO entfernen
            IoC.Get<IDebugCall>().NumValue[0] = demandedSpeed; //TODO fix
            IoC.Get<IDebugCall>().DebugNum[0] += () => { MaxSpeed = IoC.Get<IDebugCall>().NumValue[0]; };
            IoC.Get<IDebugCall>().DebugCall[0] += () => { IoC.Get<IMessageService>().AppendMessage(demandedSpeed.ToString(), MessageLevel.Warning);  };
            
            ticker = IoC.Get<ISimulationTicker>();
            ticker.timerTick += RobotCycle;
            app.Simulation.SimulationReset += (o,e) => { reset = true; };

            // viel mehr krams


        }
        
        private Matrix WorldToRobot(Matrix from)
        {
            return Matrix.Multiply(kinematics.WorldBaseMatrix.Inverse(), from);
        }
        
        private void calcDelta() {
            deltaJoints = goalJoints - startJoints;
            deltaJoints.Norm = 1;
            reset = true;
        }

        public void setFromStartFrame() {
            kinematics.TargetMatrix = WorldToRobot(manip.robot.Component.RootNode.GetFeatureTransformationInWorld(manip.robot.Component.FindFeature("startFrame")));
            startJoints = new Vector(kinematics.GetAllJointValues());
            if (goalJoints != null)
            {
                calcDelta();
            }
        }

        public void setFromGoalFrame() {
            kinematics.TargetMatrix = WorldToRobot(manip.robot.Component.RootNode.GetFeatureTransformationInWorld(manip.robot.Component.FindFeature("goalFrame")));
            goalJoints = new Vector(kinematics.GetAllJointValues());
            if (startJoints != null)
            {
                calcDelta();
            }
        }
        
        public void kill() {
            killed = true;
        }

        private void RobotCycle()
        {
            if (killed) { return; }
            
            try
            {
                if (reset)
                {
                    manip.setConfiguration(startJoints);
                    reset = false;
                    finished = false;
                    lastDistance = StaticKinetics.cartesianDistance(kinematics, startJoints, goalJoints); 
                    return;
                }
                if (finished)
                {
                    Vector buffer = startJoints;
                    startJoints = goalJoints;
                    goalJoints = buffer;
                    calcDelta();
                    finished = false;
                    return;
                }
                Vector joints = manip.getConfiguration();

                lastSpeed = Math.Min(demandedSpeed, maximumSpeed);

                double dist = StaticKinetics.cartesianDistance(kinematics, joints, goalJoints);
                if (dist < lastSpeed * ticker.tickTime)
                {
                    finished = true;
                }

                Jacobian appro = Jacobian.calcApproJacobian(kinematics, joints);
                Vector cartesianSpeed = appro.multiply(deltaJoints);
                Vector cartesianTransSpeed = new Vector(3);
                for (int i = 0; i < 3; i++)
                {
                    cartesianTransSpeed[i] = cartesianSpeed[i] / ticker.tickTime;
                }
                double factor = lastSpeed / cartesianTransSpeed.Norm;

                // sollte schneller als "joints = joints + deltaJoints * factor" sein, da nur eine Schleife!
                for (int i = 0; i < deltaJoints.Length; i++)
                {
                    joints[i] += deltaJoints[i] * factor;
                }
                
                double speed = StaticKinetics.cartesianDistance(kinematics, manip.getConfiguration(), joints) / ticker.tickTime;
                manip.setConfiguration(joints);

                //if (Math.Abs((commandedSpeed - speed) / commandedSpeed) > 0.05)
                //{
                //    IoC.Get<IMessageService>().AppendMessage("(" + manip.component.Name + ") Speed: " + speed.ToString(), MessageLevel.Warning);
                //}

            } catch(Exception ee)
            {
                debug("CycleTime: "+ ee.StackTrace);
            }

        }

        private void debug(String message)
        {
            IoC.Get<IMessageService>().AppendMessage(_app.Simulation.Elapsed.ToString() + ": " + message, MessageLevel.Warning);
        }

    }
}
