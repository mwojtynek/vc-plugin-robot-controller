using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using VisualComponents.Create3D;
using Caliburn.Micro;

using RosiTools.Debugger;
using RosiTools.SimulationTime;
using RosiTools.Printer;

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

            IoC.Get<IDebugCall>().DebugCall[0] += printCart;
            IoC.Get<IDebugCall>().DebugCall[1] += () => { demandedSpeed = IoC.Get<IDebugCall>().NumValue[0]; };

            ticker = IoC.Get<ISimulationTicker>();
            ticker.timerTick += RobotCycle;
            app.Simulation.SimulationReset += resetSimulation;
            

            // viel mehr krams
            /*
            if (manip.component.Name == "TX60")
            {
                ISimNode comp = null;
                foreach (ISimNode sim in manip.component.Nodes) {
                    if (sim.IsComponentRoot) { comp = sim; }
                }
                recursivePrint(comp, 0);

            }
            */

        }

        private void printCart()
        {
            Printer.print(manip.component.Name);
            kinematics.SetAllJointValues(manip.getConfigurationDouble());
            Matrix m = WorldToRobot(kinematics.TargetMatrix);
            StringBuilder b = new StringBuilder();
            b.AppendFormat("{0:##0.##}\t{1:##0.##}\t{2:##0.##}\t{3:####0.##}\n", m.Nx, m.Ox, m.Ax, m.Px);
            b.AppendFormat("{0:##0.##}\t{1:##0.##}\t{2:##0.##}\t{3:####0.##}\n", m.Ny, m.Oy, m.Ay, m.Py);
            b.AppendFormat("{0:##0.##}\t{1:##0.##}\t{2:##0.##}\t{3:####0.##}\n", m.Nz, m.Oz, m.Az, m.Pz);
            Printer.print(b.ToString());
        }

        private void recursivePrint(ISimNode cur, int level) {
            string dash = new string('\t', level);
            if (cur.DofType != JointType.Custom)
            {
                Printer.print(dash + cur.Name);
            }
            else
            {
                IExpressionProperty off = cur.GetProperty("Offset") as IExpressionProperty;
                IExpressionProperty joint = cur.Dof.GetProperty("Joint") as IExpressionProperty;
                if (joint != null && off != null)
                {
                    Printer.print(dash + cur.Name + " " + joint.Expression+ " " + off.Expression);

                } else
                {
                    foreach (string str in cur.Dof.PropertyNames)
                    {
                        Printer.print(dash + cur.Name + " " + str + " " + cur.Dof.GetProperty(str).GetType().ToString());
                    }

                }
            }

            if (cur.Children.Count != 0)
            {
                foreach (ISimNode child in cur.Children)
                {
                    recursivePrint(child, level + 1);
                }
            }
        }

        ~CustomController() {
            kill();
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
            ticker.timerTick -= RobotCycle;
            _app.Simulation.SimulationReset -= resetSimulation;
            IoC.Get<IDebugCall>().DebugCall[0] -= printCart;
        }

        private void resetSimulation(object sender, EventArgs e)
        {
            reset = true;
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
