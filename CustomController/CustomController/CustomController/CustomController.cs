using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using VisualComponents.Create3D;
using Caliburn.Micro;

using RosiTools.Debugger;
using RosiTools.Printer;

using System.Xml;
using System.ComponentModel;

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

        private newCustomController controllerWrapper;
        private Permutator p;

        private double lastTime;
        private double deltaTime;

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
                changeData((manip.component.FindBehavior(VisualRobotManipulatorCollector.BEHAVIOR_NAME) as INote).Note);

            } catch(Exception e)
            {
                Printer.print(e.StackTrace);
            }

            if (useSSM) {
                initSSM();
            }

            IoC.Get<IDebugCall>().DebugCall[0] += printCart;
            IoC.Get<IDebugCall>().DebugCall[1] += () => {
                demandedSpeed = IoC.Get<IDebugCall>().NumValue[0];
            };
            IoC.Get<IDebugCall>().DebugCall[2] += checkDH;
            

            app.Simulation.SimulationReset += resetSimulation;
            IoC.Get<ISimulationService>().PropertyChanged += elapsedCallback;

        }

        private void elapsedCallback(object sender, PropertyChangedEventArgs e)
        {
            if (e.PropertyName == "Elapsed") {
                this.deltaTime = _app.Simulation.Elapsed - this.lastTime;
                if (this.deltaTime < 0) {
                    this.deltaTime = 0;
                }
                this.lastTime = _app.Simulation.Elapsed;
                RobotCycle();
            }
        }

        private void printCart()
        {
            /*
            Printer.print(manip.component.Name);
            kinematics.SetAllJointValues(manip.getConfigurationDouble());
            Matrix m = WorldToRobot(kinematics.TargetMatrix);
            StringBuilder b = new StringBuilder();
            b.AppendFormat("{0:##0.##}\t{1:##0.##}\t{2:##0.##}\t{3:####0.##}\n", m.Nx, m.Ox, m.Ax, m.Px);
            b.AppendFormat("{0:##0.##}\t{1:##0.##}\t{2:##0.##}\t{3:####0.##}\n", m.Ny, m.Oy, m.Ay, m.Py);
            b.AppendFormat("{0:##0.##}\t{1:##0.##}\t{2:##0.##}\t{3:####0.##}\n", m.Nz, m.Oz, m.Az, m.Pz);
            Printer.print(b.ToString());*/
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
            _app.Simulation.SimulationReset -= resetSimulation;
            IoC.Get<ISimulationService>().PropertyChanged -= elapsedCallback;
            IoC.Get<IDebugCall>().DebugCall[0] -= printCart;
            IoC.Get<IDebugCall>().DebugCall[2] -= checkDH;
        }

        private void resetSimulation(object sender, EventArgs e)
        {
            reset = true;
        }

        private void RobotCycle()
        {
            if (killed) { return; }

            if(controllerWrapper == null) { return;  }
            
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

                //wait for next cycle 
                if (this.deltaTime == 0) {
                    return;
                }

                lastSpeed = Math.Min(demandedSpeed, maximumSpeed);

                double dist = StaticKinetics.cartesianDistance(kinematics, joints, goalJoints);
                if (dist < lastSpeed * this.deltaTime)
                {
                    finished = true;
                }
                
                Vector jointsNew = new Vector(joints.Length);
                Vector dotJointsNew = new Vector(deltaJoints.Length);
                for (int i = 0; i < joints.Length; i++) {
                    jointsNew[p.p(i)] = joints[i];
                    dotJointsNew[p.p(i)] = deltaJoints[i];
                }
                Vector cartesianSpeed = controllerWrapper.FKSpeed(jointsNew, dotJointsNew);
               
                //Jacobian appro = Jacobian.calcApproJacobian(kinematics, joints);
                //Vector cartesianSpeedAppro = appro.multiply(deltaJoints);

                Vector cartesianTransSpeed = new Vector(3);
                //Vector cartesianTransSpeedAppro = new Vector(3);

                for (int i = 0; i < 3; i++)
                {
                    cartesianTransSpeed[i] = cartesianSpeed[i] / this.deltaTime;
                    //cartesianTransSpeedAppro[i] = cartesianSpeedAppro[i] / ticker.tickTime;
                }

                double factor = lastSpeed / cartesianTransSpeed.Norm;
                                
                for (int i = 0; i < deltaJoints.Length; i++)
                {
                    joints[i] += deltaJoints[i] * factor;
                }
                
                double speed = StaticKinetics.cartesianDistance(kinematics, manip.getConfiguration(), joints) / this.deltaTime;
                manip.setConfiguration(joints);

                //if (Math.Abs((commandedSpeed - speed) / commandedSpeed) > 0.05)
                //{
                //    IoC.Get<IMessageService>().AppendMessage("(" + manip.component.Name + ") Speed: " + speed.ToString(), MessageLevel.Warning);
                //}

            } catch(Exception ee)
            {
                Printer.printTimed(ee.Message +"\n"+ ee.StackTrace);
            }

        }

        public void changeData(String data)
        {

            /*
            String[] rows = data.Split('\n');
            List<double> dh = new List<double>();
            foreach (String i in rows)
            {
                if ( String.IsNullOrWhiteSpace(i)) { break; }
                String[] vals = i.Split(',');
                if (vals.Length != 4) {
                    Printer.print("Malformed Note!");
                    return;
                }
                foreach (String val in vals) {
                    dh.Add(Double.Parse(val));
                }
            }
            if (dh.Count % 4 != 0) {
                Printer.print("Something went wrong while parsing note");
                return;
            }
            if (dh.Count / 4 != manip.jointCount) {
                Printer.print("Note does not describe enough joints!");
                return;
            }
            double[] dhArray = dh.ToArray();
            controllerWrapper = new newCustomController(dhArray);
            Printer.print(manip.component.Name + " got a KDL Interface!");
            */

            XmlDocument doc = new XmlDocument();
            try
            {
                doc.LoadXml(data);
            }
            catch (XmlException e)
            {
                Printer.print(manip.component.Name + " has not a parseable Xml configuration!");
                return;
            }

            p = null;
            List<double> dh = new List<double>();

            foreach (XmlNode node in doc.FirstChild.ChildNodes)
            {
                if (node.Name == "joints")
                {
                    XmlNodeList jointList = node.ChildNodes;
                    if (jointList.Count != manip.jointCount)
                    {
                        Printer.print(manip.component.Name + " has not a valid Xml configuration (joint count is not correct)");
                        return;
                    }
                    string[] param = { "a", "alpha", "d", "theta" };
                    foreach (XmlNode jointNode in node.ChildNodes)
                    {
                        if (jointNode.Name != "joint")
                        {
                            Printer.print("Check the joints section in the Xml configuration of " + manip.component.Name);
                            return;
                        }
                        for (int i = 0; i < 4; i++)
                        {
                            XmlNode paramInstance = jointNode.Attributes.GetNamedItem(param[i]);
                            if (paramInstance == null)
                            {
                                dh.Add(0.0);
                            }
                            else
                            {
                                dh.Add(Double.Parse(paramInstance.Value));
                            }
                        }
                    }
                    double[] dhArray = dh.ToArray();
                    controllerWrapper = new newCustomController(dhArray);
                }
                if (node.Name == "permutation")
                {
                    List<int> perm = new List<int>();
                    foreach (String sub in node.InnerText.Split(' '))
                    {
                        perm.Add(int.Parse(sub));
                    }
                    p = new Permutator(perm.ToArray());
                }
            }

            if (controllerWrapper == null)
            {
                Printer.print(manip.component.Name + " has not a valid Xml configuration (joints is missing!)");
                return;
            }

            if (p == null)
            {
                p = new Permutator(manip.jointCount);
            }
        }

        public void checkDH() { 
            if(controllerWrapper == null) { return;  }
            int joint = (int)IoC.Get<IDebugCall>().NumValue[0];
            double[] joints = new double[manip.jointCount];
            if (joint >= 0 && joint < manip.jointCount)
            {
                for (int i = 0; i < manip.jointCount; i++) {
                    joints[i] = 0.0;
                }

                joints[joint] = 90;
            } else if ( joint < 0){
                for (int i = 0; i < manip.jointCount; i++)
                {
                    joints[i] = 0;
                }
            }
            else
            {
                Random rand = new Random();
                for (int i = 0; i < manip.jointCount; i++)
                {
                    joints[i] = 180.0 * rand.NextDouble() - 90.0;
                }
            }

            StringBuilder bld = new StringBuilder();

                bld.AppendFormat("{0} jnt:", manip.component.Name);
                for (int i = 0; i < manip.jointCount; i++) {
                    bld.AppendFormat(" {0}", joints[i]);
                }
                bld.AppendLine("");

                bld.AppendFormat("{0} KDL:", manip.component.Name);
                double[] kdlJoints = new double[joints.Length];
            for (int i = 0; i < joints.Length; i++) {
                kdlJoints[p.p(i)] = joints[i]; 
            }
                double[] kdlFK = controllerWrapper.FK(new Vector(kdlJoints)).Elements;
                for (int i = 0; i < 6; i++)
                {
                    bld.AppendFormat(" {0:####0.##}", kdlFK[i]);
                }
                bld.AppendLine("");

                bld.AppendFormat("{0} VC :", manip.component.Name);
                double[] vcFK = StaticKinetics.FK(kinematics, new Vector(joints)).Elements;
                for (int i = 0; i < 6; i++)
                {
                    bld.AppendFormat(" {0:####0.##}", vcFK[i]);
                }

                Printer.print(bld.ToString());
            
        }
        
    }
}
