﻿using System;
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

        private double maximumSpeed = 1000;

        // Dieser Block sollte so irgendwann verschwinden
        Vector startJoints;
        Vector goalJoints;
        bool reset = false;
        bool finished = false;
        private double demandedSpeed = 1000;
        
        
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
                
                changeData((manip.component.FindBehavior(VisualRobotManipulatorCollector.BEHAVIOR_NAME) as INote).Note);

            } catch(Exception e)
            {
                Printer.print(e.StackTrace);
            }
            
            if (useSSM) {
                app.Simulation.SimulationStarted += InitSSM;
                app.Simulation.SimulationStopped += KillSSM;
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

        public String pythonState;
        public List<Vector> resultAngles = new List<Vector>();
        public int pathIndex;
        public bool moving = false;

        public void moveAlongJointAngleList(string pythonState, VectorOfDoubleVector vectorOfDoubleVector)
        {
            IoC.Get<IMessageService>().AppendMessage("Starting Motion...", MessageLevel.Warning);
            this.pythonState = pythonState;
            resultAngles.Clear();
            
            foreach(VectorOfDouble jointAngles in vectorOfDoubleVector){
                resultAngles.Add(new Vector(jointAngles.ToArray()));
            }

            pathIndex = 0;

            startJoints = correctJoints(manip.getConfiguration());
            goalJoints = resultAngles[pathIndex];
            while (goalJoints.Equals(startJoints)) {
                pathIndex++;
                if (pathIndex == resultAngles.Count) {
                    IStringSignal movementFinished = (IStringSignal)manip.component.FindBehavior("MovementFinished");
                    movementFinished.Value = pythonState; // indicate motion done
                    return;
                }
                goalJoints = resultAngles[pathIndex];
            }
            calcDelta();

            finished = false;
            moving = true;

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
                
        public void kill() {
            killed = true;
            _app.Simulation.SimulationReset -= resetSimulation;
            IoC.Get<ISimulationService>().PropertyChanged -= elapsedCallback;
            IoC.Get<IDebugCall>().DebugCall[0] -= printCart;
            IoC.Get<IDebugCall>().DebugCall[2] -= checkDH;
        }

        private void resetSimulation(object sender, EventArgs e)
        {
            moving = false;
            finished = true;
            reset = true;
        }

        private double lastNorm = -1;

        private Vector correctJoints(Vector joints)
        {

            Vector jointsNew = new Vector(joints.Length);
            for (int i = 0; i < joints.Length; i++)
            {
                jointsNew[p.p(i)] = joints[i];
            }
            return jointsNew;
        }

        private Vector makeJointsShittyAgain(Vector joints) {
            Vector jointsNew = new Vector(joints.Length);
            for (int i = 0; i < joints.Length; i++)
            {
                jointsNew[p.i(i)] = joints[i];
            }
            return jointsNew;
        }






        /********************************************/

        private void RobotCycle()
        {
            if (killed) { return; }

            if(controllerWrapper == null) { return;  }

            if (!moving) { return; }

            try
            {
                /* if (reset)
                 {
                     //manip.setConfiguration(startJoints);
                     reset = false;
                     finished = false;
                     //lastDistance = StaticKinetics.cartesianDistance(kinematics, startJoints, goalJoints); 
                     return;
                 }*/

                if (finished)
                {
                    pathIndex++;
                    if (pathIndex >= resultAngles.Count)
                    {
                        // TODO schmeiss die Nachricht raus

                        IStringSignal movementFinished = (IStringSignal)manip.component.FindBehavior("MovementFinished");
                        movementFinished.Value = pythonState; // indicate motion done
                        moving = false;
                        Printer.print("Finished Motion " + pythonState);
                        Printer.print(correctJoints(manip.getConfiguration()).ToString());
                        Printer.print(resultAngles[--pathIndex].ToString());

                        manip.setConfiguration(makeJointsShittyAgain(resultAngles[pathIndex]));

                        return;
                    }

                    //Printer.print("Next Segment (" + pathIndex.ToString() + "/" + (resultAngles.Count - 1).ToString() + ")");
                    startJoints = correctJoints(manip.getConfiguration());
                    goalJoints = resultAngles[pathIndex];
                    calcDelta();
                    finished = false;
                    return;
                }
                Vector joints = correctJoints(manip.getConfiguration());




                if (Double.IsNaN(deltaJoints[0]))
                {
                    Printer.print("GOT NAN!!");
                    return;
                }

                //wait for next cycle 
                if (this.deltaTime == 0)
                {
                    return;
                }

                lastSpeed = Math.Min(demandedSpeed, maximumSpeed);

                /*
                double dist = StaticKinetics.cartesianDistance(kinematics, joints, goalJoints);
                if (dist < lastSpeed * this.deltaTime)
                {
                    finished = true;
                }
                */
                
                /*
                finished = true;
                for (int i = 0; i < joints.Length; i++) {
                    if (Math.Abs(goalJoints[i] - joints[i]) > 0.1) {
                        finished = false; break;
                    }
                }*/

                /*
                Vector jointsNew = new Vector(joints.Length);
                Vector dotJointsNew = new Vector(deltaJoints.Length);
                for (int i = 0; i < joints.Length; i++)
                {
                    jointsNew[p.p(i)] = joints[i];
                    dotJointsNew[p.p(i)] = deltaJoints[i];
                }
                */

                //Vector cartesianSpeed = controllerWrapper.FKSpeed(jointsNew, dotJointsNew);
                Vector cartesianSpeed = controllerWrapper.FKSpeed(joints, deltaJoints);

                //Jacobian appro = Jacobian.calcApproJacobian(kinematics, joints);
                //Vector cartesianSpeedAppro = appro.multiply(deltaJoints);

                Vector cartesianTransSpeed = new Vector(3);
                //Vector cartesianTransSpeedAppro = new Vector(3);

                for (int i = 0; i < 3; i++)
                {
                    cartesianTransSpeed[i] = cartesianSpeed[i] / deltaTime;
                    //cartesianTransSpeedAppro[i] = cartesianSpeedAppro[i] / ticker.tickTime;
                }

                double factor = lastSpeed / cartesianTransSpeed.Norm;

                Vector newJoints = new Vector(joints.Length);

                for (int i = 0; i < deltaJoints.Length; i++)
                {
                    newJoints[i] = joints[i] + deltaJoints[i] * factor;
                }

                double dist = (goalJoints - newJoints).Norm;
                
                if (lastNorm >= 0)
                {
                    if (dist > lastNorm)
                    {
                        finished = true;
                        lastNorm = -1;
                        return;
                    }

                    if (dist < 3) {
                        Printer.print(correctJoints(manip.getConfiguration()).ToString());
                    }
                }

                lastNorm = dist;

                manip.setConfiguration(makeJointsShittyAgain(newJoints));

                

            }
            catch (Exception ee)
            {
                Printer.printTimed(ee.Message +"\n"+ ee.StackTrace);
            }

        }

        public void changeData(String data)
        {
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
