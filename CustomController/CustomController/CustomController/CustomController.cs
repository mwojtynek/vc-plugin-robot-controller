using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using VisualComponents.Create3D;
using Caliburn.Micro;

using RosiTools.Debugger;
using RosiTools.Printer;

using RosiTools.Collector;

using System.Xml;
using System.ComponentModel;


namespace CustomController
{
    [CollectorType("CustomController")]
    public partial class CustomController : CollectorClass
    {
        public VisualRobotManipulator manip;
        
        Vector deltaJoints;
        double lastSpeed = 400;

        private double maximumSpeed = 400;

        Vector startJoints;
        Vector goalJoints;

        bool finished = false;
        private double demandedSpeed = 1000;
        
        private IApplication _app;
        
        private bool killed = false;

        private newCustomController controllerWrapper;
        private Permutator p;

        private double lastTime;
        private double deltaTime;


        private double lastNorm = -1;


        public String pythonState;
        public List<Vector> resultAngles = new List<Vector>();
        public int pathIndex;
        public bool moving = false;
        
        public CustomController(ISimComponent component, IApplication app) : base(component, app)
        {
            try
            {
                _app = app;
                this.manip = new VisualRobotManipulator(app, component.Name);

            } catch(Exception e)
            {
                Printer.print(e.StackTrace);
            }
            /*
            if (useSSM) {
                initSSM();
            }*/
            
            IoC.Get<IDebugCall>().DebugCall[1] += () => {
                demandedSpeed = IoC.Get<IDebugCall>().NumValue[0];
            };
            

            app.Simulation.SimulationReset += resetSimulation;
            IoC.Get<ISimulationService>().PropertyChanged += elapsedCallback;
            Printer.print("Robot added");
        }

        private void elapsedCallback(object sender, PropertyChangedEventArgs e)
        {
            if (e.PropertyName == "Elapsed") {
                this.deltaTime = _app.Simulation.Elapsed - this.lastTime;
                this.lastTime = _app.Simulation.Elapsed;
                if (this.deltaTime > 0)
                {
                    RobotCycle();
                }
            }
        }
        
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
            startMotion();
        }
        
        private void startMotion() {
            deltaJoints = goalJoints - startJoints;
            deltaJoints.Norm = 1;

            moving = true;
            finished = false;

            lastNorm = -1;
        }
                
        public void kill() {
            killed = true;
            _app.Simulation.SimulationReset -= resetSimulation;
            IoC.Get<ISimulationService>().PropertyChanged -= elapsedCallback;
        }

        private void resetSimulation(object sender, EventArgs e)
        {
            moving = false;
            finished = true;
        }

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
            if (
                killed ||
                controllerWrapper == null ||
                !moving)
            {
                return;
            }
            
            try
            {

                // finish check
                if (finished)
                {

                    // next segment available check
                    pathIndex++;
                    if (pathIndex >= resultAngles.Count)
                    {
                        // TODO schmeiss die Nachricht raus

                        IStringSignal movementFinished = (IStringSignal)manip.component.FindBehavior("MovementFinished");
                        movementFinished.Value = pythonState; // indicate motion done
                        moving = false;
                        Printer.print("Finished Motion " + pythonState +"\n" + correctJoints(manip.getConfiguration()).ToString() +"\n" + resultAngles[--pathIndex].ToString());

                        //HOTFIX
                        manip.setConfiguration(makeJointsShittyAgain(resultAngles[pathIndex]));

                        return;
                    }
                    
                    startJoints = correctJoints(manip.getConfiguration());
                    goalJoints = resultAngles[pathIndex];
                    startMotion();
                    return;
                }

                // NaN check entfernt: darf eigentlich nicht passieren, nachdem startMotion aufgerufen wurde (erst dann wird moving true)
                // Cycle Check entfern: sollte von außerhalb geschehen. Ein Cycle-Aufruf ohne Cycle ist herrlich behindert...

                // aktuelle Joints beziehen (in korrekter Reihenfolge)
                Vector joints = correctJoints(manip.getConfiguration());
                
                // MaxSpeed bestimmen
                // TODO pure-PTP und SSM-limited PTP unterscheiden!
                lastSpeed = Math.Min(demandedSpeed, maximumSpeed);

                // TODO FKSpeed untersuchen!
                // (v, omega) = J(q) * q_dot  | wird hier berechnet
                Vector cartesianSpeed = controllerWrapper.FKSpeed(joints, deltaJoints);

                Vector cartesianTransSpeed = new Vector(3);
                
                // v aus (v,omega) wird extrahiert und korrigiert(?) TODO richtig an KDL Schnittstelle anpassen
                for (int i = 0; i < 3; i++)
                {
                    cartesianTransSpeed[i] = cartesianSpeed[i] / deltaTime;
                }

                // Geschwindigkeitsfaktor anpassen
                double factor = lastSpeed / cartesianTransSpeed.Norm;


                // neue Joints berechnen mit den an die Geschwindigkeit angepassten deltaJoints
                // q_n+1 = q_n + factor * q_dot
                Vector newJoints = new Vector(joints.Length);
                for (int i = 0; i < deltaJoints.Length; i++)
                {
                    newJoints[i] = joints[i] + deltaJoints[i] * factor;
                }
                
                // Distanz zum Ziel berechnen (in Joint-Space)
                double dist = (goalJoints - newJoints).Norm;
                
                // check ob es eine letzte Distanz gibt (mindestens 2. Aufruf)
                if (lastNorm >= 0)
                {
                    // check ob Ziel überschritten wird.
                    if (dist > lastNorm)
                    {
                        // Bewegung beenden und statt zu überschwingen fix auf das Ziel setzen (haben eh unstetige Geschwindigkeiten)
                        finished = true;
                        newJoints = goalJoints;
                    }
                    
                }
                lastNorm = dist;
                
                // Joints in Visual Components Reihenfolge setzen
                manip.setConfiguration(makeJointsShittyAgain(newJoints));
                
            }
            catch (Exception ee)
            {
                Printer.printTimed(ee.Message +"\n"+ ee.StackTrace);
            }

        }


        ~CustomController()
        {
            kill();
        }

        /*********************** Altlasten ***************************/

        public override void changeNote(String data)
        {
            Printer.print("Note change");
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
        /*
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
            
        }*/



    }
}
