﻿using System;
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
        
        
        Vector startJoints;
        Vector goalJoints;
        Vector deltaJoints;
        double lastSpeed = 400;

        private double maximumSpeed = 400;
        private double demandedSpeed = 1000;

        Vector tcpSpeed = new Vector(3);

        private double lastJointError = -1;

        bool finished = false;
                
        //private newCustomController controllerWrapper;
        private Permutator p;

        private double lastTime;
        private double deltaTime;
        
        public String pythonState;
        public List<Vector> resultAngles = new List<Vector>();
        public MotionPlan mp;
        public int pathIndex;
        public bool moving = false;

        Vector joints;

        public CustomController(ISimComponent component, IApplication app) : base(component, app)
        {
            try
            {
                this.manip = new VisualRobotManipulator(component);

            } catch(Exception e)
            {
                Printer.print(e.StackTrace);
            }
            
            if (useSSM) {
                app.Simulation.SimulationStarted += InitSSM;
                app.Simulation.SimulationStopped += KillSSM;
            }
            
            app.Simulation.SimulationReset += ResetSimulation;
            app.Simulation.SimulationStarted += (o, e) => { joints = CorrectJoints(manip.getConfiguration()); };
            IoC.Get<ISimulationService>().PropertyChanged += ElapsedCallback;
        }

        private void ElapsedCallback(object sender, PropertyChangedEventArgs e)
        {
            if (e.PropertyName == "Elapsed") {
                this.deltaTime = app.Simulation.Elapsed - this.lastTime;
                this.lastTime = app.Simulation.Elapsed;
                if (this.deltaTime > 0)
                {
                    UpdateVisualization();
                    RobotCycle();
                }
            }
        }
        
        public void MoveAlongJointAngleList(string pythonState, MotionPlan motionPlan)
        {
            Printer.printTimed(component.Name + " starts Motion to " + pythonState);
            this.pythonState = pythonState;
            
            mp = motionPlan;
            resultAngles.Clear();
            
            foreach(VectorOfDouble jointAngles in motionPlan.getLastResult()){
                resultAngles.Add(new Vector(jointAngles.ToArray()));
            }


            pathIndex = 0;

            startJoints = joints;//correctJoints(manip.getConfiguration());
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
            StartMotion();
        }
        
        private void StartMotion() {
            deltaJoints = goalJoints - startJoints;
            lastJointError = deltaJoints.Norm;

            if (Math.Abs(deltaJoints.Norm) <= 0.00001)
            {
                // if the start and goal position are identical, we are finished
                finished = true;
            } else {
                finished = false;
                deltaJoints.Norm = 1;
            }

            moving = true;
            // if deltaJoints.Norm ==== 0.0

            //lastJointError = -1;
        }
                
        public new void kill() {
            base.kill();
            app.Simulation.SimulationReset -= ResetSimulation;
            IoC.Get<ISimulationService>().PropertyChanged -= ElapsedCallback;
        }

        private void ResetSimulation(object sender, EventArgs e)
        {
            moving = false;
            finished = true;
        }

        private Vector CorrectJoints(Vector joints)
        {

            Vector jointsNew = new Vector(joints.Length);
            for (int i = 0; i < joints.Length; i++)
            {
                jointsNew[p.p(i)] = joints[i];
            }
            return jointsNew;
        }

        private Vector MakeJointsShittyAgain(Vector joints) {
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
                !isAlive ||
                //controllerWrapper == null ||
                !moving)
            {
                tcpSpeed = new Vector(3);
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
                        Printer.print(component.Name + " finished Motion: " + pythonState);

                        manip.setConfiguration(MakeJointsShittyAgain(resultAngles[pathIndex - 1]));

                        return;
                    }
                    
                    startJoints = CorrectJoints(manip.getConfiguration());
                    goalJoints = resultAngles[pathIndex];
                    StartMotion();
                    return;
                }

                // NaN check entfernt: darf eigentlich nicht passieren, nachdem startMotion aufgerufen wurde (erst dann wird moving true)
                // Cycle Check entfern: sollte von außerhalb geschehen. Ein Cycle-Aufruf ohne Cycle ist herrlich behindert...

                // aktuelle Joints beziehen (in korrekter Reihenfolge)
                joints = CorrectJoints(manip.getConfiguration());

                // MaxSpeed bestimmen
                // TODO pure-PTP und SSM-limited PTP unterscheiden!
                lastSpeed = Math.Min(demandedSpeed, maximumSpeed);

                // TODO FKSpeed untersuchen!
                // (v, omega) = J(q) * q_dot  | wird hier berechnet
                //Vector cartesianSpeed2 = controllerWrapper.FKSpeed(joints, deltaJoints);

                MotionPlanRobotDescription.MotionPlanForwardKinematicResult value = mp.getMotionPlanRobotDescription().getVelFK(joints.toWeirdVector(), deltaJoints.toWeirdVector());
                if (!value.success) return;
                Vector cartesianTransSpeed = new Vector(3);

                // v aus (v,omega) wird extrahiert und korrigiert(?) TODO richtig an KDL Schnittstelle anpassen

                /*
                for (int i = 0; i < 3; i++)
                {
                    cartesianTransSpeed2[i] = cartesianSpeed2[i] / deltaTime;
                }*/


                cartesianTransSpeed[0] = value.x * 1000 / deltaTime;
                cartesianTransSpeed[1] = value.y * 1000 / deltaTime;
                cartesianTransSpeed[2] = value.z * 1000 / deltaTime;

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
                double currentJointError = (goalJoints - newJoints).Norm;
                
                // check ob es eine letzte Distanz gibt (mindestens 2. Aufruf)
                if (lastJointError >= 0)
                {
                    // check ob Ziel überschritten wird.
                    if (currentJointError > lastJointError)
                    {
                        // Bewegung beenden und statt zu überschwingen fix auf das Ziel setzen (haben eh unstetige Geschwindigkeiten)
                        finished = true;
                        newJoints = goalJoints;
                    }
                    
                }
                lastJointError = currentJointError;

                tcpSpeed = new Vector(3);
                MotionPlanRobotDescription.MotionPlanForwardKinematicResult before = mp.getMotionPlanRobotDescription().getFK(joints.toWeirdVector(), true);
                MotionPlanRobotDescription.MotionPlanForwardKinematicResult after = mp.getMotionPlanRobotDescription().getFK(newJoints.toWeirdVector(), true);
                if (!before.success || !after.success) return;

                tcpSpeed[0] = (after.x - before.x) * 1000 / deltaTime;
                tcpSpeed[1] = (after.y - before.y) * 1000 / deltaTime;
                tcpSpeed[2] = (after.z - before.z) * 1000 / deltaTime;

                //Printer.printTimed("Speed: " + tcpSpeed.Norm.ToString() +" (Soll: " + lastSpeed.ToString() + ")");

                // Joints in Visual Components Reihenfolge setzen
                manip.setConfiguration(MakeJointsShittyAgain(newJoints));
                
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
                    //controllerWrapper = new newCustomController(dhArray);
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
            
            /*if (controllerWrapper == null)
            {
                Printer.print(manip.component.Name + " has not a valid Xml configuration (joints is missing!)");
                return;
            }*/

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
