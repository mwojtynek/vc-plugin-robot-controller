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
        private VisualRobotManipulator manip;

        private Vector startJoints;
        private Vector goalJoints;
        private Vector jointsDirection;
        private double appliedSpeed = 400;

        private double allowedSpeed = 400;
        private double demandedSpeed = 1000;

        private Vector tcpSpeed = new Vector(3);

        private Vector lastErrorForEachJoint;

        private bool finished = false;
        
        private Permutator p;

        private double lastTime;
        private double deltaTime;

        private String pythonState;
        private List<Vector> resultAngles = new List<Vector>();
        private MotionPlan mp;
        private int pathIndex;
        private bool moving = false;

        private Vector joints;

        public double DemandedSpeed { get => demandedSpeed;
                                      set => demandedSpeed=value; }

        public CustomController(ISimComponent component, IApplication app) : base(component, app)
        {
            try
            {
                this.manip = new VisualRobotManipulator(component);
            }
            catch (Exception e)
            {
                Printer.print(e.StackTrace);
            }

            if (useSSM)
            {
                app.Simulation.SimulationStarted += InitSSM;
                app.Simulation.SimulationStopped += KillSSM;
            }

            CreateStatisticsComponent();

            app.World.ObjectInvalidated += ObjectInvalidatedHook;
            app.Simulation.SimulationReset += ResetSimulation;
            app.Simulation.SimulationStarted += SimulationStarted;
            IoC.Get<ISimulationService>().PropertyChanged += ElapsedCallback;
        }

        // Destructor
        ~CustomController()
        {
            kill();
        }

        public void ObjectInvalidatedHook(object sender, EventArgs args)
        {
            String wah = "";
        }

        public void SimulationStarted(object sender, EventArgs e)
        {
            joints = ArrangeJointsToControllerOrder(manip.getConfiguration());
        }

        private void ElapsedCallback(object sender, PropertyChangedEventArgs e)
        {
            if (e.PropertyName == "Elapsed")
            {
                this.deltaTime = app.Simulation.Elapsed - this.lastTime;
                this.lastTime = app.Simulation.Elapsed;
                if (this.deltaTime > 0)
                {
                    UpdateVisualization();
                    UpdateStatisticsComponent();
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

            foreach (VectorOfDouble jointAngles in motionPlan.getLastResult())
            {
                resultAngles.Add(new Vector(jointAngles.ToArray()));
            }

            pathIndex = 0;

            startJoints = joints;
            goalJoints = resultAngles[pathIndex];
            while (goalJoints.Equals(startJoints))
            {
                pathIndex++;
                if (pathIndex == resultAngles.Count)
                {
                    IStringSignal movementFinished = (IStringSignal)manip.component.FindBehavior("MovementFinished");
                    movementFinished.Value = pythonState; // indicate motion done
                    return;
                }
                goalJoints = resultAngles[pathIndex];
            }
            StartMotion();
        }

        private void StartMotion()
        {
            jointsDirection = goalJoints - startJoints;
            lastErrorForEachJoint = goalJoints - startJoints;

            // if the start and goal position are identical, we are finished
            if (Math.Abs(jointsDirection.Norm) <= 0.00001)
            {
                finished = true;
            }
            else
            {
                finished = false;
                jointsDirection.Norm = 1;
            }
            moving = true;
        }

        public override void kill()
        {
            base.kill();
                  if(app.Simulation != null)
            {
                app.Simulation.SimulationReset -= ResetSimulation;
                app.Simulation.SimulationStarted -= SimulationStarted;
                IoC.Get<ISimulationService>().PropertyChanged -= ElapsedCallback;

            }
        }

        private void ResetSimulation(object sender, EventArgs e)
        {
            moving = false;
            finished = true;
        }

        private Vector ArrangeJointsToControllerOrder(Vector joints)
        {
            Vector jointsNew = new Vector(joints.Length);
            for (int i = 0; i < joints.Length; i++)
            {
                jointsNew[p.p(i)] = joints[i];
            }
            return jointsNew;
        }

        private Vector ArrangeJointsToVisualComponentsOrder(Vector joints)
        {
            Vector jointsNew = new Vector(joints.Length);
            for (int i = 0; i < joints.Length; i++)
            {
                jointsNew[p.i(i)] = joints[i];
            }
            return jointsNew;
        }

        private void RobotCycle()
        {
            if (!isAlive || !moving)
            {
                tcpSpeed = new Vector(3);
                return;
            }

            try
            {
                if (finished)
                {
                    // next segment available check
                    pathIndex++;
                    if (pathIndex >= resultAngles.Count)
                    {
                        IStringSignal movementFinished = (IStringSignal)manip.component.FindBehavior("MovementFinished");
                        movementFinished.Value = pythonState; // indicate motion done
                        moving = false;
                        Printer.print(component.Name + " finished Motion: " + pythonState);

                        manip.setConfiguration(ArrangeJointsToVisualComponentsOrder(resultAngles[pathIndex - 1]));
                        return;
                    }

                    startJoints = ArrangeJointsToControllerOrder(manip.getConfiguration());
                    goalJoints = resultAngles[pathIndex];
                    StartMotion();
                    return;
                }

                // aktuelle Joints beziehen (in korrekter Reihenfolge)
                joints = ArrangeJointsToControllerOrder(manip.getConfiguration());

                // MaxSpeed bestimmen
                appliedSpeed = Math.Min(demandedSpeed, allowedSpeed);

                MotionPlanRobotDescription.MotionPlanForwardKinematicResult value = mp.getMotionPlanRobotDescription().getVelFK(joints.AsVectorOfDouble(), jointsDirection.AsVectorOfDouble());
                if (!value.success) return;
                Vector cartesianTranslationalSpeed = new Vector(3);

                cartesianTranslationalSpeed[0] = value.x * 1000 / deltaTime;
                cartesianTranslationalSpeed[1] = value.y * 1000 / deltaTime;
                cartesianTranslationalSpeed[2] = value.z * 1000 / deltaTime;

                // Geschwindigkeitsfaktor anpassen
                double speedFactor = appliedSpeed / cartesianTranslationalSpeed.Norm;

                // neue Joints berechnen mit den an die Geschwindigkeit angepassten deltaJoints
                // q_n+1 = q_n + factor * q_dot
                Vector jointsAfterCurrentInterpolation = new Vector(joints.Length);
                for (int i = 0; i < jointsDirection.Length; i++)
                {
                    jointsAfterCurrentInterpolation[i] = joints[i] + jointsDirection[i] * speedFactor;
                }

                Vector currentErrorForEachJoint = goalJoints - jointsAfterCurrentInterpolation;

                finished = true;
                for (int i = 0; i < currentErrorForEachJoint.Length; i++)
                {
                    double currentError = currentErrorForEachJoint[i];
                    double lastError = lastErrorForEachJoint[i];

                    if (!HasSameSign(currentError, lastError) || Math.Abs(currentError) > Math.Abs(lastError))
                    {
                        jointsAfterCurrentInterpolation[i] = goalJoints[i];
                    }
                    else
                    {
                        finished = false;
                    }
                }

                lastErrorForEachJoint = currentErrorForEachJoint;

                tcpSpeed = new Vector(3);
                MotionPlanRobotDescription.MotionPlanForwardKinematicResult before = mp.getMotionPlanRobotDescription().getFK(joints.AsVectorOfDouble(), true);
                MotionPlanRobotDescription.MotionPlanForwardKinematicResult after = mp.getMotionPlanRobotDescription().getFK(jointsAfterCurrentInterpolation.AsVectorOfDouble(), true);
                if (!before.success || !after.success) return;

                tcpSpeed[0] = (after.x - before.x) * 1000 / deltaTime;
                tcpSpeed[1] = (after.y - before.y) * 1000 / deltaTime;
                tcpSpeed[2] = (after.z - before.z) * 1000 / deltaTime;

                // Joints in Visual Components Reihenfolge setzen
                manip.setConfiguration(ArrangeJointsToVisualComponentsOrder(jointsAfterCurrentInterpolation));

            }
            catch (Exception ee)
            {
                Printer.printTimed(ee.Message + "\n" + ee.StackTrace);
            }
        }

        private bool HasSameSign(double x, double y)
        {
            return (x > 0) == (y > 0);
        }
        
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

            if (p == null)
            {
                p = new Permutator(manip.jointCount);
            }
        }
    }
}
