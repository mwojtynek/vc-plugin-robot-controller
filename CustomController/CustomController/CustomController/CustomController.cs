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
    class CustomController
    {
        public VisualRobotManipulator manip;

        private IMotionInterpolator interpol;
        private IMotionTarget startTarget;
        private IMotionTarget goalTarget;

        private IMotionTarget kinematics;
        double[] startJoints;
        double[] goalJoints;
        double[] deltaJoints;
        bool reset = false;
        bool finished = false;
        double lastDistance;
        private double demandedSpeed = 200;

        private IApplication _app;

        private double startTime = 0;
        private double haltedTime = 0;

        public readonly double SAMPLE_TIME = 0.01;

        //Auslagern
        ISamplingTimer st;

        public CustomController(IApplication app, VisualRobotManipulator manip)
        {
            try
            {
                _app = app;
                this.manip = manip;

                kinematics = manip.robot.RobotController.CreateTarget();
                kinematics.UseJointValues = false;

                startJoints = new double[manip.robot.RobotController.Joints.Count];
                goalJoints = new double[manip.robot.RobotController.Joints.Count];

                /*
                interpol = manip.robot.RobotController.CreateMotionInterpolator();

                

                startTarget = manip.robot.RobotController.CreateTarget();
                startTarget.CartesianSpeed = 2500.0;
                startTarget.AngularSpeed = 100.0;
                startTarget.JointSpeed = 1.0;
                startTarget.MotionType = MotionType.Joint;

                goalTarget = manip.robot.RobotController.CreateTarget();
                goalTarget.CartesianSpeed = 2500.0;
                goalTarget.AngularSpeed = 100.0;
                goalTarget.JointSpeed = 15.0;
                goalTarget.MotionType = MotionType.Joint;
                */

                //manip.robot.Component.FindFeature("startFrame").TransformationChanged += frameChanged;
                //manip.robot.Component.FindFeature("goalFrame").TransformationChanged += frameChanged;
                //manip.robot.Component.FindFeature("debugFrame").TransformationChanged += frameChanged;
                setFromStartFrame();
                setFromGoalFrame();
                manip.robot.Component.FindFeature("startFrame").TransformationChanged += (e, a) => { setFromStartFrame(); };
                manip.robot.Component.FindFeature("goalFrame").TransformationChanged += (e, a) => { setFromGoalFrame(); };
            } catch(Exception e)
            {
                debug(e.Message);
            }
            IoC.Get<IDebugCall>().NumValue[0] = demandedSpeed; //TODO fix
            IoC.Get<IDebugCall>().DebugNum[0] += changeDemand;
            IoC.Get<IDebugCall>().DebugCall[0] += check;
            //setInterpolator();

            // viel mehr krams

            //später auslagern
            st = app.StatisticsManager.CreateTimer(RobotCycle);
            st.SamplingInterval = SAMPLE_TIME;

            app.Simulation.SimulationStarted += started;
            app.Simulation.SimulationStopped += stopped;
        }


        private void check() {
            IoC.Get<IMessageService>().AppendMessage(demandedSpeed.ToString(), MessageLevel.Warning);
            
        }
        private void changeDemand()
        {
            demandedSpeed = IoC.Get<IDebugCall>().NumValue[0];
        }

        /*
private void jointChanged(object sender, EventArgs e)
{
   test = manip.robot.RobotController.CreateTarget();
   test.UseJointValues = false;
   double[] newJoints = new double[manip.robot.RobotController.Joints.Count];
   for (int i = 0; i < newJoints.Length; i++)
   {
       newJoints[i] = manip.robot.Controller.Joints[i].Value;
   }
   test.SetAllJointValues(newJoints);
   double[] joints = test.GetAllJointValues();
   IoC.Get<IMessageService>().AppendMessage(
       test.TargetMatrix.Px.ToString() + "\t" + joints[0].ToString() + "\n" +
       test.TargetMatrix.Py.ToString() + "\t" + joints[1].ToString() + "\n" +
       test.TargetMatrix.Pz.ToString() + "\t" + joints[2].ToString() + "\n" +
       test.TargetMatrix.GetWPR().X.ToString() + "\t" + joints[3].ToString() + "\n" +
       test.TargetMatrix.GetWPR().Y.ToString() + "\t" + joints[4].ToString() + "\n" +
       test.TargetMatrix.GetWPR().Z.ToString() + "\t" + joints[5].ToString()
       , MessageLevel.Warning);
}*/

        // sollte so allgemein klappen, da startTarget und goalTarget den selben Bezug haben sollten...

        private Matrix WorldToRobot(Matrix from)
        {
            return Matrix.Multiply(kinematics.WorldBaseMatrix.Inverse(), from);
        }

        private double absolute(double[] vector)
        {
            try
            {
                double sum = 0;
                for (int i = 0; i < vector.Length; i++)
                {
                    sum += vector[i] * vector[i];
                }
                return Math.Sqrt(sum);
            } catch(Exception e)
            {
                debug("absolute: " + e.Message);
                return 0;
            }
        }

        private void calcDelta() {
            try
            {
                if (startJoints.Length != goalJoints.Length)
                {
                    throw new ArgumentException();
                }
                deltaJoints = new double[startJoints.Length];
                for (int i = 0; i < startJoints.Length; i++)
                {
                    deltaJoints[i] = goalJoints[i] - startJoints[i];

                }
                double len = absolute(deltaJoints);
                for (int i = 0; i < startJoints.Length; i++)
                {
                    deltaJoints[i] /= len;
                }
                reset = true;
            }catch(Exception e)
            {
                debug("calcDelta: " + e.Message);
            }
        }

        public void setFromStartFrame() {
            //setStart(manip.robot.Component.RootNode.GetFeatureTransformationInWorld(manip.robot.Component.FindFeature("startFrame")));

            kinematics.TargetMatrix = WorldToRobot(manip.robot.Component.RootNode.GetFeatureTransformationInWorld(manip.robot.Component.FindFeature("startFrame")));
            startJoints = kinematics.GetAllJointValues();
            calcDelta();
        }

        public void setFromGoalFrame() {
            //setGoal(manip.robot.Component.RootNode.GetFeatureTransformationInWorld(manip.robot.Component.FindFeature("goalFrame")));

            kinematics.TargetMatrix = WorldToRobot(manip.robot.Component.RootNode.GetFeatureTransformationInWorld(manip.robot.Component.FindFeature("goalFrame")));
            goalJoints = kinematics.GetAllJointValues();
            calcDelta();
        }
        /*
        private void frameChanged(object sender, EventArgs e)
        {
            IFeature frame = sender as IFeature;
            test = manip.robot.RobotController.CreateTarget();
            test.UseJointValues = false;
            test.TargetMatrix = Matrix.Multiply(test.WorldBaseMatrix.Inverse(), manip.robot.Component.RootNode.GetFeatureTransformationInWorld(frame));
            double[] joints = test.GetAllJointValues();
            IoC.Get<IMessageService>().AppendMessage(
                test.TargetMatrix.Px.ToString() + "\t" + joints[0].ToString() + "\n" +
                test.TargetMatrix.Py.ToString() + "\t" + joints[1].ToString() + "\n" +
                test.TargetMatrix.Pz.ToString() + "\t" + joints[2].ToString() + "\n" +
                test.TargetMatrix.GetWPR().X.ToString() + "\t" + joints[3].ToString() + "\n" +
                test.TargetMatrix.GetWPR().Y.ToString() + "\t" + joints[4].ToString() + "\n" +
                test.TargetMatrix.GetWPR().Z.ToString() + "\t" + joints[5].ToString()
                , MessageLevel.Warning);

            
        }*/

        public void setStart(Matrix target) {
            setTarget(target, ref startTarget);
        }

        public void setGoal(Matrix target)
        {
            setTarget(target, ref goalTarget);
        }

        private void setTarget(Matrix target, ref IMotionTarget toChange) {
            
            toChange.UseJointValues = false;
            toChange.TargetMatrix = Matrix.Multiply(toChange.WorldBaseMatrix.Inverse(), target);
        }

        public void setStart(double[] target) {
            setTarget(target, ref startTarget);
        }

        public void setGoal(double[] target) {
            setTarget(target, ref goalTarget);
        }

        private void setTarget(double[] target, ref IMotionTarget toChange)
        {
            
            toChange.UseJointValues = true;
            toChange.SetAllJointValues(target);
        }

        public void setCurrentToStart() {
            setStart(manip.getConfiguration());
        }
        
        public void setInterpolator(bool fromNow = false)
        {
            if (interpol.Targets.Count > 0)
            {
                interpol.SetTargetAt(0, startTarget);
            }
            else
            {
                interpol.AddTarget(startTarget);
            }

            if (interpol.Targets.Count > 1)
            {
                interpol.SetTargetAt(1, goalTarget);
            }
            else
            {
                interpol.AddTarget(goalTarget);
            }
            
            startTime = _app.Simulation.Elapsed;
            haltedTime = 0;
            IoC.Get<IMessageService>().AppendMessage("New Interpolation", MessageLevel.Warning);
        }

        private void stopped(object sender, EventArgs e)
        {
            st.StartStopTimer(false);
            
        }

        private void started(object sender, EventArgs e)
        {
            st.StartStopTimer(true);
            reset = true;
        }

        public static Vector FK(IMotionTarget kinematics, Vector joints)
        {
            Vector pos = new Vector(6);

            kinematics.SetAllJointValues(joints.Elements);
            for (int i = 0; i < 3; i++)
            {
                pos[i] = kinematics.TargetMatrix.GetP()[i];
                pos[i + 3] = kinematics.TargetMatrix.GetWPR()[i];
            }
            return pos;
        }

        public static double cartesianDistance(IMotionTarget kinematics, Vector startJ, Vector stopJ)
        {
            Vector delta = FK(kinematics, startJ).sub(FK(kinematics, stopJ));
            double dist = 0;
            for (int i = 0; i < 3; i++)
            {
                dist += delta[i] * delta[i];
            }
            return Math.Sqrt(dist);

        }

        private void RobotCycle(object sender, EventArgs e)
        {
            /*
            int jointCount;
            try
            {
                jointCount = manip.jointCount;
            }
            catch {
                return;
            }

            double j0 = Math.Sin(2 * Math.PI * _app.Simulation.Elapsed / 10) * 90;
            double[] jointVals = new double[jointCount];
            jointVals[0] = j0;
            jointVals[1] = 45;
            jointVals[2] = 45;
            manip.setConfiguration(jointVals);*/
            Vector before = new Vector(kinematics.JointCount);
            Vector after = new Vector(kinematics.JointCount);
            try
            {
                if (reset)
                {
                    manip.setConfiguration(startJoints);
                    reset = false;
                    finished = false;
                    lastDistance = cartesianDistance(kinematics, new Vector(startJoints), new Vector(goalJoints)); 
                    return;
                }
                if (finished)
                {
                    return;
                }
                double[] joints = manip.getConfiguration();


                double dist = cartesianDistance(kinematics, new Vector(joints), new Vector(goalJoints));
                if (dist < demandedSpeed * SAMPLE_TIME)
                {
                    finished = true;
                }

                Jacobian appro = Jacobian.calcApproJacobian(kinematics, joints);
                double[] cartesianSpeed = appro.multiply(deltaJoints);
                double[] cartesianTransSpeed = new double[3];
                for (int i = 0; i < 3; i++)
                {
                    cartesianTransSpeed[i] = cartesianSpeed[i] / SAMPLE_TIME;
                }
                double factor = demandedSpeed / absolute(cartesianTransSpeed);
                for (int i = 0; i < deltaJoints.Length; i++)
                {
                    joints[i] += deltaJoints[i] * factor;
                }
                before = new Vector(manip.getConfiguration());
                after = new Vector(joints);
                manip.setConfiguration(joints);

            } catch(Exception ee)
            {
                debug("CycleTime: "+ ee.Message);
            }

            double speed = cartesianDistance(kinematics, before, after) / SAMPLE_TIME;
            IoC.Get<IMessageService>().AppendMessage("Speed: " + speed.ToString(), MessageLevel.Warning);

        }

        private void debug(String message)
        {
            IoC.Get<IMessageService>().AppendMessage(_app.Simulation.Elapsed.ToString() + ": " + message, MessageLevel.Warning);
        }

    }
}
