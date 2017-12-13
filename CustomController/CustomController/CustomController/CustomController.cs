using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using VisualComponents.Create3D;
using Caliburn.Micro;

namespace RobotController
{
    class CustomController
    {
        public VisualRobotManipulator manip;

        private IMotionInterpolator interpol;
        private IMotionTarget startTarget;
        private IMotionTarget goalTarget;

        private IApplication _app;

        private double startTime = 0;
        private double haltedTime = 0;

        //Auslagern
        ISamplingTimer st;

        public CustomController(IApplication app, VisualRobotManipulator manip)
        {
            _app = app;
            this.manip = manip;

            
           interpol = manip.robot.RobotController.CreateMotionInterpolator();

            startTarget = manip.robot.RobotController.CreateTarget();
            startTarget.CartesianSpeed = 1.0;
            startTarget.AngularSpeed = 1.0;
            startTarget.JointSpeed = 1.0;
            startTarget.MotionType = MotionType.Joint;

            goalTarget = manip.robot.RobotController.CreateTarget();
            goalTarget.CartesianSpeed = 1.0;
            goalTarget.AngularSpeed = 1.0;
            goalTarget.JointSpeed = 1.0;
            goalTarget.MotionType = MotionType.Joint;
        

            manip.robot.Component.FindFeature("startFrame").TransformationChanged += frameChanged;
                manip.robot.Component.FindFeature("goalFrame").TransformationChanged += frameChanged;
           
                setFromStartFrame();
                setFromGoalFrame();
            try
            {
                setInterpolator();
            }
            catch (Exception e){
                IoC.Get<IMessageService>().AppendMessage(e.Message, MessageLevel.Warning);
            }
            // viel mehr krams

            //später auslagern
            st = app.StatisticsManager.CreateTimer(RobotCycle);
            st.SamplingInterval = 0.01;

            app.Simulation.SimulationStarted += started;
            app.Simulation.SimulationStopped += stopped;
        }

        // sollte so allgemein klappen, da startTarget und goalTarget den selben Bezug haben sollten...
        private Matrix toRobotBase(Matrix WorldTTarget) {
            return Matrix.Multiply(startTarget.WorldBaseMatrix.Inverse(), WorldTTarget);
        }

        private void setFromStartFrame() {
            setStart(toRobotBase(manip.robot.Component.RootNode.GetFeatureTransformationInWorld(manip.robot.Component.FindFeature("startFrame"))));
        }

        private void setFromGoalFrame() {
            setGoal(toRobotBase(manip.robot.Component.RootNode.GetFeatureTransformationInWorld(manip.robot.Component.FindFeature("goalFrame"))));
        }

        private void frameChanged(object sender, EventArgs e)
        {
            
        }

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

            IMotionTarget next = manip.robot.RobotController.CreateTarget();
            interpol.Interpolate(_app.Simulation.Elapsed - startTime, ref next);

            IMotionTester tester = manip.robot.RobotController.GetMotionTester();
            tester.CurrentTarget = next;
            /*
            for (int i = 0; i < 3; i++)
            {
                IoC.Get<IMessageService>().AppendMessage((next.TargetMatrix.GetP())[i].ToString(), MessageLevel.Warning);
            }*/

        }

    }
}
