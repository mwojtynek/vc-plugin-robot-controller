using System;
using System.Text;

using System.ComponentModel.Composition;
using VisualComponents.UX.Shared;
using System.Runtime.InteropServices;

using Caliburn.Micro;

using RosiTools.Debugger;
using RosiTools.Printer;

namespace CustomController
{
    [Export(typeof(IPlugin))]
    public class newCustomController : IPlugin
    {
        int id = -1;
        double[] joints = new double[6];

        public void Exit()
        {
        }

        private void setJoint()
        {
            int jn = (int)IoC.Get<IDebugCall>().NumValue[0];
            try
            {
                joints[jn] = IoC.Get<IDebugCall>().NumValue[1];
            }
            catch
            {
                Printer.print("Could not assign new Joint Value");
            }
        }

        private void doFK()
        {
            double[] frame = new double[12];
            FK(id, joints, frame);
            StringBuilder bld = new StringBuilder();
            bld.Append("Joints: ");
            for (int i = 0; i < joints.Length; i++)
            {
                bld.Append(joints[i].ToString() + " ");
            }
            bld.AppendLine("");
            for (int r = 0; r < 3; r++)
            {
                for (int c = 0; c < 4; c++)
                {
                    bld.AppendFormat("{0:###0.###}      ", frame[r + c * 3]);
                }
                bld.AppendLine("");
            }
            Printer.print(bld.ToString());
        }

        public void Initialize()
        {

            double[] DH = { 0, -90, 375,0,
                            290, 0, 20, -90,
                            0, 90, 0, 90,
                            0,-90, 310, 0,
                            0, 90, 0, 0,
                            0, 0, 70,0};
            try
            {
                id = AddRobot(DH, 6);
            }
            catch (Exception e)
            {
                Printer.print(e.GetType().ToString());
                Printer.print(e.Message);
                Printer.print(e.StackTrace);
            }
            IoC.Get<IDebugCall>().DebugCall[0] += setJoint;
            IoC.Get<IDebugCall>().DebugCall[1] += doFK;
            Printer.print("Gotta new Controller with ID: " + id.ToString());

        }

        [DllImport("CustomRobot.dll")]
        static extern int AddRobot(double[] DH_Data, int jointCount, double maxSpeed = 2.0, double maxAcceleration = 5.0, double maxJerk = 10.0);
        [DllImport("CustomRobot.dll")]
        static extern int FK(int id, double[] joints, double[] frame);
    }
}
