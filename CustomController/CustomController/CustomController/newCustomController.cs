using System;
using System.Text;

using System.ComponentModel.Composition;
using VisualComponents.UX.Shared;
using System.Runtime.InteropServices;

using Caliburn.Micro;

using VisualComponents.Create3D;

using RosiTools.Debugger;
using RosiTools.Printer;

namespace CustomController
{
    public class newCustomController
    {
        public int id { get; private set; }

        // // // Conversion Helper Methods

        private Matrix KDLFrameToMatrix(double[] frame) {
            if (frame.Length != 12)
            {
                throw new ArgumentException("The input is not a frame...");
            }
            
            Vector3 n = new Vector3();
            Vector3 o = new Vector3();
            Vector3 a = new Vector3();
            Vector3 p = new Vector3();
            for (int r = 0; r < 3; r++)
            {
                n[r] = frame[r * 3];
                o[r] = frame[r * 3 + 1];
                a[r] = frame[r * 3 + 2];
                p[r] = frame[9 + r];
            }
            return new Matrix(n, o, a, p);
        }

        private Vector MatrixToVector(Matrix frame) {
            Vector3 p = frame.GetP();
            Vector3 rot = frame.GetWPR();
            Vector ret = new Vector(6);
            for (int i = 0; i < 3; i++) {
                ret[i] = p[i];
                ret[i + 3] = rot[i];
            }
            return ret;
        }

        /*
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
        }*/

        public newCustomController(double[] DH) {
            if (DH.Length % 4 != 0) {
                throw new ArgumentException("DH must be a multiplicative of 4");
            }

            int jointCount = DH.Length / 4;
            id = AddRobot(DH, jointCount);

        }
        
        // // // // // Calculation Methods

        public Vector FKSpeed(Vector joints, Vector jointsDot) {
            double[] twist = new double[6];
            SpeedFK(id, joints.Elements, jointsDot.Elements, twist);
            return new Vector(twist);
        }

        public Vector FK(Vector joints) {
            double[] frame = new double[12];
            FK(this.id, joints.Elements, frame);
            return MatrixToVector(KDLFrameToMatrix(frame));
        }


        [DllImport("CustomRobot.dll")]
        static extern int AddRobot(double[] DH_Data, int jointCount, double maxSpeed = 2.0, double maxAcceleration = 5.0, double maxJerk = 10.0);
        [DllImport("CustomRobot.dll")]
        static extern int FK(int id, double[] joints, double[] frame);
        [DllImport("CustomRobot.dll")]
        static extern int SpeedFK(int id, double[] joints, double[] jointsDot, double[] twist);
    }
}
