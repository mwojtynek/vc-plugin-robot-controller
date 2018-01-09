using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using VisualComponents.Create3D;

namespace CustomController
{
    public static class StaticKinetics
    {
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
            Vector delta = FK(kinematics, startJ) - FK(kinematics, stopJ);
            double dist = 0;
            for (int i = 0; i < 3; i++)
            {
                dist += delta[i] * delta[i];
            }
            return Math.Sqrt(dist);

        }
    }
}
