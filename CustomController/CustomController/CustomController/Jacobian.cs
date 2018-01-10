using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Caliburn.Micro;
using VisualComponents.Create3D;

namespace CustomController
{
    public class Jacobian
    {

        private int _m = 6;
        private int _n;

        private Vector[] basis;
        public Jacobian(int n) {
            _n = n;
            basis = new Vector[_n];
            for (int i = 0; i < basis.Length; i++)
            {
                basis[i] = new Vector(_m);
            }
        }

        public Vector multiply(Vector input)
        {
            if(input.Length != _n)
            {
                throw new ArgumentException();
            }

            Vector result = new Vector(_m);

            // Matrix-Vector Multiplikation (Skalarprodukt von Zeile und einziger Spalte)
            for (int element = 0; element < _m; element++)
            {
                //Skalarprodukt - theoretisch kann sum weggelassen werden. Ich mach mir mal vor, dass das so schneller bearbeitet wird...
                double sum = 0;
                for (int basis_k = 0; basis_k < _n; basis_k++)
                {
                    sum += basis[basis_k][element] * input[basis_k];
                }
                result[element] = sum;
            }
            return result;
        }

        public void setBasis(int i, Vector basis_i)
        {
            if (basis_i.Length != 6)
            {
                throw new ArgumentException();
            } 
            if(i >= _n)
            {
                throw new ArgumentOutOfRangeException();
            }
            basis[i] = basis_i;
        }


        

        public static Jacobian calcApproJacobian(IMotionTarget kinematics, Vector joints)
        {
            double samplestep = 1;
            if (kinematics.JointCount != joints.Length)
            {
                throw new ArgumentException();
            }
            Jacobian jacob = new Jacobian(joints.Length);
            
            Vector canonic;
            for (int k = 0; k < jacob._n; k++)
            {
                canonic = Vector.canonic(jacob._n, k, samplestep);
                Vector ek = StaticKinetics.FK(kinematics, joints) - StaticKinetics.FK(kinematics, joints - canonic);
                jacob.setBasis(k, ek * (1/samplestep));
            }
            return jacob;
        }

    }
}
