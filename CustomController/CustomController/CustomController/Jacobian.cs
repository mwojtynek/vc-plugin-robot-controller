using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Caliburn.Micro;
using VisualComponents.Create3D;

namespace CustomController
{
    class Jacobian
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

        public double[] multiply(double[] input)
        {
            if(input.Length != _m)
            {
                throw new ArgumentException();
            }

            double[] result = new double[_m];

            // Matrix-Vector Multiplikation (Skalarprodukt von Zeile und einziger Spalte)
            for (int element = 0; element < _m; element++)
            {
                //Skalarprodukt
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


        

        public static Jacobian calcApproJacobian(IMotionTarget kinematics, double[] joints)
        {
            double samplestep = 1;
            if (kinematics.JointCount != joints.Length)
            {
                throw new ArgumentException();
            }
            Jacobian jacob = new Jacobian(joints.Length);
            Vector current = new Vector(joints);
            Vector canonic;
            for (int k = 0; k < jacob._n; k++)
            {
                canonic = Vector.canonic(jacob._n, k, samplestep);
                Vector ek = CustomController.FK(kinematics, current).sub(CustomController.FK(kinematics, current.sub(canonic)));
                jacob.setBasis(k, ek.multiply(1/samplestep));
            }
            return jacob;
        }

    }
}
