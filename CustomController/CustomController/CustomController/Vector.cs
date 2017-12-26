using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CustomController
{
    class Vector
    {
        private int _n;
        private double[] vals;

        public double[] Elements
        {
            get
            {
                return vals;
            }
        }

        public int Length
        {
            get {
                return _n;
            }
        }
        public double this[int i]
        {
            get
            {
                return vals[i];
            }
            set
            {
                vals[i] = value;
            }
        }


        public Vector(int n)
        {
            _n = n;
            vals = new double[_n];
        }

        public Vector(double[] elements)
        {
            _n = elements.Length;
            vals = new double[_n];
            this.set(elements);
        }

        public static Vector canonic(int n, int i, double norm = 1.0)
        {
            if (i >= n)
            {
                throw new ArgumentOutOfRangeException();
            }

            Vector result = new Vector(n);
            result[i] = norm;
            return result;
        }

        public void set(double[] value) {
            if (value.Length != Length)
            {
                throw new ArgumentException();
            }

            for (int i = 0; i < Length; i++)
            {
                vals[i] = value[i];
            }
        }

        private void checkLength(Vector second)
        {
            if (second.Length != Length)
            {
                throw new ArgumentException();
            }

        }

        public Vector add(Vector second)
        {
            checkLength(second);

            Vector result = new Vector(Length);

            for (int i = 0; i < Length; i++)
            {
                result[i] = vals[i] + second[i];
            }

            return result;
        }

        public Vector sub(Vector second)
        {
            checkLength(second);

            Vector result = new Vector(Length);

            for (int i = 0; i < Length; i++)
            {
                result[i] = vals[i] - second[i];
            }

            return result;
        }

        public Vector multiply(Vector second)
        {
            checkLength(second);

            Vector result = new Vector(Length);

            for (int i = 0; i < Length; i++)
            {
                result[i] = vals[i] * second[i];
            }

            return result;

        }

        public Vector multiply(double factor)
        {
            Vector result = new Vector(Length);

            for (int i = 0; i < Length; i++)
            {
                result[i] = vals[i] * factor;
            }

            return result;

        }

        public double Norm
        {
            get
            {
                double sum = 0;
                for (int i=0; i < Length; i++)
                {
                    sum += vals[i] * vals[i];
                }
                return Math.Sqrt(sum);
            }
            set
            {
                this.multiply(value / Norm);
            }
        }
    }
}
