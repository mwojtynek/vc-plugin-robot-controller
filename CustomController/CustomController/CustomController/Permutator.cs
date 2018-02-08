using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CustomController
{
    class Permutator
    {
        private int[] fp;
        private int[] ip;

        public Permutator(int[] table) {
            fp = table;
            ip = new int[table.Length];
            for (int i = 0; i < table.Length; i++) {
                ip[fp[i]] = i;
            }
        }

        public Permutator(int size) {
            fp = new int[size];
            ip = new int[size];
            for (int i = 0; i < size; i++) {
                fp[i] = i;
                ip[i] = i;
            }
        }

        public int p(int p) {
            return fp[p];
        }
        public int i(int p) {
            return ip[p];
        }
    }
}
