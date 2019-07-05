using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ServoPIDControl
{
    public class ServoPidModel
    {
        public int Id { get; set; }
        public float P { get; set; }
        public float I { get; set; }
        public float D { get; set; }
        public float DLambda { get; set; } = 1.0f;

        public float SetPoint { get; set; }
        public float Input { get; set; }
        public float Output { get; set; }

        public float Integrator { get; set; }
        public float DFiltered { get; set; }

        public float InputScale { get; set; } = 1;
        public float InputBias { get; set; } = 0;
    }
}