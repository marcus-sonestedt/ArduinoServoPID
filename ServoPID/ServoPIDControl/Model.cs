using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ServoPIDControl
{
    public class Model
    {
        public string PortName { get; set; } = "COM3";
        public bool Enabled { get; set; } = true;
        public ObservableCollection<ServoPidModel> Servos { get; set; } = new ObservableCollection<ServoPidModel>();
    }
}
