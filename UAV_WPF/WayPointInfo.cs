using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace UAV_WPF
{
   public class WayPointInfo
    {
        public string command { get; set; }
        public double lat { get; set; }
        public double  lng { get; set; }
        public double heigh { get; set; }
        public DateTime stayTime { get; set; }
    }
}
