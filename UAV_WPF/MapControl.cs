using GMap.NET.WindowsPresentation;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Shapes;

namespace UAV_WPF
{
   public class MapControl : GMapControl
    {
        private Rectangle rec;

        public Rectangle Rec { get => rec; set => rec = value; }

    }
}
