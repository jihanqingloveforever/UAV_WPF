using GMap.NET;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace UAV_WPF
{
    public class PointLatLngAlt
    {
        public double Lat = 0;
        public double Lng = 0;
        public double Alt = 0;
        public PointLatLngAlt(double lat, double lng, double alt)
        {
            this.Lat = lat;
            this.Lng = lng;
            this.Alt = alt;
        }
        public PointLatLngAlt(PointLatLng point)
        {
            this.Lat = point.Lat;
            this.Lng = point.Lng;
            this.Alt = Global.defaultAlt;
        }
        public PointLatLngAlt(PointLatLng point, double alt)
        {
            this.Lat = point.Lat;
            this.Lng = point.Lng;
            this.Alt = alt;
        }
    }
}