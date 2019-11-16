using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace UAV_WPF
{
    public struct LocationWayPoint
    {
        public float alt;
        /// <summary>
        /// 对应任务项的Command 就是指令类型
        /// </summary>
        public byte id;
        public double lat;
        public double lng;
        public byte options;//坐标系的选择
        /// <summary>
        /// 对应任务项中的参数
        /// </summary>
        public float p1;
        public float p2;
        public float p3;
        public float p4;
    }
}
