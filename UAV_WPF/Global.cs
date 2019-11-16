using GMap.NET;
using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace UAV_WPF
{
    class Global
    {
        public static int baudRate;
        public static string portsName;
        public static bool isConn = false;
        public static byte sysID;
        public static byte compID;
        public static byte seq = 0;
        public static SerialPort serialPort;
//        public static List<PointLatLngAlt> pointList = new List<PointLatLngAlt>();
        public static double defaultAlt = 5;//默认的高度
        public static double monifyAlt;//修改的高度
        public static double landAlt = 1;//降落高度

        public static MavStatus mavStatus;
        public static float takeOffAlt = 10;//默认起飞高度
        public static int bufferNum = 0;//缓存区字节数，用于判断是不是连接上
        public static string mapName = "谷歌地图";//
        public static AccessMode mapReadWay = AccessMode.ServerAndCache;//地图读取方式，默认从服务器和缓存中读取。
        public static PointLatLng nowLocation;//当用户连接上飞控的时候，让地图的中心在当前位置
//        public static PointLatLngAlt home = new PointLatLngAlt(0, 0, 0);//home点的位置
        public static bool alCalibrationLevel = false;
        public static string CalibrationAccelInfo = "";
        public static string test = "test";
        string str;
    }
}
