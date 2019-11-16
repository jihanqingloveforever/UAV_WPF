using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading.Tasks;
using System.Xml.Linq;

namespace UAV_WPF.Tools
{
    class Helper
    {
        #region 数据帧转换为字节数组
        /// <summary>
        /// 数据帧转换为字节数组
        /// </summary>
        /// <param name="structObj">完整数据帧结构体</param>
        /// <returns></returns>
        public static byte[] StructToByteArray(MAVLink.MavLink.MavLinkFrame dataFrame)
        {
            byte[] mavLinkFrame = new byte[dataFrame.Len + 8];
            mavLinkFrame[0] = dataFrame.STX;
            mavLinkFrame[1] = dataFrame.Len;
            mavLinkFrame[2] = dataFrame.SEQ;
            mavLinkFrame[3] = dataFrame.SYSID;
            mavLinkFrame[4] = dataFrame.CompID;
            mavLinkFrame[5] = dataFrame.MsgID;
            for (int i = 0; i < dataFrame.Len; i++)
            {
                mavLinkFrame[i + 6] = dataFrame.Data[i];
            }
            mavLinkFrame[mavLinkFrame.Length - 2] = dataFrame.Chk_A;
            mavLinkFrame[mavLinkFrame.Length - 1] = dataFrame.Chk_B;
            return mavLinkFrame;
        }
        #endregion

        #region 除去校验码的数据帧转换为字节数组
        /// <summary>
        /// 除去校验码的数据帧转换为字节数组
        /// </summary>
        /// <param name="structObj">除去校验码的数据帧结构体</param>
        /// <returns></returns>
        public static byte[] StructToByteArray_2(MAVLink.MavLink.MavLinkFrame_2 dataFrame)
        {
            byte[] mavLinkFrame = new byte[dataFrame.Len + 6];
            mavLinkFrame[0] = dataFrame.STX;
            mavLinkFrame[1] = dataFrame.Len;
            mavLinkFrame[2] = dataFrame.SEQ;
            mavLinkFrame[3] = dataFrame.SYSID;
            mavLinkFrame[4] = dataFrame.CompID;
            mavLinkFrame[5] = dataFrame.MsgID;
            for (int i = 0; i < dataFrame.Len; i++)
            {
                mavLinkFrame[i + 6] = dataFrame.Data[i];
            }
            return mavLinkFrame;
        }
        #endregion

        #region 判断两个字节数组是否相等
        /// <summary>
        /// 判断两个字节数组是否相等
        /// </summary>
        /// <param name="b1"></param>
        /// <param name="b2"></param>
        /// <returns></returns>
        public static bool IsEqual(byte[] b1, byte[] b2)
        {
            if (b1.Length != b2.Length)
                return false;
            if (b1 == null || b2 == null)
                return false;
            for (int i = 0; i < b1.Length; i++)
            {
                if (b1[i] != b2[i])
                    return false;
            }
            return true;
        }
        #endregion

        #region 字节数组转化为数字
        /// <summary>
        /// 字节数组转化为数字
        /// </summary>
        /// <param name="byteArr">数据帧<param>
        /// <param name="start">在数据帧的中起始位置</param>
        /// <param name="end">结束位置</param>
        /// <param name="type">类型 ushort short float uint</param>
        /// <returns></returns>
        public static object Turn(byte[] byteArr, int start, int end, DataType type)
        {
            switch (end - start)
            {
                case 1:
                    if (type == DataType.U_Short)
                    {
                        return BitConverter.ToUInt16(byteArr, start);
                    }
                    else//short
                    {
                        return BitConverter.ToInt16(byteArr, start);
                    }
                case 3:
                    if (type == DataType.Float)
                    {
                        return BitConverter.ToSingle(byteArr, start);
                    }
                    else if (type == DataType.U_Int)
                    {
                        return BitConverter.ToUInt32(byteArr, start);

                    }
                    else //int
                    {
                        return BitConverter.ToInt32(byteArr, start);
                    }

                case 7:
                    return BitConverter.ToUInt64(byteArr, start);
                default:
                    return 0;
            }

        }
        #endregion

        #region 判断传入的字符是不是double
        public static bool IsDouble(string str)
        {
            return Regex.IsMatch(str, @"^[+-]?\d*[.]?\d*$");
        }
        #endregion



        public static bool IsInt(string str)
        {
            return Regex.IsMatch(str, @"^\d+$");
        }
        #region 将弧度转化为角度
        /// <summary>
        /// 将弧度转化为角度
        /// </summary>
        /// <param name="angle">弧度</param>
        /// <returns></returns>
        public static float RadianToAngle(float angle)
        {
            return (float)(angle * 180 / Math.PI);
        }
        #endregion

        #region 根据两点的经纬度求距离
        /// <summary>
        /// 两个坐标点经纬度
        /// </summary>
        /// <param name="lat1"></param>
        /// <param name="lng1"></param>
        /// <param name="lat2"></param>
        /// <param name="lng2"></param>
        /// <returns>两点之间的距离</returns>
        public static double GetDistance(double lat1, double lng1, double lat2, double lng2)
        {
            double EARTH_RADIUS = 6378.137;
            double radLat1 = rad(lat1);
            double radLat2 = rad(lat2);
            double a = radLat1 - radLat2;
            double b = rad(lng1) - rad(lng2);
            double s = 2 * Math.Asin(Math.Sqrt(Math.Pow(Math.Sin(a / 2), 2) + Math.Cos(radLat1) * Math.Cos(radLat2) * Math.Pow(Math.Sin(b / 2), 2)));
            s = s * EARTH_RADIUS;
            s = Math.Round(s * 10000) / 10000;
            return s;
        }
        /// <summary>
        /// 把相应的经纬度转换成弧度求两点的距离
        /// </summary>
        /// <param name="d"></param>
        /// <returns></returns>
        public static double rad(double d)
        {
            double PI = 3.1415926;
            return d * PI / 180.0;
        }
        #endregion

        #region 将用户最后一次写的数据，写入到配置文件中
        /// <summary>
        /// 将用户最后一次写的数据，写入到配置文件中
        /// </summary>
        /// <param name="configName"></param>
        /// <param name="value"></param>
        public static void WriteDataToXml(Dictionary<string, string> config)
        {
            XDocument xDocument = new XDocument();
            //XDeclaration xDeclaration = new XDeclaration("1.0", "UTF-8", null);
            //xDocument.Add(xDeclaration);
            XElement rootElement = new XElement("Config");
            xDocument.Add(rootElement);
            foreach (var item in config)
            {
                XElement configElement = new XElement(item.Key);
                configElement.Value = item.Value;
                rootElement.Add(configElement);
            }
            xDocument.Save("Config.Xml");
        }
        #endregion
    }
    public enum DataType
    {
        /// <summary>
        /// 无符号16位
        /// </summary>
        U_Short,
        /// <summary>
        /// 有符号16位
        /// </summary>
        S_Short,
        /// <summary>
        /// 无符号32位
        /// </summary>
        U_Int,
        /// <summary>
        /// 有符号32位
        /// </summary>
        S_Int,
        /// <summary>
        /// 单精度浮点数
        /// </summary>
        Float,
        /// <summary>
        /// 无符号64位
        /// </summary>
        U_Int64,
        /// <summary>
        /// 有符号64位
        /// </summary>
        S_Int64,
    }
}
