using MAVLink;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace UAV_WPF
{
    public class MavLinkInterface
    {
        MavStatus mavStatus = new MavStatus();

        #region 组装和发送数据帧
        /// <summary>
        /// 组装和发送数据帧 
        /// </summary>
        /// <param name="msgID">消息包的编号</param>
        /// <param name="structData">消息包的结构体</param>
        public void AssembleAndSendFrame(byte msgID, object structData)
        {
            //if (!Global.isConn)
            //{
            //    return;
            //}
            //if (msgID == (byte)MAVLink.MavLink.MAVLINK_MSG_ID.MISSION_REQUEST_LIST ||
            //        msgID == (byte)MAVLink.MavLink.MAVLINK_MSG_ID.MISSION_REQUEST_PARTIAL_LIST ||
            //        msgID == (byte)MAVLink.MavLink.MAVLINK_MSG_ID.MISSION_REQUEST ||
            //        msgID == (byte)MAVLink.MavLink.MAVLINK_MSG_ID.PARAM_REQUEST_LIST ||
            //        msgID == (byte)MAVLink.MavLink.MAVLINK_MSG_ID.PARAM_REQUEST_READ ||
            //        msgID == (byte)MAVLink.MavLink.MAVLINK_MSG_ID.RALLY_FETCH_POINT ||
            //        msgID == (byte)MAVLink.MavLink.MAVLINK_MSG_ID.FENCE_FETCH_POINT)
            //{
            //}
            //else
            //{
            //    return;
            //}
            //byte[] dataPacket = MavlinkUtil.StructureToByteArray(structData);
            //byte[] mavLinkFrame = new byte[dataPacket.Length + 8];
            //mavLinkFrame[0] = 254;
            //mavLinkFrame[1] = (byte)dataPacket.Length;
            //mavLinkFrame[2] = Global.seq;
            //mavLinkFrame[3] = Global.sysID;
            //mavLinkFrame[4] = Global.compID;
            //mavLinkFrame[5] = msgID;
            //dataPacket.CopyTo(mavLinkFrame, 6);
            ////算校验和
            //ushort checksum = MavlinkCRC.crc_calculate(mavLinkFrame, mavLinkFrame[1] + 6);
            //checksum = MavlinkCRC.crc_accumulate(MavlinkCRC.MAVLINK_MESSAGE_CRCS[msgID], checksum);
            //byte ck_a = (byte)(checksum & 0xFF); ///< High byte
            //byte ck_b = (byte)(checksum >> 8); ///< Low byte
            //mavLinkFrame[mavLinkFrame.Length - 1] = ck_b;
            //mavLinkFrame[mavLinkFrame.Length - 2] = ck_a;
            //Global.serialPort.Write(mavLinkFrame, 0, mavLinkFrame.Length);
            //if (Global.seq > 255)
            //{
            //    Global.seq = 0;
            //}
            //else
            //{
            //    Global.seq++;
            //}
        }
        #endregion

        /// <summary>
        /// 发送航点
        /// </summary>
        /// <param name="lwp">地图上的点</param>
        /// <param name="index">序号</param>
        /// <param name="frame">mavlink帧格式</param>
        /// <param name="current"></param>
        /// <param name="autocontinue"></param>
        /// <returns></returns>
        
        //public MAVLink.MavLink.MAV_MISSION_RESULT SetWayPoint(LocationWayPoint lwp, ushort index, MavLink.MAV_FRAME frame, byte current = 0, byte autocontinue = 1)
        //{
        //    MAVLink.MavLink.mavlink_Mission_Item mission_Item = new MavLink.mavlink_Mission_Item();
        //    mission_Item.target_system = Global.sysID;
        //    mission_Item.target_component = Global.compID;
        //    mission_Item.command = lwp.id;
        //    mission_Item.current = current;
        //    mission_Item.autocontinue = autocontinue;
        //    mission_Item.frame = (byte)frame;
        //    mission_Item.y = (float)(lwp.lng);
        //    mission_Item.x = (float)(lwp.lat);
        //    mission_Item.z = (float)(lwp.alt);
        //    mission_Item.param1 = lwp.p1;
        //    mission_Item.param2 = lwp.p2;
        //    mission_Item.param3 = lwp.p3;
        //    mission_Item.param4 = lwp.p4;
        //    mission_Item.seq = index;
        //    return SetWayPoint(mission_Item);
        //}

        //public MavLink.MAV_MISSION_RESULT SetWayPoint(MavLink.mavlink_Mission_Item mission_Item)
        //{
        //    AssembleAndSendFrame((byte)MavLink.MAVLINK_MSG_ID.MISSION_ITEM, mission_Item);
        //    DateTime start = DateTime.Now;
        //    //没有发送成功时，重复发送次数
        //    int retryNum = 10;
        //    while (true)
        //    {
        //        if (!(start.AddMilliseconds(400) > DateTime.Now))
        //        {
        //            if (retryNum > 0)
        //            {
        //                AssembleAndSendFrame((byte)MavLink.MAVLINK_MSG_ID.MISSION_ITEM, mission_Item);
        //                start = DateTime.Now;
        //                retryNum--;
        //                continue;
        //            }
        //        }
        //        byte[] buffer = ReadFrame();
        //        if (buffer[5] == (byte)MavLink.MAVLINK_MSG_ID.MISSION_REQUEST)
        //        {
        //            var answer = buffer.ByteArrayToStructure<MavLink.mavlink_mission_request>(6);
        //            if (answer.seq == (mission_Item.seq + 1))
        //            {
        //                if (mission_Item.current == 2)
        //                {
        //                    mavStatus.GuidedMode = mission_Item;
        //                }
        //                else if (mission_Item.current == 3)
        //                {
        //                }
        //                else
        //                {
        //                    mavStatus.missionList[mission_Item.seq] = mission_Item;
        //                }
        //                return MavLink.MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED;
        //            }
        //            else
        //            {
        //                start = DateTime.MinValue;
        //            }
        //        }
        //        else if (buffer[5] == (byte)MavLink.MAVLINK_MSG_ID.MISSION_ACK)
        //        {
        //            var ans = buffer.ByteArrayToStructure<MavLink.mavlink_Mission_Ack>(6);
        //            if (mission_Item.current == 2)
        //            {
        //                mavStatus.GuidedMode = mission_Item;
        //            }
        //            else if (mission_Item.current == 3)
        //            {
        //            }
        //            else
        //            {
        //                mavStatus.missionList[mission_Item.seq] = mission_Item;
        //            }
        //        }
        //    }
        //}

        //public byte[] ReadFrame()
        //{
        //    return Global.mavLinkFrame;
        //}

        #region 发送命令
        /// <summary>
        /// 发送命令
        /// </summary>
        /// <param name="action">命令</param>
        /// <param name="p1"></param>
        /// <param name="p2"></param>
        /// <param name="p3"></param>
        /// <param name="p4"></param>
        /// <param name="p5"></param>
        /// <param name="p6"></param>
        /// <param name="p7"></param>
        /// <returns></returns>
        //public bool DoCommand(MAVLink.MavLink.MAV_CMD action, float p1, float p2, float p3, float p4, float p5, float p6, float p7)
        //{
        //    MavLink.mavlink_Command_Long command = new MavLink.mavlink_Command_Long();
        //    command.target_component = Global.compID;
        //    command.target_system = Global.sysID;
        //    command.command = (ushort)action;
        //    command.param1 = p1;
        //    command.param2 = p2;
        //    command.param3 = p3;
        //    command.param4 = p4;
        //    command.param5 = p5;
        //    command.param6 = p6;
        //    command.param7 = p7;
        //    AssembleAndSendFrame((byte)MavLink.MAVLINK_MSG_ID.COMMAND_LONG, command);
        //    DateTime start = DateTime.Now;
        //    int retryNum = 3;
        //    int timeout = 2000;
        //    //起飞前设备校准
        //    if (action == MavLink.MAV_CMD.PREFLIGHT_CALIBRATION && p5 == 1)
        //    {
        //        return true;
        //    }
        //    else if (action == MAVLink.MavLink.MAV_CMD.PREFLIGHT_CALIBRATION)
        //    {
        //        retryNum = 1;
        //        timeout = 25000;
        //    }
        //    //飞控关机或者重启
        //    else if (action == MAVLink.MavLink.MAV_CMD.PREFLIGHT_REBOOT_SHUTDOWN)
        //    {
        //        AssembleAndSendFrame((byte)MavLink.MAVLINK_MSG_ID.COMMAND_LONG, command);
        //        return true;
        //    }
        //    //机载组件的启动和关闭
        //    else if (action == MAVLink.MavLink.MAV_CMD.COMPONENT_ARM_DISARM)
        //    {
        //        timeout = 10000;
        //    }
        //    else if (action == MAVLink.MavLink.MAV_CMD.PREFLIGHT_CALIBRATION && p6 == 1)
        //    {
        //        AssembleAndSendFrame((byte)MavLink.MAVLINK_MSG_ID.COMMAND_LONG, command);
        //        return true;
        //    }
        //    while (true)
        //    {
        //        if (!(start.AddMilliseconds(timeout) > DateTime.Now))
        //        {
        //            if (retryNum > 0)
        //            {
        //                AssembleAndSendFrame((byte)MavLink.MAVLINK_MSG_ID.COMMAND_LONG, command);
        //                start = DateTime.Now;
        //                retryNum--;
        //                continue;
        //            }
        //        }
        //        byte[] buffer = ReadFrame();
        //        if (buffer[5] == (byte)MavLink.MAVLINK_MSG_ID.COMMAND_ACK)
        //        {
        //            var ack = buffer.ByteArrayToStructure<MAVLink.MavLink.mavlink_Command_Ack>(6);
        //            if (ack.result == (byte)MavLink.MAV_Result.ACCEPTED)
        //            {
        //                return true;
        //            }
        //            else
        //            {
        //                return false;
        //            }
        //        }
        //    }
        //}
        #endregion

        //public bool SetParam(string paraName, double value, bool force = false)
        //{
        //    if (!mavStatus.paraDic.ContainsKey(paraName))
        //    {
        //        return false;
        //    }
        //    if (mavStatus.paraDic[paraName] == value)
        //    {
        //        return true;
        //    }
        //    MavLink.mavlink_Param_Set param_Set = new MavLink.mavlink_Param_Set();

        //}


        //public Dictionary<string, double> getParamList()
        //{
        //    List<int> paramIndexList = new List<int>();
        //    mavStatus.paraDic = new Dictionary<string, double>();
        //    MavLink.mavlink_Mission_Request_List req = new MavLink.mavlink_Mission_Request_List();
        //    req.target_system = Global.sysID;
        //    req.target_component = Global.compID;
        //    //发送 请求传送所有参数-----如果发送了，对方没有收到，怎么办？
        //    AssembleAndSendFrame((byte)MavLink.MAVLINK_MSG_ID.PARAM_REQUEST_LIST, req);
        //    byte[] buffer;
        //    int paramCount=1;//参数总数
        //    DateTime now = DateTime.Now;
        //    StreamWriter sw = new StreamWriter(new FileStream(@"C:\Users\wangzheng\Desktop\读取.txt", FileMode.Create));
        //    while (true)
        //    {
        //        buffer = ReadFrame();
        //        for(int i=0;i<buffer.Length;i++)
        //        {
        //            sw.Write(buffer[i] + "  ");
        //        }
        //        sw.WriteLine();
        //        if (buffer[5] == (byte)MavLink.MAVLINK_MSG_ID.PARAM_VALUE && now.AddMilliseconds(600) > DateTime.Now)
        //        {
        //            var ans = buffer.ByteArrayToStructure<MavLink.mavlink_Param_Value>(6);
        //            string paramID = System.Text.ASCIIEncoding.ASCII.GetString(ans.param_id);
        //            paramCount = (ans.param_count);
        //            int pos = paramID.IndexOf('\0');
        //            if (pos != -1)
        //            {
        //                paramID = paramID.Substring(0, pos);
        //            }
        //            //如果字典中不包含参数名字
        //            if (!mavStatus.paraDic.ContainsKey(paramID))
        //            {
        //                mavStatus.paraDic.Add(paramID, ans.param_value);
        //            }
        //            //如果字典中包含参数名字---可能一次发送2遍,以最后发来的为准
        //            else
        //            {
        //                mavStatus.paraDic[paramID] = ans.param_value;
        //            }
        //            //列表中不包含 参数的索引
        //            if (!paramIndexList.Contains(ans.param_index))
        //            {
        //                paramIndexList.Add(ans.param_index);
        //            }

        //        }
        //        else if (now.AddMilliseconds(600) < DateTime.Now)
        //        {
        //            break;//超时之后 哪个没有发过来，然后自己一个一个请求
        //        }
        //        else
        //        {
        //            continue;
        //        }
        //    }
        //    for (short i = 0; i < paramCount; i++)
        //    {
        //        //如果有的参数没有发过来
        //        if (!paramIndexList.Contains(i))
        //        {
        //            //用20号  发送指定参数
        //            MavLink.mavlink_Param_Request_Read req2 = new MavLink.mavlink_Param_Request_Read();
        //            req2.target_component = Global.compID;
        //            req2.target_system = Global.sysID;
        //            req2.param_index = i;
        //            req2.param_id = new byte[] { 0x0 };
        //            Array.Resize(ref req2.param_id, 16);
        //            for (int j = 0; j < 3; j++)//防止没有发送成功 发3次
        //            {
        //                AssembleAndSendFrame((byte)MavLink.MAVLINK_MSG_ID.PARAM_REQUEST_READ, req2);
        //            }
        //            DateTime now2 = DateTime.Now;
        //            while (true)
        //            {
        //                buffer = ReadFrame();
        //                if (buffer[5] == (byte)MavLink.MAVLINK_MSG_ID.PARAM_VALUE)
        //                {
        //                    var ans = buffer.ByteArrayToStructure<MavLink.mavlink_Param_Value>(6);
        //                    string paramID = System.Text.ASCIIEncoding.ASCII.GetString(ans.param_id);
        //                    int pos = paramID.IndexOf('\0');
        //                    if (pos != -1)
        //                    {
        //                        paramID = paramID.Substring(0, pos);
        //                    }
        //                    //如果字典中不包含参数名字
        //                    if (!mavStatus.paraDic.ContainsKey(paramID))
        //                    {
        //                        mavStatus.paraDic.Add(paramID, ans.param_value);
        //                    }
        //                    //如果字典中包含参数名字---可能一次发送2遍,以最后发来的为准
        //                    else
        //                    {
        //                        mavStatus.paraDic[paramID] = ans.param_value;
        //                    }
        //                    //列表中不包含 参数的索引
        //                    if (!paramIndexList.Contains(ans.param_index))
        //                    {
        //                        paramIndexList.Add(ans.param_index);
        //                    }
        //                }
        //                else if (now2.AddMilliseconds(600) < DateTime.Now)
        //                {
        //                    break;//超时之后 哪个没有发过来，然后自己一个一个请求
        //                }
        //                else//没有超时 但是发来的不是 22号帧
        //                {
        //                    continue;
        //                }
        //            }
        //        }
        //    }
        //    return mavStatus.paraDic;
        //}
        //byte[] buffer;
        //byte[] mavLinkFrame;
        //byte dataLength;
        //byte ck_a;
        //byte ck_b;
        //MAVLink.MavLink.MavLinkFrame_2 dataFrame_2;
        //List<byte> mavDataList = new List<byte>();
        //public byte[] ReadPacket()
        //{
        //    //不能用这个方法读取 
        //    //用一个while(true)循环 读取   
        //    while (true)
        //    {
        //        buffer = new byte[Global.serialPort.BytesToRead];
        //        Global.serialPort.Read(buffer, 0, Global.serialPort.BytesToRead);
        //        mavDataList.AddRange(buffer);
        //        while (mavDataList.Count > 2)
        //        {
        //            //第一个是254 且 长度够一个数据帧 (不一定是一个正确帧)
        //            if (mavDataList[0] == 254 && mavDataList.Count >= 8 + mavDataList[1])
        //            {
        //                dataLength = mavDataList[1];

        //                mavLinkFrame = new byte[8 + dataLength];
        //                mavLinkFrame = mavDataList.GetRange(0, 8 + dataLength).ToArray();
        //                dataFrame_2 = new MavLink.MavLinkFrame_2(mavLinkFrame);
        //                ushort checksum = MavlinkCRC.crc_calculate(Helper.StructToByteArray_2(dataFrame_2), 6 + dataLength);
        //                checksum = MavlinkCRC.crc_accumulate(MavlinkCRC.MAVLINK_MESSAGE_CRCS[mavLinkFrame[5]], checksum);
        //                ck_a = (byte)(checksum & 0xFF);
        //                ck_b = (byte)(checksum >> 8);
        //                //254是数据部分的254  不是头
        //                if (mavLinkFrame[mavLinkFrame.Length - 2] != ck_a || mavLinkFrame[mavLinkFrame.Length - 1] != ck_b)
        //                {
        //                    mavDataList.RemoveAt(0);
        //                    ReadPacket();
        //                }
        //                else//正确的帧
        //                {
        //                    //移除这个帧
        //                    mavDataList.RemoveRange(0, 8 + dataLength);
        //                    return mavLinkFrame;
        //                }
        //            }
        //            //第一个是254 但 长度不够一个数据帧
        //            else if (mavDataList[0] == 254 && mavDataList.Count < 8 + mavDataList[1])
        //            {
        //                ReadPacket();
        //            }
        //            //第一个不是 254
        //            else
        //            {
        //                //将第一个移除
        //                mavDataList.RemoveAt(0);
        //                ReadPacket();
        //            }
        //        }
        //        ReadPacket();
        //    }
        //}

    }

}
