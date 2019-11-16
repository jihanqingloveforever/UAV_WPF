using MAVLink;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace UAV_WPF
{
    public class MavStatus : MavLink
    {
        public MavLink.mavlink_Heart_Pack heart_Pack = new MAVLink.MavLink.mavlink_Heart_Pack();
        public MavLink.mavlink_SYS_Status sys_Status = new MAVLink.MavLink.mavlink_SYS_Status();
        public MavLink.mavlink_System_Time system_Time = new MAVLink.MavLink.mavlink_System_Time();
        public MavLink.mavlink_Attitude attitude = new MAVLink.MavLink.mavlink_Attitude();
        public MavLink.mavlink_Global_Position global_Position = new MAVLink.MavLink.mavlink_Global_Position();
        public MavLink.mavlink_GPS_Row gps_Row = new MAVLink.MavLink.mavlink_GPS_Row();
        public MavLink.mavlink_raw_imu_t raw_imu_t = new MAVLink.MavLink.mavlink_raw_imu_t();
        public MavLink.mavlink_raw_imu2_t raw_imu2_t = new MAVLink.MavLink.mavlink_raw_imu2_t();
        public MavLink.mavlink_scaled_pressure_t scaled_pressure_t = new MAVLink.MavLink.mavlink_scaled_pressure_t();
        public MavLink.mavlink_Nav_Controller_Output nav_Controller_Output = new MAVLink.MavLink.mavlink_Nav_Controller_Output();
        public MavLink.mavlink_Statustext status_Text = new MAVLink.MavLink.mavlink_Statustext();
        public MavLink.mavlink_Param_Value param_Value = new mavlink_Param_Value();
        public MavLink.mavlink_Vfr_Hud Vfr_Hud = new mavlink_Vfr_Hud();
        public MavLink.mavlink_Param_Set Param_Set = new mavlink_Param_Set();
        public MavLink.mavlink_sensor_offsets_t sensor_offset = new mavlink_sensor_offsets_t();
        /// <summary>
        /// 航点列表
        /// </summary>
        public Dictionary<int, mavlink_Mission_Item> missionList = new Dictionary<int, mavlink_Mission_Item>();
        /// <summary>
        /// 存储
        /// </summary>
        public mavlink_Mission_Item GuidedMode = new mavlink_Mission_Item();
        /// <summary>
        /// 存储参数的字典
        /// </summary>
        public Dictionary<string, double> paraDic = new Dictionary<string, double>();
        /// <summary>
        /// 参数值所对应的参数类型
        /// </summary>
        public Dictionary<string, MAV_PARAM_TYPE> param_Types = new Dictionary<string, MAV_PARAM_TYPE>();

    }
}
