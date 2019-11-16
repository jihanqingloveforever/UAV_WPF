using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace MAVLink
{
    public partial class MavLink
    {
        #region MavLink帧
        /// <summary>
        /// MavLink帧
        /// </summary>
        [StructLayoutAttribute(LayoutKind.Sequential, Pack = 1)]
        public struct MavLinkFrame
        {
            /// <summary>
            /// 固定值 254
            /// </summary>
            public byte STX;
            /// <summary>
            /// 数据包的长度
            /// </summary>
            public byte Len;
            /// <summary>
            /// 帧序号
            /// </summary>
            public byte SEQ;
            /// <summary>
            /// 系统编号
            /// </summary>
            public byte SYSID;
            /// <summary>
            /// 单元编号
            /// </summary>
            public byte CompID;
            /// <summary>
            /// 数据包的类型
            /// </summary>
            public byte MsgID;
            /// <summary>
            /// 数据包中的数据部分
            /// </summary>
            public byte[] Data;
            /// <summary>
            /// 校验码 低八位
            /// </summary>
            public byte Chk_A;
            /// <summary>
            /// 效验码 高八位
            /// </summary>
            public byte Chk_B;

            public MavLinkFrame(byte[] Frame)
            {
                this.STX = Frame[0];
                this.Len = Frame[1];
                this.SEQ = Frame[2];
                this.SYSID = Frame[3];
                this.CompID = Frame[4];
                this.MsgID = Frame[5];
                this.Data = new byte[Len];
                Data = (byte[])Frame.Skip(6).Take(Len).ToArray();
                this.Chk_A = Frame[Frame.Length - 2];
                this.Chk_B = Frame[Frame.Length - 1];
            }


        }
        #endregion

        #region 除了校验和的MavLink帧
        /// <summary>
        /// 除了校验和的MavLink帧
        /// </summary>
        [StructLayoutAttribute(LayoutKind.Sequential, Pack = 1)]
        public struct MavLinkFrame_2
        {
            /// <summary>
            /// 固定值 254
            /// </summary>
            public byte STX;
            /// <summary>
            /// 数据包的长度
            /// </summary>
            public byte Len;
            /// <summary>
            /// 帧序号
            /// </summary>
            public byte SEQ;
            /// <summary>
            /// 系统编号
            /// </summary>
            public byte SYSID;
            /// <summary>
            /// 单元编号
            /// </summary>
            public byte CompID;
            /// <summary>
            /// 数据包的类型
            /// </summary>
            public byte MsgID;
            /// <summary>
            /// 数据包中的数据部分
            /// </summary>
            public byte[] Data;


            public MavLinkFrame_2(byte[] Frame)
            {
                this.STX = Frame[0];
                this.Len = Frame[1];
                this.SEQ = Frame[2];
                this.SYSID = Frame[3];
                this.CompID = Frame[4];
                this.MsgID = Frame[5];
                this.Data = new byte[Len];
                Data = (byte[])Frame.Skip(6).Take(Len).ToArray();
            }
        }
        #endregion

        #region 枚举 不同消息包所对应的值
        /// <summary>
        /// 枚举 不同消息包所对应的值
        /// </summary>
        public enum MAVLINK_MSG_ID
        {
            /// <summary>
            /// 心跳包
            /// </summary>
            HEARTBEAT = 0,
            /// <summary>
            /// 系统状态
            /// </summary>
            SYS_STATUS = 1,
            SYSTEM_TIME = 2,
            PING = 4,
            CHANGE_OPERATOR_CONTROL = 5,
            CHANGE_OPERATOR_CONTROL_ACK = 6,
            AUTH_KEY = 7,
            SET_MODE = 11,
            /// <summary>
            /// 请求机载计算机发送指定参数
            /// </summary>
            PARAM_REQUEST_READ = 20,
            /// <summary>
            /// 请求发送所有的参数 
            /// </summary>
            PARAM_REQUEST_LIST = 21,
            PARAM_VALUE = 22,
            PARAM_SET = 23,
            GPS_RAW_INT = 24,
            GPS_STATUS = 25,
            SCALED_IMU = 26,
            RAW_IMU = 27,
            RAW_PRESSURE = 28,
            SCALED_PRESSURE = 29,
            ATTITUDE = 30,
            ATTITUDE_QUATERNION = 31,
            LOCAL_POSITION_NED = 32,
            GLOBAL_POSITION_INT = 33,
            RC_CHANNELS_SCALED = 34,
            RC_CHANNELS_RAW = 35,
            SERVO_OUTPUT_RAW = 36,
            /// <summary>
            /// 请求回送一部分任务清单 
            /// </summary>
            MISSION_REQUEST_PARTIAL_LIST = 37,
            MISSION_WRITE_PARTIAL_LIST = 38,
            /// <summary>
            /// 任务项
            /// </summary>
            MISSION_ITEM = 39,
            /// <summary>
            /// 任务下载请求
            /// </summary>
            MISSION_REQUEST = 40,
            MISSION_SET_CURRENT = 41,
            MISSION_CURRENT = 42,
            /// <summary>
            /// 任务单下载请求
            /// </summary>
            MISSION_REQUEST_LIST = 43,
            MISSION_COUNT = 44,
            MISSION_CLEAR_ALL = 45,
            MISSION_ITEM_REACHED = 46,
            /// <summary>
            /// 任务回应
            /// </summary>
            MISSION_ACK = 47,
            SET_GPS_GLOBAL_ORIGIN = 48,
            GPS_GLOBAL_ORIGIN = 49,
            PARAM_MAP_RC = 50,
            SAFETY_SET_ALLOWED_AREA = 54,
            SAFETY_ALLOWED_AREA = 55,
            ATTITUDE_QUATERNION_COV = 61,
            NAV_CONTROLLER_OUTPUT = 62,
            GLOBAL_POSITION_INT_COV = 63,
            LOCAL_POSITION_NED_COV = 64,
            RC_CHANNELS = 65,
            REQUEST_DATA_STREAM = 66,
            DATA_STREAM = 67,
            MANUAL_CONTROL = 69,
            RC_CHANNELS_OVERRIDE = 70,
            MISSION_ITEM_INT = 73,
            VFR_HUD = 74,
            COMMAND_INT = 75,
            /// <summary>
            /// 发送一个命令
            /// </summary>
            COMMAND_LONG = 76,
            /// <summary>
            /// 命令应答 
            /// </summary>
            COMMAND_ACK = 77,
            MANUAL_SETPOINT = 81,
            SET_ATTITUDE_TARGET = 82,
            ATTITUDE_TARGET = 83,
            SET_POSITION_TARGET_LOCAL_NED = 84,
            POSITION_TARGET_LOCAL_NED = 85,
            SET_POSITION_TARGET_GLOBAL_INT = 86,
            POSITION_TARGET_GLOBAL_INT = 87,
            LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET = 89,
            HIL_STATE = 90,
            HIL_CONTROLS = 91,
            HIL_RC_INPUTS_RAW = 92,
            OPTICAL_FLOW = 100,
            GLOBAL_VISION_POSITION_ESTIMATE = 101,
            VISION_POSITION_ESTIMATE = 102,
            VISION_SPEED_ESTIMATE = 103,
            VICON_POSITION_ESTIMATE = 104,
            HIGHRES_IMU = 105,
            OPTICAL_FLOW_RAD = 106,
            HIL_SENSOR = 107,
            SIM_STATE = 108,
            RADIO_STATUS = 109,
            FILE_TRANSFER_PROTOCOL = 110,
            TIMESYNC = 111,
            HIL_GPS = 113,
            HIL_OPTICAL_FLOW = 114,
            HIL_STATE_QUATERNION = 115,
            SCALED_IMU2 = 116,
            LOG_REQUEST_LIST = 117,
            LOG_ENTRY = 118,
            LOG_REQUEST_DATA = 119,
            LOG_DATA = 120,
            LOG_ERASE = 121,
            LOG_REQUEST_END = 122,
            GPS_INJECT_DATA = 123,
            GPS2_RAW = 124,
            POWER_STATUS = 125,
            SERIAL_CONTROL = 126,
            GPS_RTK = 127,
            GPS2_RTK = 128,
            SCALED_IMU3 = 129,
            DATA_TRANSMISSION_HANDSHAKE = 130,
            ENCAPSULATED_DATA = 131,
            DISTANCE_SENSOR = 132,
            TERRAIN_REQUEST = 133,
            TERRAIN_DATA = 134,
            TERRAIN_CHECK = 135,
            TERRAIN_REPORT = 136,
            SCALED_PRESSURE2 = 137,
            ATT_POS_MOCAP = 138,
            SET_ACTUATOR_CONTROL_TARGET = 139,
            ACTUATOR_CONTROL_TARGET = 140,
            SCALED_PRESSURE3 = 143,
            BATTERY_STATUS = 147,
            AUTOPILOT_VERSION = 148,
            LANDING_TARGET = 149,
            SENSOR_OFFSETS = 150,
            SET_MAG_OFFSETS = 151,
            MEMINFO = 152,
            AP_ADC = 153,
            DIGICAM_CONFIGURE = 154,
            DIGICAM_CONTROL = 155,
            MOUNT_CONFIGURE = 156,
            MOUNT_CONTROL = 157,
            MOUNT_STATUS = 158,
            FENCE_POINT = 160,
            FENCE_FETCH_POINT = 161,
            FENCE_STATUS = 162,
            AHRS = 163,
            SIMSTATE = 164,
            HWSTATUS = 165,
            RADIO = 166,
            LIMITS_STATUS = 167,
            WIND = 168,
            DATA16 = 169,
            DATA32 = 170,
            DATA64 = 171,
            DATA96 = 172,
            RANGEFINDER = 173,
            AIRSPEED_AUTOCAL = 174,
            RALLY_POINT = 175,

            RALLY_FETCH_POINT = 176,
            COMPASSMOT_STATUS = 177,
            AHRS2 = 178,
            CAMERA_STATUS = 179,
            CAMERA_FEEDBACK = 180,
            BATTERY2 = 181,
            AHRS3 = 182,
            AUTOPILOT_VERSION_REQUEST = 183,
            LED_CONTROL = 186,
            MAG_CAL_PROGRESS = 191,
            MAG_CAL_REPORT = 192,
            EKF_STATUS_REPORT = 193,
            PID_TUNING = 194,
            GIMBAL_REPORT = 200,
            GIMBAL_CONTROL = 201,
            GIMBAL_RESET = 202,
            GIMBAL_AXIS_CALIBRATION_PROGRESS = 203,
            GIMBAL_SET_HOME_OFFSETS = 204,
            GIMBAL_HOME_OFFSET_CALIBRATION_RESULT = 205,
            GIMBAL_SET_FACTORY_PARAMETERS = 206,
            GIMBAL_FACTORY_PARAMETERS_LOADED = 207,
            GIMBAL_ERASE_FIRMWARE_AND_CONFIG = 208,
            GIMBAL_PERFORM_FACTORY_TESTS = 209,
            GIMBAL_REPORT_FACTORY_TESTS_PROGRESS = 210,
            GIMBAL_REQUEST_AXIS_CALIBRATION_STATUS = 211,
            GIMBAL_REPORT_AXIS_CALIBRATION_STATUS = 212,
            GIMBAL_REQUEST_AXIS_CALIBRATION = 213,
            GOPRO_HEARTBEAT = 215,
            GOPRO_GET_REQUEST = 216,
            GOPRO_GET_RESPONSE = 217,
            GOPRO_SET_REQUEST = 218,
            GOPRO_SET_RESPONSE = 219,
            RPM = 226,
            VIBRATION = 241,
            V2_EXTENSION = 248,
            MEMORY_VECT = 249,
            DEBUG_VECT = 250,
            NAMED_VALUE_FLOAT = 251,
            NAMED_VALUE_INT = 252,
            STATUSTEXT = 253,
            DEBUG = 254,

        }
        #endregion

        #region 枚举 飞行器类型的
        /// <summary>
        /// 飞行器类型的枚举
        /// </summary>
        public enum MAV_TYPE
        {
            ///<summary> 通用微型飞行器</summary>
            GENERIC = 0,
            ///<summary> 固定翼飞机</summary>
            FIXED_WING = 1,
            ///<summary> 四轴</summary>
            QUADROTOR = 2,
            ///<summary>  共轴 </summary>
            COAXIAL = 3,
            ///<summary> 直机 </summary>
            HELICOPTER = 4,
            ///<summary> 地面跟踪天线 </summary>
            ANTENNA_TRACKER = 5,
            ///<summary> 地面站 </summary>
            GCS = 6,
            ///<summary> 有控飞艇 </summary>
            AIRSHIP = 7,
            ///<summary>自由飞气球 </summary>
            FREE_BALLOON = 8,
            ///<summary> 火箭  </summary>
            ROCKET = 9,
            ///<summary>地面车辆  </summary>
            GROUND_ROVER = 10,
            ///<summary> 水面船艇 </summary>
            SURFACE_BOAT = 11,
            ///<summary>潜艇 </summary>
            SUBMARINE = 12,
            ///<summary>六轴 </summary>
            HEXAROTOR = 13,
            ///<summary>八轴 </summary>
            OCTOROTOR = 14,
            ///<summary> 三轴  </summary>
            TRICOPTER = 15,
            ///<summary> 扑翼机</summary>
            FLAPPING_WING = 16,
            ///<summary> 风筝 </summary>
            KITE = 17,
            ///<summary> Onboard companion controller | </summary>
            ONBOARD_CONTROLLER = 18,
            ///<summary> Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter. | </summary>
            VTOL_DUOROTOR = 19,
            ///<summary> Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter. | </summary>
            VTOL_QUADROTOR = 20,
            ///<summary> VTOL reserved 1 | </summary>
            VTOL_RESERVED1 = 21,
            ///<summary> VTOL reserved 2 | </summary>
            VTOL_RESERVED2 = 22,
            ///<summary> VTOL reserved 3 | </summary>
            VTOL_RESERVED3 = 23,
            ///<summary> VTOL reserved 4 | </summary>
            VTOL_RESERVED4 = 24,
            ///<summary> VTOL reserved 5 | </summary>
            VTOL_RESERVED5 = 25,
            ///<summary> Onboard gimbal | </summary>
            GIMBAL = 26,
            ///<summary>  | </summary>
            ENUM_END = 27,

        };
        #endregion

        #region 枚举 飞控型号
        /// <summary>
        /// 枚举 飞控型号
        /// </summary>
        public enum MAV_AUTOPILOT
        {
            ///<summary>通用飞控，支持全部功能 </summary>
            GENERIC = 0,
            ///<summary> PIXHAWK autopilot</summary>
            PIXHAWK = 1,
            ///<summary> SLUGS autopilot</summary>
            SLUGS = 2,
            ///<summary> ArduPilotMega / ArduCopter</summary>
            ARDUPILOTMEGA = 3,
            ///<summary> OpenPilot</summary>
            OPENPILOT = 4,
            ///<summary> Generic autopilot only supporting simple waypoints | </summary>
            GENERIC_WAYPOINTS_ONLY = 5,
            ///<summary> Generic autopilot supporting waypoints and other simple navigation commands | </summary>
            GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY = 6,
            ///<summary> Generic autopilot supporting the full mission command set | </summary>
            GENERIC_MISSION_FULL = 7,
            ///<summary>无效飞控 </summary>
            INVALID = 8,
            ///<summary> PPZ UAV  </summary>
            PPZ = 9,
            ///<summary> UAV Dev Board | </summary>
            UDB = 10,
            ///<summary> FlexiPilot | </summary>
            FP = 11,
            ///<summary> PX4 Autopilot  </summary>
            PX4 = 12,
            ///<summary> SMACCMPilot </summary>
            SMACCMPILOT = 13,
            ///<summary> AutoQuad  </summary>
            AUTOQUAD = 14,
            ///<summary> Armazila </summary>
            ARMAZILA = 15,
            ///<summary> Aerob </summary>
            AEROB = 16,
            ///<summary> ASLUAV autopilot  </summary>
            ASLUAV = 17,
            ///<summary>  | </summary>
            ENUM_END = 18,

        };
        #endregion

        #region 枚举 系统当前模式
        /// <summary>
        /// 枚举 系统当前模式
        /// </summary>
        public enum MAV_MODE_FLAG
        {
            ///<summary> 0b00000001 留待扩展 </summary>
            CUSTOM_MODE_ENABLED = 1,
            ///<summary> 0b00000010 测试模式使能。本标识仅供临时的系统测试之用，不应该用于实际航行的应用中。</summary>
            TEST_ENABLED = 2,
            ///<summary> 0b00000100 全自主航行模式使能。系统自行决定目的地。前一项“导航使能”可以设置为 0 或 1 状态，这取决于具体的应用。 </summary>
            AUTO_ENABLED = 4,
            ///<summary> 0b00001000 导航使能。导航数据和指令来自导航/航点指令表文件。 </summary>
            GUIDED_ENABLED = 8,
            ///<summary> 0b00010000 高度/位置电子增稳使能。在此状态下，飞行器仍需</summary>
            STABILIZE_ENABLED = 16,
            ///<summary> 0b00100000 硬件环在线模拟使能。所有发动机、舵机及其他动作设备阻断，但内部软件处于全部可操作状态。</summary>
            HIL_ENABLED = 32,
            ///<summary> 0b01000000 遥控输入信号使能。 </summary>
            MANUAL_INPUT_ENABLED = 64,
            ///<summary> 0b10000000 主发动机使能。准备好起飞</summary>
            SAFETY_ARMED = 128,
            ///<summary>  | </summary>
            ENUM_END = 129,

        };
        #endregion

        #region 枚举 命令返回值
        /// <summary>
        /// 枚举 命令返回值
        /// </summary>
        public enum MAV_Result
        {
            ///<summary> 接受并执行</summary>
            ACCEPTED = 0,
            ///<summary> 暂时拒绝</summary>
            TEMPORARILY_REJECTED = 1,
            ///<summary> 永久拒绝</summary>
            DENIED = 2,
            ///<summary> 不支持命令</summary>
            UNSUPPORTED = 3,
            ///<summary> 执行失败</summary>
            FAILED = 4,
        };

        #endregion

        #region 枚举 指令
        /// <summary>
        /// 枚举 指令
        /// </summary>
        public enum MAV_CMD
        {
            ///<summary> 航点指令 值为特定值16 </summary>
            WAYPOINT = 16,
            ///<summary> 持续盘旋</summary>
            LOITER_UNLIM = 17,
            ///<summary> 在航点盘旋 N 圈 </summary>
            LOITER_TURNS = 18,
            ///<summary> 在航点盘旋N秒</summary>
            LOITER_TIME = 19,
            ///<summary> 返回起飞点</summary>
            RETURN_TO_LAUNCH = 20,
            ///<summary> 设置着陆点</summary>
            LAND = 21,
            ///<summary> 地面起飞或者手抛起飞</summary>
            TAKEOFF = 22,
            ///<summary> Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached. |Empty| Empty| Empty| Empty| Empty| Empty| Desired altitude in meters|  </summary>
            CONTINUE_AND_CHANGE_ALT = 30,
            ///<summary> Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.  Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached.  Additionally, if the Heading Required parameter is non-zero the  aircraft will not leave the loiter until heading toward the next waypoint.  |Heading Required (0 = False)| Radius in meters. If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter.| Empty| Empty| Latitude| Longitude| Altitude|  </summary>
            LOITER_TO_ALT = 31,
            ///<summary> 兴趣点（可用于操作相机在设定的高度拍照） </summary>
            ROI = 80,
            ///<summary> 避障或其他航路规划设定 </summary>
            PATHPLANNING = 81,
            ///<summary> Navigate to MISSION using a spline path. |Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)| Empty| Empty| Empty| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  </summary>
            SPLINE_WAYPOINT = 82,
            ///<summary> Mission command to wait for an altitude or downwards vertical speed. This is meant for high altitude balloon launches, allowing the aircraft to be idle until either an altitude is reached or a negative vertical speed is reached (indicating early balloon burst). The wiggle time is how often to wiggle the control surfaces to prevent them seizing up. |altitude (m)| descent speed (m/s)| Wiggle Time (s)| Empty| Empty| Empty| Empty|  </summary>
            ALTITUDE_WAIT = 83,
            ///<summary> hand control over to an external controller |On / Off (> 0.5f on)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            GUIDED_ENABLE = 92,
            ///<summary> 无实际内容，仅用于标记导航指令的数上限。</summary>
            LAST = 95,
            ///<summary> 延时 N 秒</summary>
            CONDITION_DELAY = 112,
            ///<summary> 前往设定高度，然后继续执行其他指令</summary>
            CONDITION_CHANGE_ALT = 113,
            ///<summary> 前往设定距离（到下一航点），然后继续。</summary>
            CONDITION_DISTANCE = 114,
            ///<summary> 前往设定的航向。</summary>
            CONDITION_YAW = 115,
            ///<summary> 无实际内容，仅用于标记状态指令的数上限。</summary>
            CONDITION_LAST = 159,
            ///<summary> 设定模式 </summary>
            DO_SET_MODE = 176,
            ///<summary> 跳转到任务单某个位置，并执行 N 次。相当于 While(i<n);循环。</summary>
            DO_JUMP = 177,
            ///<summary> 改变速度和/或油门 </summary>
            DO_CHANGE_SPEED = 178,
            ///<summary> 设定家位置（起飞点）到当前点或指定点。</summary>
            DO_SET_HOME = 179,
            ///<summary>  设定系统参数 注意：了解参数的确切含义，然后再修改. </summary>
            DO_SET_PARAMETER = 180,
            ///<summary> 设定继电器开关参数。</summary>
            DO_SET_RELAY = 181,
            ///<summary> 按照设定的次数和周期让继电器反复开关 </summary>
            DO_REPEAT_RELAY = 182,
            ///<summary> 设定舵机舵 </summary>
            DO_SET_SERVO = 183,
            ///<summary> 按照设定的位置、周期、次数，抖舵。</summary>
            DO_REPEAT_SERVO = 184,
            ///<summary> Terminate flight immediately |Flight termination activated if > 0.5| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            DO_FLIGHTTERMINATION = 185,
            ///<summary> Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. The Latitude/Longitude is optional, and may be set to 0/0 if not needed. If specified then it will be used to help find the closest landing sequence. |Empty| Empty| Empty| Empty| Latitude| Longitude| Empty|  </summary>
            DO_LAND_START = 189,
            ///<summary> Mission command to perform a landing from a rally point. |Break altitude (meters)| Landing speed (m/s)| Empty| Empty| Empty| Empty| Empty|  </summary>
            DO_RALLY_LAND = 190,
            ///<summary> Mission command to safely abort an autonmous landing. |Altitude (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            DO_GO_AROUND = 191,
            ///<summary> 设定机载摄机、照相机</summary>
            DO_CONTROL_VIDEO = 200,
            ///<summary> Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of intereset mode. (see MAV_ROI enum)| MISSION index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  </summary>
            DO_SET_ROI = 201,
            ///<summary> Mission command to configure an on-board camera controller system. |Modes: P, TV, AV, M, Etc| Shutter speed: Divisor number for one second| Aperture: F stop number| ISO number e.g. 80, 100, 200, Etc| Exposure type enumerator| Command Identity| Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)|  </summary>
            DO_DIGICAM_CONFIGURE = 202,
            ///<summary> Mission command to control an on-board camera controller system. |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Empty|  </summary>
            DO_DIGICAM_CONTROL = 203,
            ///<summary> Mission command to configure a camera or antenna mount |Mount operation mode (see MAV_MOUNT_MODE enum)| stabilize roll? (1 = yes, 0 = no)| stabilize pitch? (1 = yes, 0 = no)| stabilize yaw? (1 = yes, 0 = no)| Empty| Empty| Empty|  </summary>
            DO_MOUNT_CONFIGURE = 204,
            ///<summary> Mission command to control a camera or antenna mount |pitch or lat in degrees, depending on mount mode.| roll or lon in degrees depending on mount mode| yaw or alt (in meters) depending on mount mode| reserved| reserved| reserved| MAV_MOUNT_MODE enum value|  </summary>
            DO_MOUNT_CONTROL = 205,
            ///<summary> Mission command to set CAM_TRIGG_DIST for this flight |Camera trigger distance (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            DO_SET_CAM_TRIGG_DIST = 206,
            ///<summary> Mission command to enable the geofence |enable? (0=disable, 1=enable)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            DO_FENCE_ENABLE = 207,
            ///<summary> Mission command to trigger a parachute |action (0=disable, 1=enable, 2=release, for some systems see PARACHUTE_ACTION enum, not in general message set.)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            DO_PARACHUTE = 208,
            ///<summary> Mission command to perform motor test |motor sequence number (a number from 1 to max number of motors on the vehicle)| throttle type (0=throttle percentage, 1=PWM, 2=pilot throttle channel pass-through. See MOTOR_TEST_THROTTLE_TYPE enum)| throttle| timeout (in seconds)| Empty| Empty| Empty|  </summary>
            DO_MOTOR_TEST = 209,
            ///<summary> Change to/from inverted flight |inverted (0=normal, 1=inverted)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            DO_INVERTED_FLIGHT = 210,
            ///<summary> Mission command to operate EPM gripper |gripper number (a number from 1 to max number of grippers on the vehicle)| gripper action (0=release, 1=grab. See GRIPPER_ACTIONS enum)| Empty| Empty| Empty| Empty| Empty|  </summary>
            DO_GRIPPER = 211,
            ///<summary> Enable/disable autotune |enable (1: enable, 0:disable)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            DO_AUTOTUNE_ENABLE = 212,
            ///<summary> Mission command to control a camera or antenna mount, using a quaternion as reference. |q1 - quaternion param #1, w (1 in null-rotation)| q2 - quaternion param #2, x (0 in null-rotation)| q3 - quaternion param #3, y (0 in null-rotation)| q4 - quaternion param #4, z (0 in null-rotation)| Empty| Empty| Empty|  </summary>
            DO_MOUNT_CONTROL_QUAT = 220,
            ///<summary> set id of master controller |System ID| Component ID| Empty| Empty| Empty| Empty| Empty|  </summary>
            DO_GUIDED_MASTER = 221,
            ///<summary> set limits for external control |timeout - maximum time (in seconds) that external controller will be allowed to control vehicle. 0 means no timeout| absolute altitude min (in meters, AMSL) - if vehicle moves below this alt, the command will be aborted and the mission will continue.  0 means no lower altitude limit| absolute altitude max (in meters)- if vehicle moves above this alt, the command will be aborted and the mission will continue.  0 means no upper altitude limit| horizontal move limit (in meters, AMSL) - if vehicle moves more than this distance from it's location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal altitude limit| Empty| Empty| Empty|  </summary>
            DO_GUIDED_LIMITS = 222,
            ///<summary> 无实际内容，仅用于标记指令的数上限。 </summary>
            DO_LAST = 240,
            ///<summary> 航前设备校准， 仅在起飞前模式有效。</summary>
            PREFLIGHT_CALIBRATION = 241,
            ///<summary> 传感器偏移设定 </summary>
            PREFLIGHT_SET_SENSOR_OFFSETS = 242,
            ///<summary> 参数和任务存取指令</summary>
            PREFLIGHT_STORAGE = 245,
            ///<summary> 机载计算机和飞控的重启和关机。仅在起飞前模式有效。</summary>
            PREFLIGHT_REBOOT_SHUTDOWN = 246,
            ///<summary> 高优先级强制执行</summary>
            OVERRIDE_GOTO = 252,
            ///<summary> 任务单启动，从 n 到 m </summary>
            MISSION_START = 300,
            ///<summary> 机载组件启动和关闭 </summary>
            COMPONENT_ARM_DISARM = 400,
            ///<summary> Starts receiver pairing |0:Spektrum| 0:Spektrum DSM2, 1:Spektrum DSMX|  </summary>
            START_RX_PAIR = 500,
            ///<summary> Request autopilot capabilities |1: Request autopilot version| Reserved (all remaining params)|  </summary>
            REQUEST_AUTOPILOT_CAPABILITIES = 520,
            ///<summary> Start image capture sequence |Duration between two consecutive pictures (in seconds)| Number of images to capture total - 0 for unlimited capture| Resolution in megapixels (0.3 for 640x480, 1.3 for 1280x720, etc)|  </summary>
            IMAGE_START_CAPTURE = 2000,
            ///<summary> Stop image capture sequence |Reserved| Reserved|  </summary>
            IMAGE_STOP_CAPTURE = 2001,
            ///<summary> Starts video capture |Camera ID (0 for all cameras), 1 for first, 2 for second, etc.| Frames per second| Resolution in megapixels (0.3 for 640x480, 1.3 for 1280x720, etc)|  </summary>
            VIDEO_START_CAPTURE = 2500,
            ///<summary> Stop the current video capture |Reserved| Reserved|  </summary>
            VIDEO_STOP_CAPTURE = 2501,
            ///<summary> Create a panorama at the current position |Viewing angle horizontal of the panorama (in degrees, +- 0.5 the total angle)| Viewing angle vertical of panorama (in degrees)| Speed of the horizontal rotation (in degrees per second)| Speed of the vertical rotation (in degrees per second)|  </summary>
            PANORAMA_CREATE = 2800,
            ///<summary> Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity. |Operation mode. 0: prepare single payload deploy (overwriting previous requests), but do not execute it. 1: execute payload deploy immediately (rejecting further deploy commands during execution, but allowing abort). 2: add payload deploy to existing deployment list.| Desired approach vector in degrees compass heading (0..360). A negative value indicates the system can define the approach vector at will.| Desired ground speed at release time. This can be overriden by the airframe in case it needs to meet minimum airspeed. A negative value indicates the system can define the ground speed at will.| Minimum altitude clearance to the release position in meters. A negative value indicates the system can define the clearance at will.| Latitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT| Longitude unscaled for MISSION_ITEM or in 1e7 degrees for MISSION_ITEM_INT| Altitude, in meters AMSL|  </summary>
            PAYLOAD_PREPARE_DEPLOY = 30001,
            ///<summary> Control the payload deployment. |Operation mode. 0: Abort deployment, continue normal mission. 1: switch to payload deploment mode. 100: delete first payload deployment request. 101: delete all payload deployment requests.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  </summary>
            PAYLOAD_CONTROL_DEPLOY = 30002,
            ///<summary>  | </summary>
            ENUM_END = 30003,
            ///<summary> Initiate a magnetometer calibration |uint8_t bitmask of magnetometers (0 means all)| Automatically retry on failure (0=no retry, 1=retry).| Save without user input (0=require input, 1=autosave).| Delay (seconds)| Autoreboot (0=user reboot, 1=autoreboot)| Empty| Empty|  </summary>
            DO_START_MAG_CAL = 42424,
            ///<summary> Initiate a magnetometer calibration |uint8_t bitmask of magnetometers (0 means all)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            DO_ACCEPT_MAG_CAL = 42425,
            ///<summary> Cancel a running magnetometer calibration |uint8_t bitmask of magnetometers (0 means all)| Empty| Empty| Empty| Empty| Empty| Empty|  </summary>
            DO_CANCEL_MAG_CAL = 42426,

        };
        #endregion

        #region 枚举 系统状态
        /// <summary>
        /// 枚举 系统状态
        /// </summary>
        public enum MAV_STATE
        {
            ///<summary> 未初始化系统，状态未知</summary>
            UNINIT = 0,
            ///<summary> 正在启动 </summary>
            BOOT = 1,
            ///<summary> 正在校准，未准备好起飞</summary>
            CALIBRATING = 2,
            ///<summary> 系统地面待命，随时可以起飞 </summary>
            STANDBY = 3,
            ///<summary> 开车/开航。发动机已 启动</summary>
            ACTIVE = 4,
            ///<summary> 系统处于失常飞行状态，仍能导航</summary>
            CRITICAL = 5,
            ///<summary> 系统处于失常飞行状态，若干设备失灵，坠落状态</summary>
            EMERGENCY = 6,
            ///<summary> 系统刚执行了关机指令，正在关闭</summary>
            POWEROFF = 7,
            ///<summary>  | </summary>
            ENUM_END = 8,

        };
        #endregion

        #region 枚举 命令应答
        /// <summary>
        /// 枚举 命令应答 
        /// </summary>
        public enum MAV_CMD_ACK
        {
            ///<summary> 指令正常</summary>
            OK = 1,
            ///<summary> 一般性错误</summary>
            ERR_FAIL = 2,
            ///<summary> 拒绝执行</summary>
            ERR_ACCESS_DENIED = 3,
            ///<summary> 不支持的命令</summary>
            ERR_NOT_SUPPORTED = 4,
            ///<summary> 不支持此命令的坐标格式</summary>
            ERR_COORDINATE_FRAME_NOT_SUPPORTED = 5,
            ///<summary> 命令坐标格式正常，但坐标值超过了本系统的安全设定</summary>
            ERR_COORDINATES_OUT_OF_RANGE = 6,
            ///<summary> 维度超出范围</summary>
            ERR_X_LAT_OUT_OF_RANGE = 7,
            ///<summary> 经度超出范围</summary>
            ERR_Y_LON_OUT_OF_RANGE = 8,
            ///<summary> 高度超出范围</summary>
            ERR_Z_ALT_OUT_OF_RANGE = 9,
        };
        #endregion

        #region 枚举 任务返回值
        /// <summary>
        /// 任务返回值 
        /// </summary>
        public enum MAV_MISSION_RESULT
        {
            ///<summary> 接受</summary>
            MAV_MISSION_ACCEPTED = 0,
            ///<summary> 一般错误</summary>
            MAV_MISSION_ERROR = 1,
            ///<summary> 不支持</summary>
            MAV_MISSION_UNSUPPORTED_FRAME = 2,
            ///<summary> 指令不支持</summary>
            MAV_MISSION_UNSUPPORTED = 3,
            ///<summary> 任务超出存储空间</summary>
            MAV_MISSION_NO_SPACE = 4,
            ///<summary> value 参数数据非法</summary>
            MAV_MISSION_INVALID = 5,
            ///<summary> 参数1非法</summary>
            MAV_MISSION_INVALID_PARAM1 = 6,
            ///<summary> 参数2非法</summary>
            MAV_MISSION_INVALID_PARAM2 = 7,
            ///<summary> 参数3非法</summary>
            MAV_MISSION_INVALID_PARAM3 = 8,
            ///<summary> 参数4非法</summary>
            MAV_MISSION_INVALID_PARAM4 = 9,
            ///<summary> 参数5非法</summary>
            MAV_MISSION_INVALID_PARAM5_X = 10,
            ///<summary> 参数6非法</summary>
            MAV_MISSION_INVALID_PARAM6_Y = 11,
            ///<summary> 参数7非法</summary>
            MAV_MISSION_INVALID_PARAM7 = 12,
            ///<summary> 航点越界</summary>
            MAV_MISSION_INVALID_SEQUENCE = 13,
            ///<summary> 拒绝接受此传送者</summary>
            MAV_MISSION_DENIED = 14,
        };
        #endregion

        #region 枚举 消息帧坐标格式
        /// <summary>
        /// 枚举 消息帧坐标格式
        /// </summary>
        public enum MAV_FRAME
        {
            ///<summary> WGS84坐标系</summary>
            GLOBAL = 0,
            ///<summary> 局部坐标系</summary>
            LOCAL_NED = 1,
            ///<summary> 非坐标帧,标识任务指令</summary>
            MISSION = 2,
            ///<summary> 相对高度。WGS84 坐标系，高度数据为相对起飞点的高度。纬高格式。</summary>
            GLOBAL_RELATIVE_ALT = 3,
            ///<summary> 局部坐标系。 1，X 向东，2，Y 向北，3，Z 向上。 </summary>
            LOCAL_ENU = 4,
            ///<summary> Global coordinate frame, WGS84 coordinate system. First value / x: latitude in degrees*1.0e-7, second value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL) | </summary>
            GLOBAL_INT = 5,
            ///<summary> Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude with 0 being at the altitude of the home location. | </summary>
            GLOBAL_RELATIVE_ALT_INT = 6,
            ///<summary> Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position. | </summary>
            LOCAL_OFFSET_NED = 7,
            ///<summary> Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right. | </summary>
            BODY_NED = 8,
            ///<summary> Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an obstacle - e.g. useful to command 2 m/s^2 acceleration to the east. | </summary>
            BODY_OFFSET_NED = 9,
            ///<summary> Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model. | </summary>
            GLOBAL_TERRAIN_ALT = 10,
            ///<summary> Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground level in terrain model. | </summary>
            GLOBAL_TERRAIN_ALT_INT = 11,
            ///<summary>  | </summary>
            ENUM_END = 12,

        };
        #endregion

        #region 枚举 参数类型
        /// <summary>
        /// 枚举 参数类型
        /// </summary>
        public enum MAV_PARAM_TYPE
        {
            ///<summary>8位无符号整数 | </summary>
            UINT8 = 1,
            ///<summary> 8位有符号整数 | </summary>
            INT8 = 2,
            ///<summary> 16-bit unsigned integer | </summary>
            UINT16 = 3,
            ///<summary> 16-bit signed integer | </summary>
            INT16 = 4,
            ///<summary> 32-bit unsigned integer | </summary>
            UINT32 = 5,
            ///<summary> 32-bit signed integer | </summary>
            INT32 = 6,
            ///<summary> 64-bit unsigned integer | </summary>
            UINT64 = 7,
            ///<summary> 64-bit signed integer | </summary>
            INT64 = 8,
            ///<summary> 32-bit floating-point | </summary>
            REAL32 = 9,
            ///<summary> 64-bit floating-point | </summary>
            REAL64 = 10,
            ///<summary>  | </summary>
            ENUM_END = 11,

        }; 
        #endregion

        #region 00 心跳包
        /// <summary>
        /// 心跳包
        /// </summary>
        public struct mavlink_Heart_Pack
        {
            /// <summary>自定义模式 </summary>
            public UInt32 custom_mode;
            /// <summary>飞行棋类型</summary>
            public byte type;
            /// <summary> 飞控型号</summary>
            public byte autopilot;
            /// <summary> 系统模式</summary>
            public byte base_mode;
            /// <summary>系统状态</summary>
            public byte system_status;
            /// <summary>协议版本</summary>
            public byte mavlink_version;

        };
        #endregion

        #region 01 系统状态
        /// <summary>
        /// 系统状态
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 31)]
        public struct mavlink_SYS_Status
        {
            /// <summary> 机载控制传感器存在</summary>
            public UInt32 onboard_control_sensors_present;
            /// <summary> 传感器的开启状态</summary>
            public UInt32 onboard_control_sensors_enabled;
            /// <summary> 传感器的健康状态</summary>
            public UInt32 onboard_control_sensors_health;
            /// <summary> 主循环负载，机载计算机负载</summary>
            public UInt16 load;
            /// <summary> 电池电压</summary>
            public UInt16 voltage_battery;
            /// <summary> 工作电流</summary>
            public Int16 current_battery;
            /// <summary> 电池余量</summary>
            public byte battery_remaining;
            /// <summary> 丢包率</summary>
            public UInt16 drop_rate_comm;
            /// <summary> 丢包总计 </summary>
            public UInt16 errors_comm;
            /// <summary> 导航仪误差 1</summary>
            public UInt16 errors_count1;
            /// <summary> 导航仪误差 2</summary>
            public UInt16 errors_count2;
            /// <summary> 导航仪误差 3</summary>
            public UInt16 errors_count3;
            /// <summary> 导航仪误差 4</summary>
            public UInt16 errors_count4;

        };
        #endregion

        #region 02 系统时间
        /// <summary>
        /// 系统时间
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 12)]
        public struct mavlink_System_Time
        {
            /// <summary>
            /// UNIX 时间戳 从 1970 年 1 月 1 日（UTC/GMT 的午夜）开始所 过的秒数
            /// </summary>
            public UInt64 time_unix_usec;
            /// <summary>
            /// 系统启动开始的秒数
            /// </summary>
            public UInt32 time_boot_ms;
        }
        #endregion

        #region 11 模式设置
        /// <summary>
        /// 模式设置
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 6)]
        public struct mavlink_Set_Mode
        {
            /// <summary> 目标系统</summary>
            public UInt32 custom_mode;
            /// <summary> 新的基本模式</summary>
            public byte target_system;
            /// <summary> 用户自定义模式</summary>
            public byte base_mode;
        };
        #endregion

        #region 20 请求发送指定机载参数
        /// <summary>
        /// 20 请求发送指定机载参数
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 20)]
        public struct mavlink_Param_Request_Read
        {
            /// <summary> 参数索引 就是参数名称</summary>
            public Int16 param_index;
            /// <summary> 系统ID</summary>
            public byte target_system;
            /// <summary> 组件ID</summary>
            public byte target_component;
            /// <summary> 参数名称</summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
            public byte[] param_id;

        }; 
        #endregion

        #region 21 请求发送所有的参数
        /// <summary>
        /// 请求发送所有的参数 
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 2)]
        public struct mavlink_Param_Request_List
        {
            /// <summary>系统ID</summary>
            public byte target_system;
            /// <summary>组件ID</summary>
            public byte target_component;
        };
        #endregion

        #region 22 参数数值
        /// <summary>
        /// 参数数值
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 25)]
        public struct mavlink_Param_Value
        {
            /// <summary>参数值</summary>
            public Single param_value;
            /// <summary>参数</summary>
            public UInt16 param_count;
            /// <summary>本参数索引</summary>
            public UInt16 param_index;
            /// <summary>机载参数身份号</summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
            public byte[] param_id;
            /// <summary>参数类型</summary>
            public byte param_type;

        };
        #endregion

        #region 23 参数设置
        /// <summary>
        /// 23 参数设置
        /// </summary>
        /// 发送消息给非C#程序时，不能使用序列化和反序列化，因此就必须把消息转为内存格式，然后再发送出去，这时就涉及到消息在内存中的对齐方式了。
        /// 通常我们发送消息都是用struct，在想要改变对齐方式的struct之前加上下述语句即可
        /// [StructLayout(LayoutKind.Sequential, Pack=4)]
        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 23)]
        public struct mavlink_Param_Set
        {
            /// <summary> 参数值</summary>
            public Single param_value;
            /// <summary> 系统ID</summary>
            public byte target_system;
            /// <summary> 组件ID</summary>
            public byte target_component;
            /// <summary> 机载参数身份号</summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
            public byte[] param_id;
            /// <summary> 参数类型</summary>
            public byte param_type;

        };
        #endregion

        #region 24 GPS信息
        /// <summary>
        /// GPS信息
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 30)]
        public struct mavlink_GPS_Row
        {
            /// <summary> UNIX 格式时间戳</summary>
            public UInt64 time_usec;
            /// <summary> 0 或 1：尚未定位，2：2D 定位，3：3D 定位。</summary>
            public byte fix_type;
            /// <summary> 纬度 </summary>
            public Int32 lat;
            /// <summary> 经度</summary>
            public Int32 lng;
            /// <summary> 高度</summary>
            public Int32 alt;
            /// <summary> 水平定位精度。 如果不知道，就设置为 65535.</summary>
            public UInt16 eph;
            /// <summary> 垂直定位精度。如果不知道，就设置为 65535. </summary>
            public UInt16 epv;
            /// <summary> 地速。如果不知道，就设置为 65535.</summary>
            public UInt16 vel;
            /// <summary> 方向。注意：这不是机头指向，而是整体移动方向。如果不知道，就设置为 65535。</summary>
            public UInt16 cog;

            /// <summary> 可见卫星数。如果不知道，就设置为 255.</summary>
            public byte satellites_visible;

        };
        #endregion

        #region 27 原始的姿态传感器信息  
        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 26)]
        public struct mavlink_raw_imu_t
        {
            /// <summary> Timestamp (microseconds since UNIX epoch or microseconds since system boot) 时间戳（自UNIX系统启动以来的微妙）</summary>
            public UInt64 time_usec;
            /// <summary> X acceleration (raw) X轴的加速度</summary>
            public Int16 xacc;
            /// <summary> Y acceleration (raw) </summary>
            public Int16 yacc;
            /// <summary> Z acceleration (raw) </summary>
            public Int16 zacc;
            /// <summary> Angular speed around X axis (raw) 绕X轴的加速度</summary>
            public Int16 xgyro;
            /// <summary> Angular speed around Y axis (raw) </summary>
            public Int16 ygyro;
            /// <summary> Angular speed around Z axis (raw) </summary>
            public Int16 zgyro;
            /// <summary> X Magnetic field (raw) X轴的磁场强度</summary>
            public Int16 xmag;
            /// <summary> Y Magnetic field (raw) </summary>
            public Int16 ymag;
            /// <summary> Z Magnetic field (raw) </summary>
            public Int16 zmag;

        };
        #endregion

        #region  29  处理之后的气压数据  //123
        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 14)]
        public struct mavlink_scaled_pressure_t
        {
            /// <summary> Timestamp (milliseconds since system boot) 时间戳</summary>
            public UInt32 time_boot_ms;
            /// <summary> Absolute pressure (hectopascal) 绝对压力（百帕）</summary>
            public Single press_abs;
            /// <summary> Differential pressure 1 (hectopascal) 相对压力（百帕）</summary>
            public Single press_diff;
            /// <summary> Temperature measurement (0.01 degrees celsius) 温度测量（0.01摄氏度）</summary>
            public Int16 temperature;

        };
        #endregion

        #region 30 姿态信息
        /// <summary>
        /// 姿态信息
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 28)]
        public struct mavlink_Attitude
        {
            /// <summary> 时间戳</summary>
            public UInt32 time_boot_ms;
            /// <summary> 滚动角</summary>
            public Single roll;
            /// <summary> 俯仰角</summary>
            public Single pitch;
            /// <summary> 偏航角</summary>
            public Single yaw;
            /// <summary> 滚转角速度 </summary>
            public Single rollspeed;
            /// <summary> 俯仰角速度 </summary>
            public Single pitchspeed;
            /// <summary> 偏航角速度</summary>
            public Single yawspeed;

        };

        #endregion

        #region 33 位置信息
        /// <summary>
        /// 位置信息
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 28)]
        public struct mavlink_Global_Position
        {
            /// <summary> 时间戳</summary>
            public UInt32 time_boot_ms;
            /// <summary> 纬度</summary>
            public Int32 lat;
            /// <summary> 经度</summary>
            public Int32 lon;
            /// <summary> 海拔高度</summary>
            public Int32 alt;
            /// <summary> 相对地面高度</summary>
            public Int32 relative_alt;
            /// <summary> X 地速 （1234 表示 12.34 米/秒）</summary>
            public Int16 vx;
            /// <summary> Y 地速 （1234 表示 12.34 米/秒） </summary>
            public Int16 vy;
            /// <summary> Z 地速 （1234 表示 12.34 米/秒） </summary>
            public Int16 vz;
            /// <summary> 罗盘   （1234 表示 12.34, 0~359.99） </summary>
            public UInt16 hdg;
        };
        #endregion

        #region 39 传送具体的任务项
        /// <summary>
        /// 39 传送具体的任务项
        /// </summary>
         [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 37)]
        public struct mavlink_Mission_Item
        {
            /// <summary> 参数1</summary>
            public Single param1;
            /// <summary> 参数2</summary>
            public Single param2;
            /// <summary> 参数3</summary>
            public Single param3;
            /// <summary> 参数4</summary>
            public Single param4;
            /// <summary> 维度</summary>
            public Single x;
            /// <summary> 经度</summary>
            public Single y;
            /// <summary> 高度</summary>
            public Single z;
            /// <summary> seq序列号</summary>
            public UInt16 seq;
            /// <summary> 预定行为的任务</summary>
            public UInt16 command;
            /// <summary> 系统ID</summary>
            public byte target_system;
            /// <summary> 组件ID</summary>
            public byte target_component;
            /// <summary> 坐标系的任务</summary>
            public byte frame;
            /// <summary> false:0, true:1 </summary>
            public byte current;
            /// <summary> 自动执行下一航点指令</summary>
            public byte autocontinue;

        };
        #endregion

        #region 40 按指定的序号下载任务
        /// <summary>
        /// 40 按指定的序号下载任务
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 4)]
        public struct mavlink_mission_request
        {
            /// <summary> 序号</summary>
            public UInt16 seq;
            /// <summary> 系统ID</summary>
            public byte target_system;
            /// <summary> 组件ID</summary>
            public byte target_component;

        };
        #endregion

        #region 41 设置当前任务
        /// <summary>
        /// 41 设置当前任务
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 4)]
        public struct mavlink_Mission_Set_Current
        {
            /// <summary> seq序号</summary>
            public UInt16 seq;
            /// <summary> 系统ID</summary>
            public byte target_system;
            /// <summary> 组件ID</summary>
            public byte target_component;

        };
        #endregion

        #region 43 任务单下载请求
        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 2)]
        public struct mavlink_Mission_Request_List
        {
            /// <summary>系统编号</summary>
            public byte target_system;
            /// <summary>单元编号</summary>
            public byte target_component;
        }
        #endregion

        #region 44 任务计数
        /// <summary>
        /// 44 任务计数
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 4)]
        public struct mavlink_Mission_Count
        {
            /// <summary> 任务数目</summary>
            public UInt16 count;
            /// <summary> 系统ID</summary>
            public byte target_system;
            /// <summary> 组件ID</summary>
            public byte target_component;
        };
        #endregion

        #region 47 任务回应
        /// <summary>
        /// 47 任务回应
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 3)]
        public struct mavlink_Mission_Ack
        {
            /// <summary> 系统ID</summary>
            public byte target_system;
            /// <summary> 组件ID </summary>
            public byte target_component;
            /// <summary> 枚举 任务返回值类型</summary>
            public byte type;
        };
        #endregion

        #region 62 导航控制器输出
        /// <summary>
        /// 导航控制器输出
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 26)]
        public struct mavlink_Nav_Controller_Output
        {
            /// <summary> 目标滚转角</summary>
            public Single nav_roll;
            /// <summary> 目标俯仰角 </summary>
            public Single nav_pitch;
            /// <summary> 高度差</summary>
            public Single alt_error;
            /// <summary> 速度差 </summary>
            public Single aspd_error;
            /// <summary> 水平位置差</summary>
            public Single xtrack_error;
            /// <summary> 目标指向角 </summary>
            public Int16 nav_bearing;
            /// <summary> 目标方位</summary>
            public Int16 target_bearing;
            /// <summary> 到下一任务点的距离</summary>
            public UInt16 wp_dist;

        };

        #endregion

        #region  66 请求数据流  //123
        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 6)]
        public struct mavlink_request_data_stream_t
        {
            /// <summary> The requested message rate 请求消息的速度</summary>
            public UInt16 req_message_rate;
            /// <summary> The target requested to send the message stream. 请求发送的对象</summary>
            public byte target_system;
            /// <summary> The target requested to send the message stream. 请求发送的对象</summary>
            public byte target_component;
            /// <summary> The ID of the requested data stream 请求数据流的ID</summary>
            public byte req_stream_id;
            /// <summary> 1 to start sending, 0 to stop sending. 1开始，0结束 </summary>
            public byte start_stop;

        };
        #endregion 

        #region 74 HUD显示信息
        /// <summary>
        /// 74 HUD显示信息
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 20)]
        public struct mavlink_Vfr_Hud
        {
            /// <summary> 空速</summary>
            public Single airspeed;
            /// <summary> 地速</summary>
            public Single groundspeed;
            /// <summary> 高度</summary>
            public Single alt;
            /// <summary> 爬升率</summary>
            public Single climb;
            /// <summary> 航向</summary>
            public Int16 heading;
            /// <summary> 油门</summary>
            public UInt16 throttle;

        };
        #endregion

        #region 76 命令
        /// <summary>
        /// 76 命令
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 33)]
        public struct mavlink_Command_Long
        {
            public Single param1;
            public Single param2;
            public Single param3;
            public Single param4;
            public Single param5;
            public Single param6;
            public Single param7;
            /// <summary> 命令</summary>
            public UInt16 command;
            /// <summary> 目标系统</summary>
            public byte target_system;
            /// <summary> 目标组件</summary>
            public byte target_component;
            /// <summary> 确认次数</summary>
            public byte confirmation;

        };
        #endregion

        #region 77 命令应答
        /// <summary>
        /// 77 命令应答
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 3)]
        public struct mavlink_Command_Ack
        {
            /// <summary> 命令ID</summary>
            public UInt16 command;
            /// <summary> 命令结果</summary>
            public byte result;
        };
        #endregion

        #region 116 原始的姿态传感器信息
        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 26)]
        public struct mavlink_raw_imu2_t
        {
            /// <summary> Timestamp (microseconds since UNIX epoch or microseconds since system boot) 时间戳（自UNIX系统启动以来的微妙）</summary>
            public UInt32 time_usec;
            /// <summary> X acceleration (raw) X轴的加速度</summary>
            public Int16 xacc;
            /// <summary> Y acceleration (raw) </summary>
            public Int16 yacc;
            /// <summary> Z acceleration (raw) </summary>
            public Int16 zacc;
            /// <summary> Angular speed around X axis (raw) 绕X轴的加速度</summary>
            public Int16 xgyro;
            /// <summary> Angular speed around Y axis (raw) </summary>
            public Int16 ygyro;
            /// <summary> Angular speed around Z axis (raw) </summary>
            public Int16 zgyro;
            /// <summary> X Magnetic field (raw) X轴的磁场强度</summary>
            public Int16 xmag;
            /// <summary> Y Magnetic field (raw) </summary>
            public Int16 ymag;
            /// <summary> Z Magnetic field (raw) </summary>
            public Int16 zmag;

        };
        #endregion

        #region 147 电池状态
        /// <summary>
        /// 147 电池状态
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 36)]
        public struct mavlink_Battery_Status
        {
            /// <summary> 电池电流 1表示10毫安 -1表示无相关设备</summary>
            public Int32 current_consumed;
            /// <summary> 消耗能量</summary>
            public Int32 energy_consumed;
            /// <summary> 温度</summary>
            public Int16 temperature;
            /// <summary> 电池电压 单位毫伏</summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 10)]
            public UInt16[] voltages;
            /// <summary> 当前电流</summary>
            public Int16 current_battery;
            /// <summary> 电池ID</summary>
            public byte id;
            /// <summary> 电池的功能</summary>
            public byte battery_function;
            /// <summary> 电池的化学类型</summary>
            public byte type;
            /// <summary> 电池的剩余量</summary>
            public byte battery_remaining;

        };
        #endregion

        #region 150传感器偏移量

        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 42)]
        public struct mavlink_sensor_offsets_t
        {
            /// <summary> magnetic declination (radians)磁偏角（弧度） </summary>
            public Single mag_declination;
            /// <summary> raw pressure from barometer 气压计</summary>
            public Int32 raw_press;
            /// <summary> raw temperature from barometer温度 </summary>
            public Int32 raw_temp;
            /// <summary> gyro X calibration 陀螺仪校准</summary>
            public Single gyro_cal_x;
            /// <summary> gyro Y calibration </summary>
            public Single gyro_cal_y;
            /// <summary> gyro Z calibration </summary>
            public Single gyro_cal_z;
            /// <summary> accel X calibration </summary>
            public Single accel_cal_x;
            /// <summary> accel Y calibration </summary>
            public Single accel_cal_y;
            /// <summary> accel Z calibration </summary>
            public Single accel_cal_z;
            /// <summary> magnetometer X offset </summary>
            public Int16 mag_ofs_x;
            /// <summary> magnetometer Y offset </summary>
            public Int16 mag_ofs_y;
            /// <summary> magnetometer Z offset </summary>
            public Int16 mag_ofs_z;

        }; 
        #endregion

        #region 253 文本信息
        /// <summary>
        /// 文本信息
        /// </summary>
        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 51)]
        public struct mavlink_Statustext
        {
            /// <summary> 消息级别</summary>
            public byte severity;
            /// <summary> 消息内容</summary>
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 50)]
            public byte[] text;

        };
        #endregion

        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 33)]
        public struct mavlink_command_long_t
        {
            /// <summary> Parameter 1, as defined by MAV_CMD enum. </summary>
            public Single param1;
            /// <summary> Parameter 2, as defined by MAV_CMD enum. </summary>
            public Single param2;
            /// <summary> Parameter 3, as defined by MAV_CMD enum. </summary>
            public Single param3;
            /// <summary> Parameter 4, as defined by MAV_CMD enum. </summary>
            public Single param4;
            /// <summary> Parameter 5, as defined by MAV_CMD enum. </summary>
            public Single param5;
            /// <summary> Parameter 6, as defined by MAV_CMD enum. </summary>
            public Single param6;
            /// <summary> Parameter 7, as defined by MAV_CMD enum. </summary>
            public Single param7;
            /// <summary> Command ID, as defined by MAV_CMD enum. </summary>
            public UInt16 command;
            /// <summary> System which should execute the command </summary>
            public byte target_system;
            /// <summary> Component which should execute the command, 0 for all components </summary>
            public byte target_component;
            /// <summary> 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command) </summary>
            public byte confirmation;

        };


    }



}
