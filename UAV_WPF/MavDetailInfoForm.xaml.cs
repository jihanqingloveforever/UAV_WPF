using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;

namespace UAV_WPF
{
    /// <summary>
    /// MavDetailInfoForm.xaml 的交互逻辑
    /// </summary>
    public partial class MavDetailInfoForm : Window
    {
        Timer timer;

        public MavDetailInfoForm()
        {
            InitializeComponent();
            //ThreadStart threadStart = SetData;
            //Thread thread = new Thread(threadStart);
            //thread.TrySetApartmentState(ApartmentState.STA);
            //thread.IsBackground = true;
            //thread.Start();
            timer = new Timer(new TimerCallback(timer_Elapsed),this,0,1000);
        }
        private void timer_Elapsed(object state)
        {
            this.Dispatcher.BeginInvoke(new Action(() => SetData()),null);
        }
        private void SetData()
        {
            this.txtType.Text = Global.mavStatus.heart_Pack.type.ToString();
            this.rollspeed.Text = Global.mavStatus.attitude.rollspeed.ToString();
            this.pitchspeed.Text = Global.mavStatus.attitude.pitchspeed.ToString();
            this.yawspeed.Text = Global.mavStatus.attitude.yawspeed.ToString();
            this.load.Text = Global.mavStatus.sys_Status.load.ToString();
            this.nav_pitch.Text = Global.mavStatus.nav_Controller_Output.nav_pitch.ToString();
            this.nav_bearing.Text = Global.mavStatus.nav_Controller_Output.nav_bearing.ToString();
            this.xtrack_error.Text = Global.mavStatus.nav_Controller_Output.xtrack_error.ToString();
            this.nav_roll.Text = Global.mavStatus.nav_Controller_Output.nav_roll.ToString();
            this.errors_count1.Text = Global.mavStatus.sys_Status.errors_count1.ToString();
            this.errors_count2.Text = Global.mavStatus.sys_Status.errors_count2.ToString();
            this.errors_count3.Text = Global.mavStatus.sys_Status.errors_count3.ToString();
            this.errors_count4.Text = Global.mavStatus.sys_Status.errors_count4.ToString();
            this.relative_alt.Text = (Global.mavStatus.global_Position.relative_alt / 1000).ToString();
            this.fix_type.Text = Global.mavStatus.gps_Row.fix_type.ToString();
            this.errors_comm.Text = Global.mavStatus.sys_Status.errors_comm.ToString();
            this.vx.Text = Global.mavStatus.global_Position.vx.ToString();
            this.vy.Text = Global.mavStatus.global_Position.vy.ToString();
            this.vz.Text = Global.mavStatus.global_Position.vz.ToString();
            this.drop_rate_comm.Text = Global.mavStatus.sys_Status.drop_rate_comm.ToString();
            this.roll.Text = Global.mavStatus.attitude.roll.ToString();
            this.pitch.Text = Global.mavStatus.attitude.pitch.ToString();
            this.yaw.Text = Global.mavStatus.attitude.yaw.ToString();
            this.time_boot_ms.Text = Global.mavStatus.global_Position.time_boot_ms.ToString();
            this.time_usec.Text = Global.mavStatus.gps_Row.time_usec.ToString();
            this.txtVersion.Text = Global.mavStatus.heart_Pack.mavlink_version.ToString();
            this.alt_error.Text = Global.mavStatus.nav_Controller_Output.alt_error.ToString();
            this.aspd_error.Text = Global.mavStatus.nav_Controller_Output.aspd_error.ToString();
            this.lat.Text = (Global.mavStatus.gps_Row.lat / Math.Pow(10, 7)).ToString();
            this.lng.Text = (Global.mavStatus.gps_Row.lng / Math.Pow(10, 7)).ToString();
            this.alt.Text = Global.mavStatus.gps_Row.alt.ToString();
            this.eph.Text = Global.mavStatus.gps_Row.eph.ToString();
            this.epv.Text = Global.mavStatus.gps_Row.epv.ToString();
            this.vel.Text = Global.mavStatus.gps_Row.vel.ToString();
            this.cog.Text = Global.mavStatus.gps_Row.cog.ToString();
            this.lat1.Text = Global.mavStatus.global_Position.lat.ToString();
            this.lon.Text = Global.mavStatus.global_Position.lon.ToString();
            this.hdg.Text = Global.mavStatus.global_Position.hdg.ToString();
            this.time_boot_ms1.Text = Global.mavStatus.attitude.time_boot_ms.ToString();
            this.time_unix_usec.Text = Global.mavStatus.system_Time.time_unix_usec.ToString();
            this.time_boot_ms11.Text = Global.mavStatus.system_Time.time_boot_ms.ToString();
            this.wp_dist.Text = Global.mavStatus.nav_Controller_Output.wp_dist.ToString();
            this.txtAutopilot.Text = Global.mavStatus.heart_Pack.autopilot.ToString();
            this.txtUserModel.Text = Global.mavStatus.heart_Pack.custom_mode.ToString();
            this.txtSystemState.Text = Global.mavStatus.heart_Pack.system_status.ToString();
            this.alt1.Text = Global.mavStatus.global_Position.alt.ToString();
            this.voltage_battery.Text = Global.mavStatus.sys_Status.voltage_battery.ToString();
            this.current_battery.Text = Global.mavStatus.sys_Status.current_battery.ToString();
            this.battery_remaining.Text = Global.mavStatus.sys_Status.battery_remaining.ToString();
            this.target_bearing.Text = Global.mavStatus.nav_Controller_Output.target_bearing.ToString();
            this.satellites_visible.Text = Global.mavStatus.gps_Row.satellites_visible.ToString();
            this.onboard_control_sensors_enabled.Text = Global.mavStatus.sys_Status.onboard_control_sensors_enabled.ToString();
            this.onboard_control_sensors_health.Text = Global.mavStatus.sys_Status.onboard_control_sensors_health.ToString();
            this.Accel_X.Text = Global.mavStatus.sensor_offset.accel_cal_x.ToString();
            this.Accel_Y.Text = Global.mavStatus.sensor_offset.accel_cal_y.ToString();
            this.Accel_Z.Text = Global.mavStatus.sensor_offset.accel_cal_z.ToString();
            this.txtgpslng.Text = (Global.mavStatus.gps_Row.lng / Math.Pow(10, 7)).ToString();
            this.txtgpslat.Text = (Global.mavStatus.gps_Row.lat / Math.Pow(10, 7)).ToString();
            this.txtgpsalt.Text = (Global.mavStatus.gps_Row.alt / Math.Pow(10, 7)).ToString();
        }
    }
}
