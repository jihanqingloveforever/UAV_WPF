using MAVLink;
using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Text;
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
    /// FlyModel.xaml 的交互逻辑
    /// </summary>
    public partial class FlyModel : Window
    {
        public MavLinkInterface mavLinkInterface = new MavLinkInterface();
        private readonly System.Windows.Forms.Timer timer = new System.Windows.Forms.Timer();//定义时钟
        SerialPort sp = new SerialPort();
        private readonly double pwm = 0;
        public FlyModel()
        {
            InitializeComponent();
           
        }
        private string IdToMode(int n)
        {
            switch (n)
            {
                case 0:
                    return "自稳";                  
                case 2:
                    return "定高";                
                case 3:
                    return "任务";                 
                case 4:
                    return "引导";                  
                case 6:
                    return "返航";                  
                case 7:
                    return "环绕";                 
                case 9:
                    return "降落";                  
                case 16:
                    return "定点";                   
                default:
                    return "";                 
            }
        }
        public new void Activate()
        {
            #region 绑定数据源

            string[] Source = new string[] { "自稳", "定高", "定点", "任务", "环绕", "返航", "降落", "引导" };
            ComboBox[] flightmodes = new ComboBox[] { FlightMode1, FlightMode2, FlightMode3, FlightMode4, FlightMode5, FlightMode6 };
            foreach (ComboBox temp in flightmodes)
            {               
                temp.ItemsSource = Source;
            }
            CurrentMode.Content= FlightMode1.Text.Trim();
            CurrentPWM.Content = pwm.ToString();
            #endregion

            #region 给不同飞行模式设置初始值

            try
            {
                string mode1 = IdToMode(int.Parse(Global.mavStatus.paraDic["FLTMODE1"].ToString()));
                string mode2 = IdToMode(int.Parse(Global.mavStatus.paraDic["FLTMODE2"].ToString()));
                string mode3 = IdToMode(int.Parse(Global.mavStatus.paraDic["FLTMODE3"].ToString()));
                string mode4 = IdToMode(int.Parse(Global.mavStatus.paraDic["FLTMODE4"].ToString()));
                string mode5 = IdToMode(int.Parse(Global.mavStatus.paraDic["FLTMODE5"].ToString()));
                string mode6 = IdToMode(int.Parse(Global.mavStatus.paraDic["FLTMODE6"].ToString()));
                FlightMode1.SelectedItem = mode1;
                FlightMode2.SelectedItem = mode2;
                FlightMode3.SelectedItem = mode3;
                FlightMode4.SelectedItem = mode4;
                FlightMode5.SelectedItem = mode5;
                FlightMode6.SelectedItem = mode6;
            }
            catch
            {
                string mode1 = "自稳";
                string mode2 = "定高";
                string mode3 = "任务";
                string mode4 = "环绕";
                string mode5 = "返航";
                string mode6 = "降落";
                FlightMode1.SelectedItem = mode1;
                FlightMode2.SelectedItem = mode2;
                FlightMode3.SelectedItem = mode3;
                FlightMode4.SelectedItem = mode4;
                FlightMode5.SelectedItem = mode5;
                FlightMode6.SelectedItem = mode6;
            }
            if (Global.mavStatus.paraDic.ContainsKey("SIMPLE"))
            {
                var simple = int.Parse(Global.mavStatus.paraDic["SIMPLE"].ToString());
                Cb_simple1.IsChecked = ((simple >> 0 & 1) == 1);
                Cb_simple2.IsChecked = ((simple >> 1 & 1) == 1);
                Cb_simple3.IsChecked = ((simple >> 2 & 1) == 1);
                Cb_simple4.IsChecked = ((simple >> 3 & 1) == 1);
                Cb_simple5.IsChecked = ((simple >> 4 & 1) == 1);
                Cb_simple6.IsChecked = ((simple >> 5 & 1) == 1);
            }
            if (Global.mavStatus.paraDic.ContainsKey("SUPER_SIMPLE"))
            {
                var simple = int.Parse(Global.mavStatus.paraDic["SUPER_SIMPLE"].ToString());
                Cb_ss1.IsChecked = ((simple >> 0 & 1) == 1);
                Cb_ss2.IsChecked = ((simple >> 1 & 1) == 1);
                Cb_ss3.IsChecked = ((simple >> 2 & 1) == 1);
                Cb_ss4.IsChecked = ((simple >> 3 & 1) == 1);
                Cb_ss5.IsChecked = ((simple >> 4 & 1) == 1);
                Cb_ss6.IsChecked = ((simple >> 5 & 1) == 1);
            }
            #endregion

            timer.Tick += timer_Tick;
            timer.Enabled = true;
            timer.Interval = 100;
            timer.Start();
        }
        private void SetGeoFence(string paramname, float value, byte num)
        {
            MavLink.mavlink_Param_Set param_set = new MavLink.mavlink_Param_Set();
            param_set.param_value = value;
            param_set.target_system = Global.sysID;
            param_set.target_component = Global.compID;
            byte[] temp = Encoding.ASCII.GetBytes(paramname);
            Array.Resize(ref temp, 16);
            param_set.param_id = temp;
            param_set.param_type = num;//参数值类型
            mavLinkInterface.AssembleAndSendFrame((byte)MavLink.MAVLINK_MSG_ID.PARAM_SET, param_set);
        }
        private void timer_Tick(object sender, EventArgs e)
        {         
            if (pwm <= 1230)
            {
                CurrentMode.Content = FlightMode1.Text.Trim();
                CurrentPWM.Content = pwm.ToString();
            }
            else if (pwm > 1230 && pwm <= 1360)
            {
                CurrentMode.Content = FlightMode2.Text.Trim();
                CurrentPWM.Content = pwm.ToString();
            }
            else if (pwm > 1360 && pwm <= 1490)
            {
                CurrentMode.Content = FlightMode3.Text.Trim();
                CurrentPWM.Content = pwm.ToString();
            }
            else if (pwm > 1490 && pwm <= 1620)
            {
                CurrentMode.Content = FlightMode4.Text.Trim();
                CurrentPWM.Content = pwm.ToString();
            }
            else if (pwm > 1620 && pwm <= 1749)
            {
                CurrentMode.Content = FlightMode5.Text.Trim();
                CurrentPWM.Content = pwm.ToString();
            }
            else if (pwm > 1749)
            {
                CurrentMode.Content = FlightMode6.Text.Trim();
                CurrentPWM.Content = pwm.ToString();
            }
        }

        private bool SetParam(Control item, string paraname)
        {
            try
            {
                //暂时这样写
                switch (item.ToString())
                {
                    //Stabilize 0
                    case "自稳":
                        SetGeoFence(paraname, 0, 2);
                        break;
                    //Altitude 2 
                    case "定高":
                        SetGeoFence(paraname, 2, 2);
                        break;
                    //PosHold 16
                    case "定点":
                        SetGeoFence(paraname, 16, 2);
                        break;
                    //Auto 3
                    case "任务":
                        SetGeoFence(paraname, 3, 2);
                        break;
                    //Circle 7
                    case "环绕":
                        SetGeoFence(paraname, 7, 2);
                        break;
                    //RTL 6
                    case "返航":
                        SetGeoFence(paraname, 6, 2);
                        break;
                    //Land 9
                    case "降落":
                        SetGeoFence(paraname, 9, 2);
                        break;
                    //Guided 4
                    case "引导":
                        SetGeoFence(paraname, 4, 2);
                        break;
                }
                return true;
            }
            catch
            {
                return false;
            }
        }
        private void SaveModes_Click(object sender, RoutedEventArgs e)
        {
            {
                try
                {
                    SetParam(FlightMode1, "FLTMODE1");
                    SetParam(FlightMode2, "FLTMODE2");
                    SetParam(FlightMode3, "FLTMODE3");
                    SetParam(FlightMode4, "FLTMODE4");
                    SetParam(FlightMode5, "FLTMODE5");
                    SetParam(FlightMode6, "FLTMODE6");
                    float value = 0;

                    if(Cb_simple1.IsChecked == null || Cb_simple1.IsChecked == false){ value += 0; }
                    else{  value += (int)SimpleMode.Simple1; }
                    if (Cb_simple2.IsChecked == null || Cb_simple2.IsChecked == false) { value += 0; }
                    else { value += (int)SimpleMode.Simple2; }
                    if (Cb_simple3.IsChecked == null || Cb_simple3.IsChecked == false) { value += 0; }
                    else { value += (int)SimpleMode.Simple3; }
                    if (Cb_simple4.IsChecked == null || Cb_simple4.IsChecked == false) { value += 0; }
                    else { value += (int)SimpleMode.Simple4; }
                    if (Cb_simple5.IsChecked == null || Cb_simple5.IsChecked == false) { value += 0; }
                    else { value += (int)SimpleMode.Simple5; }
                    if (Cb_simple6.IsChecked == null || Cb_simple6.IsChecked == false) { value += 0; }
                    else { value += (int)SimpleMode.Simple6; }

                    SetGeoFence("SIMPLE", value, 2);
               
                   if(Cb_ss1.IsChecked==null||Cb_ss1.IsChecked == false) { value += 0; }
                   else { value += (int)SimpleMode.Simple1; ; }
                    if (Cb_ss2.IsChecked == null || Cb_ss2.IsChecked == false) { value += 0; }
                    else { value += (int)SimpleMode.Simple2; ; }
                    if (Cb_ss3.IsChecked == null || Cb_ss3.IsChecked == false) { value += 0; }
                    else { value += (int)SimpleMode.Simple3; ; }
                    if (Cb_ss4.IsChecked == null || Cb_ss4.IsChecked == false) { value += 0; }
                    else { value += (int)SimpleMode.Simple4; ; }
                    if (Cb_ss5.IsChecked == null || Cb_ss5.IsChecked == false) { value += 0; }
                    else { value += (int)SimpleMode.Simple5; ; }
                    if (Cb_ss6.IsChecked == null || Cb_ss6.IsChecked == false) { value += 0; }
                    else { value += (int)SimpleMode.Simple6; ; }
                    SetGeoFence("SUPER_SIMPLE", value, 2);
                    SaveModes.Content = "已保存";
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.ToString());
                }
            }
        }
        [Flags]
        public enum SimpleMode      //飞行模式枚举
        {
            None = 0,
            Simple1 = 1,
            Simple2 = 2,
            Simple3 = 4,
            Simple4 = 8,
            Simple5 = 16,
            Simple6 = 32
        }
    }
}
