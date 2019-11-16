using System;
using System.Collections.Generic;
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
    /// ConfigAccelerometerCalibration.xaml 的交互逻辑
    /// </summary>
    public partial class ConfigAccelerometerCalibration : Window
    {
        int result = 0;
        bool alCalibrationLevel = false;
        public Func<bool> btnCalibrationLevelEvent;
        public Func<int, bool> btnCalibrationAccelEvent;
        public Action btnCalibrationClose;
        public ConfigAccelerometerCalibration()
        {
            InitializeComponent();
            if (lbAccel.Content.ToString().Contains("左") || lbAccel.Content.ToString().Contains("右") || lbAccel.Content.ToString().Contains("上") || lbAccel.Content.ToString().Contains("下") || lbAccel.Content.ToString().Contains("反") || lbAccel.Content.ToString().Contains("水平"))
            {
                btnCalibrationAccel.Content = "完成时点击";
            }
        }

        public void setCalibrationAccelInfo(string info)
        {
            if (info.ToLower().Contains("calibration failed"))
            {
                BitmapImage bitmapImage = new BitmapImage(new Uri("Images\0180480005.jpg", UriKind.RelativeOrAbsolute));
                ShowImage.Source = bitmapImage;
                btnCalibrationAccel.Content = "完成";
                lbAccel.Content = "加速度计校准失败！";
                Global.CalibrationAccelInfo = "";
                Global.alCalibrationLevel = false;
            }
            else if (info.ToLower().Contains("calibration successful"))
            {
                BitmapImage bitmapImage = new BitmapImage(new Uri("Images\0180480005.jpg", UriKind.RelativeOrAbsolute));
                ShowImage.Source = bitmapImage;
                btnCalibrationAccel.Content = "完成";
                lbAccel.Content = "加速度计校准成功！";
                Global.CalibrationAccelInfo = "";
                Global.alCalibrationLevel = false;
            }
            else if (info.ToLower().Contains("left"))
            {
                BitmapImage bitmapImage = new BitmapImage(new Uri("Images\0180480005.jpg", UriKind.RelativeOrAbsolute));
                ShowImage.Source = bitmapImage;
                lbAccel.Content = "请左放置您的自驾仪！";
                btnCalibrationAccel.Content = "完成时点击";
                Global.CalibrationAccelInfo = "请左放置您的自驾仪！";
            }
            else if (info.ToLower().Contains("right"))
            {
                BitmapImage bitmapImage = new BitmapImage(new Uri("Images\0180480005.jpg", UriKind.RelativeOrAbsolute));
                ShowImage.Source = bitmapImage;
                lbAccel.Content = "请右放置您的自驾仪！";
                btnCalibrationAccel.Content = "完成时点击";
                Global.CalibrationAccelInfo = "请右放置您的自驾仪！";
            }
            else if (info.ToLower().Contains("down"))
            {
                BitmapImage bitmapImage = new BitmapImage(new Uri("Images\0180480005.jpg", UriKind.RelativeOrAbsolute));
                ShowImage.Source = bitmapImage;
                lbAccel.Content = "请向下放置您的自驾仪！";
                btnCalibrationAccel.Content = "完成时点击";
                Global.CalibrationAccelInfo = "请向下放置您的自驾仪！";
            }
            else if (info.ToLower().Contains("up"))
            {
                BitmapImage bitmapImage = new BitmapImage(new Uri("Images\0180480005.jpg", UriKind.RelativeOrAbsolute));
                ShowImage.Source = bitmapImage;
                lbAccel.Content = "请向上放置您的自驾仪！";
                btnCalibrationAccel.Content = "完成时点击";
                Global.CalibrationAccelInfo = "请向上放置您的自驾仪！";
            }
            else if (info.ToLower().Contains("back"))
            {
                BitmapImage bitmapImage = new BitmapImage(new Uri("Images\0180480005.jpg", UriKind.RelativeOrAbsolute));
                ShowImage.Source = bitmapImage;
                lbAccel.Content = "请反向放置您的自驾仪！";
                btnCalibrationAccel.Content = "完成时点击";
                Global.CalibrationAccelInfo = "请反向放置您的自驾仪！";
            }
            else if (info.ToLower().Contains("place vehicle level and press any key"))
            {
                if (Convert.ToInt32(btnCalibrationLevel.Tag) != 1)
                {
                    BitmapImage bitmapImage = new BitmapImage(new Uri("Images\0180480005.jpg", UriKind.RelativeOrAbsolute));
                    ShowImage.Source = bitmapImage;
                    lbAccel.Content = "请水平放置您的自驾仪！";
                    btnCalibrationAccel.Content = "完成时点击";
                    Global.CalibrationAccelInfo = "请水平放置您的自驾仪！";
                }
            }
            lbAccel.Visibility = Visibility.Collapsed;
        }
        /// <summary>
        /// 加速度计校准
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void BtnCalibrationAccel_Click(object sender, RoutedEventArgs e)
        {
            if (btnCalibrationAccel.Content.ToString() == "完成")
            {
                return;
            }
            if (!Global.isConn)
            {
                MessageBox.Show("请检查串口是否连接！", "提示", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }
            result++;
            btnCalibrationAccelEvent(result);
            if (Global.alCalibrationLevel)
            {
                btnCalibrationAccel.Content = "完成时点击";
            }
        }
        /// <summary>
        /// 水平校准
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void BtnCalibrationLevel_Click(object sender, RoutedEventArgs e)
        {
            if (btnCalibrationLevel.Content.ToString() == "完成")
            {
                return;
            }
            if (!Global.isConn)
            {
                MessageBox.Show("请检查串口是否连接！", "提示", MessageBoxButton.OK, MessageBoxImage.Information);
                return;
            }
            btnCalibrationLevel.Tag = 1;
            Global.alCalibrationLevel = btnCalibrationLevelEvent();
            btnCalibrationLevel.Content = "完成";
        }
    }
}
