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
    /// SetTakeOffAltForm.xaml 的交互逻辑
    /// </summary>
    public partial class SetTakeOffAltForm : Window
    {
        public SetTakeOffAltForm()
        {
            InitializeComponent();
        }

        private void BtnCancel_Click(object sender, RoutedEventArgs e)
        {
            this.Close();         
        }

        private void BtnOk_Click(object sender, RoutedEventArgs e)
        {
            double defaultAlt = 10.0;
            if (!double.TryParse(this.txtTakeOffAlt.Text.Trim(),out defaultAlt))
            {
                MessageBox.Show("输入高度错误!");
                return;
            }
            else if (Convert.ToSingle(txtTakeOffAlt.Text.Trim()) <= 0)
            {
                MessageBox.Show("输入高度无效!");
                return;
            }
            else
            {
                Global.takeOffAlt = Convert.ToSingle(txtTakeOffAlt.Text.Trim());
                this.Close();
            }
        }
    }
}
