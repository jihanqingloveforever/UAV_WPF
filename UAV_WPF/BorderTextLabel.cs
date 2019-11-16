using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Media;
using System.Globalization;
namespace UAV_WPF
{
   public class BorderTextLabel : System.Windows.Controls.Label
    {
        // 表示可通过诸如样式、数据绑定、动画和继承等方法设置的属性。
        public static readonly DependencyProperty TextProperty = DependencyProperty.Register("Text", typeof(string), typeof(BorderTextLabel), new FrameworkPropertyMetadata(string.Empty, Redraw));
        public static readonly DependencyProperty StrokeProperty = DependencyProperty.Register("Stroke", typeof(Brush), typeof(BorderTextLabel), new FrameworkPropertyMetadata(Brushes.Black, Redraw));
        public static readonly DependencyProperty StrokeThicknessProperty = DependencyProperty.Register("StrokeThickness", typeof(double), typeof(BorderTextLabel), new FrameworkPropertyMetadata((double)1, Redraw));
        private static void Redraw(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            //使元素的呈现失效，并强制执行完整的新布局处理过程。 
            //完成布局循环后调用 System.Windows.UIElement.OnRender(System.Windows.Media.DrawingContext)。
            ((BorderTextLabel)d).InvalidateVisual();
        }
        public string Text
        {
            get { return (string)GetValue(TextProperty); }
            set { SetValue(TextProperty, value); }
        }
        public Brush Stroke
        {
            get { return (Brush)GetValue(StrokeProperty); }
            set { SetValue(StrokeProperty, value); }
        }
        public double StrokeThickness
        {
            get { return (double)GetValue(StrokeThicknessProperty); }
            set { SetValue(StrokeThicknessProperty, value); }
        }
        protected override void OnRender(DrawingContext drawingContext)
        {
            base.OnRender(drawingContext);
            //    使用指定的文本、区域性、流方向、字体、字体大小和画笔初始化 
            //    System.Windows.Media.FormattedText 类新实例。
            FormattedText formattedText = new FormattedText(this.Text, CultureInfo.CurrentCulture, FlowDirection, new Typeface(FontFamily, FontStyle, FontWeight, FontStretch), FontSize, this.Foreground);
            if (double.IsNaN(this.Width))
                this.Width = formattedText.Width;
            if (double.IsNaN(this.Height))
                this.Height = formattedText.Height;
            Point startp = new Point(0, 0);
            if (this.HorizontalContentAlignment == HorizontalAlignment.Right)
                startp.X = this.Width = formattedText.Width;
            if (this.HorizontalContentAlignment == HorizontalAlignment.Center)
                startp.X = (this.Width - formattedText.Width) / 2;
            if (this.VerticalContentAlignment == VerticalAlignment.Bottom)
                startp.X = this.Height - formattedText.Height;
            if (this.VerticalContentAlignment == VerticalAlignment.Center)
                startp.X = (this.Height - formattedText.Height) / 2;
            //返回 System.Windows.Media.Geometry 对象，表示带格式的文本，包括所有标志符号和文本修饰
            var textgeometry = formattedText.BuildGeometry(startp);

            //   brush:   用于填充 System.Windows.Media.Geometry 的 System.Windows.Media.Brush。 此参数可选，而且可以   null。 如果 brush 为 null，则不绘制任何填充。
            //   pen: 用于对 System.Windows.Media.Geometry 描边的 System.Windows.Media.Pen。 此参数可选，而且可以为 null。   如果 pen 为 null，则不绘制任何笔划。
            //   geometry:   要绘制的 System.Windows.Media.Geometry。
            drawingContext.DrawGeometry(this.Foreground, new Pen(Stroke, StrokeThickness), textgeometry);

        }
    }
}
