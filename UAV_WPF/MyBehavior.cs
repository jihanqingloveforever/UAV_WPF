using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Interactivity;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;

namespace UAV_WPF
{
    public class MyBehavior : Behavior<UIElement>
    {
        private Canvas canvas;

        private bool isDragging = false;

        private Point mouseOffset;

        protected override void OnAttached()
        {
            base.OnAttached();

            this.AssociatedObject.MouseLeftButtonDown +=

                new System.Windows.Input.MouseButtonEventHandler(AssociatedObject_MouseLeftButtonDown);

            this.AssociatedObject.MouseMove +=

                new System.Windows.Input.MouseEventHandler(AssociatedObject_MouseMove);

            this.AssociatedObject.MouseLeftButtonUp +=

                new System.Windows.Input.MouseButtonEventHandler(AssociatedObject_MouseRightButtonUp);
        }

        protected override void OnDetaching()
        {
            base.OnDetaching();

            this.AssociatedObject.MouseLeftButtonDown -=

                new System.Windows.Input.MouseButtonEventHandler(AssociatedObject_MouseLeftButtonDown);

            this.AssociatedObject.MouseMove -=

                new System.Windows.Input.MouseEventHandler(AssociatedObject_MouseMove);

            this.AssociatedObject.MouseLeftButtonUp -=

                new System.Windows.Input.MouseButtonEventHandler(AssociatedObject_MouseRightButtonUp);
        }

        void AssociatedObject_MouseRightButtonUp(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            if (isDragging)
            {
                AssociatedObject.ReleaseMouseCapture();

                isDragging = false;
            }
        }

        void AssociatedObject_MouseMove(object sender, System.Windows.Input.MouseEventArgs e)
        {
            if (isDragging)
            {
                Point point = e.GetPosition(canvas);

                AssociatedObject.SetValue(Canvas.TopProperty, point.Y - mouseOffset.Y);

                AssociatedObject.SetValue(Canvas.LeftProperty, point.X - mouseOffset.X);
            }
        }

        void AssociatedObject_MouseLeftButtonDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            if (this.canvas == null)
            {
                canvas = VisualTreeHelper.GetParent(this.AssociatedObject) as Canvas;
            }

            isDragging = true;

            mouseOffset = e.GetPosition(AssociatedObject);

            AssociatedObject.CaptureMouse();
        }
    }
}
