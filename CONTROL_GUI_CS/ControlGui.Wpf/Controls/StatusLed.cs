using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;

namespace ControlGui.Wpf.Controls
{
    public class StatusLed : Control
    {
        public static readonly DependencyProperty StateProperty =
            DependencyProperty.Register(nameof(State), typeof(string), typeof(StatusLed),
                new PropertyMetadata("off", OnStateChanged));

        public string State
        {
            get => (string)GetValue(StateProperty);
            set => SetValue(StateProperty, value);
        }

        static StatusLed()
        {
            DefaultStyleKeyProperty.OverrideMetadata(typeof(StatusLed), new FrameworkPropertyMetadata(typeof(StatusLed)));
        }

        public StatusLed()
        {
            Width = 14;
            Height = 14;
        }

        private static void OnStateChanged(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            if (d is StatusLed led)
            {
                led.InvalidateVisual();
            }
        }

        protected override void OnRender(DrawingContext dc)
        {
            base.OnRender(dc);

            Color fillColor = State?.ToLowerInvariant() switch
            {
                "ready" or "streaming" or "online" => Theme.LedReadyColor,
                "warn" or "warning" => Theme.LedWarnColor,
                "error" or "fault" => Theme.LedErrorColor,
                "connecting" => Theme.LedConnectingColor,
                _ => Theme.LedOffColor
            };

            var brush = new SolidColorBrush(fillColor);
            var pen = new Pen(Theme.BorderHoverBrush, 1.0);
            double radius = Math.Min(ActualWidth, ActualHeight) / 2.0 - 1.0;
            if (radius > 0)
            {
                var center = new Point(ActualWidth / 2.0, ActualHeight / 2.0);
                dc.DrawEllipse(brush, pen, center, radius, radius);
            }
        }
    }
}
