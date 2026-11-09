using System;
using System.Windows.Controls;
using System.Windows.Input;

namespace ControlGui.Wpf.Controls
{
    public class JogButton : Button
    {
        public string Axis { get; set; } = "x";
        public double Sign { get; set; } = 1.0;

        public event Action<string, double>? JogPressed;
        public event Action? JogReleased;

        private bool _isHeld = false;

        public JogButton()
        {
            PreviewMouseDown += OnMouseDown;
            PreviewMouseUp += OnMouseUp;
            MouseLeave += OnMouseLeave;
        }

        private void OnMouseDown(object sender, MouseButtonEventArgs e)
        {
            if (e.ChangedButton == MouseButton.Left && IsEnabled)
            {
                _isHeld = true;
                JogPressed?.Invoke(Axis, Sign);
            }
        }

        private void OnMouseUp(object sender, MouseButtonEventArgs e)
        {
            if (_isHeld)
            {
                _isHeld = false;
                JogReleased?.Invoke();
            }
        }

        private void OnMouseLeave(object sender, MouseEventArgs e)
        {
            if (_isHeld)
            {
                _isHeld = false;
                JogReleased?.Invoke();
            }
        }

        public void ForceRelease()
        {
            if (_isHeld)
            {
                _isHeld = false;
                JogReleased?.Invoke();
            }
        }
    }
}
