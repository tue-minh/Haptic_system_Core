using System;
using System.Globalization;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using ControlGui.Core.Motor;

namespace ControlGui.Wpf.Panels
{
    public partial class StepperPanel : UserControl
    {
        private MotorWorker? _worker;
        private double _velocity = 5.0;
        private double _accel = 50.0;
        private bool _isUpdatingUi = false;

        public StepperPanel()
        {
            InitializeComponent();
            WireJogButtons();
        }

        private void WireJogButtons()
        {
            Controls.JogButton[] buttons = new[] { BtnXMinus, BtnXPlus, BtnYPlus, BtnYMinus, BtnZMinus, BtnZPlus };
            foreach (var b in buttons)
            {
                b.JogPressed += OnJogPressed;
                b.JogReleased += OnJogReleased;
            }
        }

        public void Bind(MotorWorker? worker)
        {
            _worker = worker;
            bool ok = worker != null && worker.Snapshot().Connected;
            SetMotionEnabled(ok && (worker?.Snapshot().Enabled ?? false));
        }

        public void UpdateTelemetry(MotorSnapshot snap)
        {
            TxtPosX.Text = $"{snap.X:+0.00;-0.00;+0.00} mm";
            TxtPosY.Text = $"{snap.Y:+0.00;-0.00;+0.00} mm";
            TxtPosZ.Text = $"{snap.Z:+0.00;-0.00;+0.00} mm";

            if (!snap.Connected)
            {
                TxtDriverStatus.Text = "OFFLINE";
                TxtDriverStatus.Foreground = Theme.TextDimBrush;
                BtnEnable.IsEnabled = false;
                BtnEnable.Content = "Enable Drivers";
                SetMotionEnabled(false);
                return;
            }

            BtnEnable.IsEnabled = true;
            if (snap.Enabled)
            {
                BtnEnable.Content = "Disable Drivers";
                TxtDriverStatus.Text = snap.Busy ? "BUSY" : "READY";
                TxtDriverStatus.Foreground = snap.Busy ? Theme.WarningBrush : Theme.SuccessBrush;
                SetMotionEnabled(true);
            }
            else
            {
                BtnEnable.Content = "Enable Drivers";
                TxtDriverStatus.Text = "DISABLED";
                TxtDriverStatus.Foreground = Theme.DangerBrush;
                SetMotionEnabled(false);
            }
        }

        private void SetMotionEnabled(bool enabled)
        {
            BtnXMinus.IsEnabled = enabled;
            BtnXPlus.IsEnabled = enabled;
            BtnYPlus.IsEnabled = enabled;
            BtnYMinus.IsEnabled = enabled;
            BtnZMinus.IsEnabled = enabled;
            BtnZPlus.IsEnabled = enabled;
        }

        private void OnEnableClick(object sender, RoutedEventArgs e)
        {
            if (_worker == null) return;
            bool current = _worker.Snapshot().Enabled;
            _worker.Enable(!current);
        }

        private void OnJogPressed(string axis, double sign)
        {
            if (_worker == null) return;
            double vel = _velocity * sign;
            _worker.Jog(axis, vel, _accel);
        }

        private void OnJogReleased()
        {
            _worker?.JogStop();
        }

        private void OnEstopClick(object sender, RoutedEventArgs e)
        {
            _worker?.Abort();
        }

        private void OnVelTextChanged(object sender, TextChangedEventArgs e)
        {
            if (_isUpdatingUi || SliderVel == null) return;
            if (double.TryParse(TxtVel?.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out var v))
            {
                _velocity = Math.Clamp(v, MotorConstants.VelMin, MotorConstants.VelMax);
                _isUpdatingUi = true;
                SliderVel.Value = _velocity;
                _isUpdatingUi = false;
            }
        }

        private void OnSliderVelChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (_isUpdatingUi || TxtVel == null || SliderVel == null) return;
            _velocity = SliderVel.Value;
            _isUpdatingUi = true;
            TxtVel.Text = _velocity.ToString("F1", CultureInfo.InvariantCulture);
            _isUpdatingUi = false;
        }

        private void OnAccelTextChanged(object sender, TextChangedEventArgs e)
        {
            if (_isUpdatingUi || SliderAccel == null) return;
            if (double.TryParse(TxtAccel?.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out var a))
            {
                _accel = Math.Clamp(a, MotorConstants.AccelMin, MotorConstants.AccelMax);
                _isUpdatingUi = true;
                SliderAccel.Value = _accel;
                _isUpdatingUi = false;
            }
        }

        private void OnSliderAccelChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (_isUpdatingUi || TxtAccel == null || SliderAccel == null) return;
            _accel = SliderAccel.Value;
            _isUpdatingUi = true;
            TxtAccel.Text = _accel.ToString("F0", CultureInfo.InvariantCulture);
            _isUpdatingUi = false;
        }
    }
}
