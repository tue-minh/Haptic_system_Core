using System;
using System.Globalization;
using System.Windows;
using System.Windows.Controls;
using ControlGui.Core.Motor;

namespace ControlGui.Wpf.Panels
{
    public partial class GripperPanel : UserControl
    {
        private MotorWorker? _worker;
        private bool _isUpdatingUi = false;

        public GripperPanel()
        {
            InitializeComponent();
        }

        public void Bind(MotorWorker? worker)
        {
            _worker = worker;
        }

        public void UpdateTelemetry(MotorSnapshot snap)
        {
            if (!snap.Connected)
            {
                TxtServoStatus.Text = "Offline";
                TxtServoStatus.Foreground = Theme.TextDimBrush;
                TxtReadoutPos.Text = "----";
                TxtReadoutVolt.Text = "---- V";
                TxtReadoutLoad.Text = "---- %";
                TxtReadoutTemp.Text = "---- °C";
                return;
            }

            TxtServoStatus.Text = snap.ServoStatusMsg;
            TxtServoStatus.Foreground = snap.ServoOnline ? Theme.SuccessBrush : Theme.WarningBrush;

            if (snap.ServoOnline)
            {
                TxtReadoutPos.Text = $"{snap.ServoPos}";
                TxtReadoutVolt.Text = $"{snap.ServoVoltV:F1} V";
                TxtReadoutLoad.Text = $"{snap.ServoLoadPct:F1} %";
                TxtReadoutTemp.Text = $"{snap.ServoTempC} °C";
            }
        }

        private byte GetServoId()
        {
            return byte.TryParse(TxtServoId?.Text, out var id) ? id : MotorConstants.DefaultServoId;
        }

        private int GetSpeed()
        {
            return int.TryParse(TxtSpeed?.Text, out var spd) ? Math.Clamp(spd, 100, 3000) : 800;
        }

        private int GetAccel()
        {
            return int.TryParse(TxtAccel?.Text, out var acc) ? Math.Clamp(acc, 10, 200) : 30;
        }

        private void OnScanClick(object sender, RoutedEventArgs e)
        {
            _worker?.ScanServo();
        }

        private void OnPingClick(object sender, RoutedEventArgs e)
        {
            _worker?.PingServo(GetServoId());
        }

        private void OnOpenClick(object sender, RoutedEventArgs e)
        {
            _worker?.ServoMove(MotorConstants.GripperOpen, GetSpeed(), GetAccel(), GetServoId());
            SliderTargetPos.Value = MotorConstants.GripperOpen;
        }

        private void OnCloseClick(object sender, RoutedEventArgs e)
        {
            _worker?.ServoMove(MotorConstants.GripperClose, GetSpeed(), GetAccel(), GetServoId());
            SliderTargetPos.Value = MotorConstants.GripperClose;
        }

        private void OnMoveClick(object sender, RoutedEventArgs e)
        {
            int pos = (int)SliderTargetPos.Value;
            _worker?.ServoMove(pos, GetSpeed(), GetAccel(), GetServoId());
        }

        private void OnSliderTargetChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (_isUpdatingUi || TxtTargetPos == null || SliderTargetPos == null) return;
            _isUpdatingUi = true;
            TxtTargetPos.Text = ((int)SliderTargetPos.Value).ToString();
            _isUpdatingUi = false;
        }

        private void OnTargetTextChanged(object sender, TextChangedEventArgs e)
        {
            if (_isUpdatingUi || SliderTargetPos == null) return;
            if (int.TryParse(TxtTargetPos?.Text, out var pos))
            {
                pos = Math.Clamp(pos, MotorConstants.GripperPosMin, MotorConstants.GripperPosMax);
                _isUpdatingUi = true;
                SliderTargetPos.Value = pos;
                _isUpdatingUi = false;
            }
        }
    }
}
