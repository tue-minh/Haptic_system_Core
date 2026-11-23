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
                BtnOpen.IsEnabled = false;
                BtnClose.IsEnabled = false;
                BtnMove.IsEnabled = false;
                BtnScan.IsEnabled = false;
                BtnPing.IsEnabled = false;
                SliderTargetPos.IsEnabled = false;
                TxtTargetPos.IsEnabled = false;
                return;
            }

            bool isScanning = snap.ServoScanning;
            BtnScan.IsEnabled = !isScanning;
            BtnPing.IsEnabled = !isScanning;
            BtnScan.Content = isScanning ? "Scanning..." : "Scan Servo";

            TxtServoStatus.Text = snap.ServoStatusMsg;
            TxtServoStatus.Foreground = snap.ServoOnline
                ? Theme.SuccessBrush
                : (isScanning ? Theme.WarningBrush : Theme.DangerBrush);

            if (snap.ServoOnline)
            {
                TxtReadoutPos.Text = $"{snap.ServoPos}";
                TxtReadoutVolt.Text = $"{snap.ServoVoltV:F1} V";
                TxtReadoutLoad.Text = $"{snap.ServoLoadPct:F1} %";
                TxtReadoutTemp.Text = $"{snap.ServoTempC} °C";
            }

            // Synchronize Servo ID text box from active telemetry if user is not actively editing
            if (TxtServoId != null && !TxtServoId.IsFocused && snap.ServoOnline && snap.ServoId > 0)
            {
                string idStr = snap.ServoId.ToString();
                if (TxtServoId.Text != idStr)
                {
                    _isUpdatingUi = true;
                    TxtServoId.Text = idStr;
                    _isUpdatingUi = false;
                }
            }

            bool canControl = snap.Connected && snap.ServoOnline;
            BtnOpen.IsEnabled = canControl;
            BtnClose.IsEnabled = canControl;
            BtnMove.IsEnabled = canControl;
            SliderTargetPos.IsEnabled = canControl;
            TxtTargetPos.IsEnabled = canControl;
        }

        private byte GetServoId()
        {
            return byte.TryParse(TxtServoId?.Text, out var id) ? id : MotorConstants.DefaultServoId;
        }

        private void OnServoIdTextChanged(object sender, TextChangedEventArgs e)
        {
            if (_isUpdatingUi || _worker == null) return;
            if (byte.TryParse(TxtServoId?.Text, out var id) && id > 0)
            {
                _worker.SetServoId(id);
            }
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
            _worker?.ServoMove(MotorConstants.GripperOpen, GetSpeed(), GetAccel());
            SliderTargetPos.Value = MotorConstants.GripperOpen;
        }

        private void OnCloseClick(object sender, RoutedEventArgs e)
        {
            _worker?.ServoMove(MotorConstants.GripperClose, GetSpeed(), GetAccel());
            SliderTargetPos.Value = MotorConstants.GripperClose;
        }

        private void OnMoveClick(object sender, RoutedEventArgs e)
        {
            int pos = (int)SliderTargetPos.Value;
            _worker?.ServoMove(pos, GetSpeed(), GetAccel());
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
