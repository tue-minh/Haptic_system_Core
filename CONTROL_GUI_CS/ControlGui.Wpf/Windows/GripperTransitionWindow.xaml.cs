using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;
using ControlGui.Core.Haptic;
using ControlGui.Core.Motor;
using ControlGui.Core.Paxini;
using Microsoft.Win32;
using Path = System.IO.Path;

namespace ControlGui.Wpf.Windows
{
    public partial class GripperTransitionWindow : Window
    {
        private readonly Func<MotorWorker?> _getMotor;
        private readonly Func<SensorLink?> _getLeft;
        private readonly Func<SensorLink?> _getRight;
        private readonly Action<string> _log;
        private readonly DispatcherTimer _uiTimer;

        private GripperTransitionRunner? _runner;
        private MotorWorker? _ownMotor;
        private Stm32HapticStreamer? _ownHapticStreamer;
        private SensorLink? _ownLeft;
        private SensorLink? _ownRight;
        private bool _uiReady = false;
        private bool _updatingServoUi = false;

        private static readonly Brush[] ValveBrushes =
        {
            Brushes.Crimson, Brushes.DarkOrange, Brushes.SeaGreen, Brushes.RoyalBlue, Brushes.Purple
        };

        public GripperTransitionWindow(
            Func<MotorWorker?> getMotor,
            Func<SensorLink?> getLeft,
            Func<SensorLink?> getRight,
            Action<string>? log = null,
            GripperMotionMode initialMotionMode = GripperMotionMode.None)
        {
            _getMotor = getMotor;
            _getLeft = getLeft;
            _getRight = getRight;
            _log = log ?? (_ => { });
            InitializeComponent();
            _uiReady = true;

            SelectMotionMode(initialMotionMode);
            string prefix = initialMotionMode == GripperMotionMode.None ? "gripper_transition" : "gripper_motion";
            TxtDbPath.Text = Path.Combine(
                "recordings",
                $"{prefix}_{DateTime.Now:yyyyMMdd_HHmmss}.db");

            RefreshComPorts();

            _uiTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(50) };
            _uiTimer.Tick += OnUiTick;
            _uiTimer.Start();

            Closing += (_, _) =>
            {
                _uiTimer.Stop();
                _runner?.Stop();
                _runner?.Dispose();
                _runner = null;
                DisposeOwnConnections();
            };
        }

        private MotorWorker? ResolveMotor()
        {
            var motor = _getMotor?.Invoke();
            if (motor != null && motor.Snapshot().Connected) return motor;
            return _ownMotor;
        }

        private SensorLink? ActiveLink(Func<SensorLink?> getExternal, SensorLink? own)
        {
            var external = getExternal();
            if (external != null && external.IsActive()) return external;
            if (own != null && own.IsActive()) return own;
            return external ?? own;
        }

        private SensorReading? ResolveLeft()
        {
            var link = ActiveLink(_getLeft, _ownLeft);
            return link != null && link.IsActive() ? link.Reading : null;
        }

        private SensorReading? ResolveRight()
        {
            var link = ActiveLink(_getRight, _ownRight);
            return link != null && link.IsActive() ? link.Reading : null;
        }

        // ------------------------------------------------------------------
        // Connections
        // ------------------------------------------------------------------

        private static void FillPortCombo(ComboBox combo)
        {
            string? previous = combo.SelectedItem as string;
            combo.Items.Clear();

            var ports = SerialPort.GetPortNames()
                .OrderBy(p => p, StringComparer.OrdinalIgnoreCase)
                .ToList();
            foreach (var port in ports) combo.Items.Add(port);
            combo.Items.Add("MOCK_PORT");

            if (!string.IsNullOrEmpty(previous) && combo.Items.Contains(previous))
                combo.SelectedItem = previous;
            else if (ports.Count > 0)
                combo.SelectedIndex = 0;
            else
                combo.SelectedIndex = combo.Items.Count - 1;
        }

        private void RefreshComPorts()
        {
            FillPortCombo(CboMotorPort);
            FillPortCombo(CboLeftPort);
            FillPortCombo(CboRightPort);
        }

        private void OnRefreshPortsClick(object sender, RoutedEventArgs e)
        {
            RefreshComPorts();
        }

        private void SyncOwnHapticStreamer(bool want)
        {
            if (want && _ownHapticStreamer == null && _ownMotor != null)
            {
                _ownHapticStreamer = new Stm32HapticStreamer(
                    writer: bytes => _ownMotor?.WriteFrame(bytes) ?? 0,
                    log: _log);
                _ownHapticStreamer.Start();
            }
            else if (!want && _ownHapticStreamer != null)
            {
                _ownHapticStreamer.Stop();
                _ownHapticStreamer.Dispose();
                _ownHapticStreamer = null;
            }
        }

        private void OnMotorConnectClick(object sender, RoutedEventArgs e)
        {
            if (_ownMotor != null)
            {
                _runner?.Stop();
                SyncOwnHapticStreamer(false);
                _ownMotor.Shutdown();
                _ownMotor.Dispose();
                _ownMotor = null;
                BtnMotorConnect.Content = "Connect";
                return;
            }

            string? port = CboMotorPort.SelectedItem as string;
            if (string.IsNullOrWhiteSpace(port))
            {
                MessageBox.Show(this, "Select the STM32 COM port first.", "COM Port",
                    MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }

            _ownMotor = new MotorWorker(port, 1_000_000, _log, pollHz: 15.0);
            _ownMotor.Start();
            BtnMotorConnect.Content = "Disconnect";
        }

        private void OnLeftConnectClick(object sender, RoutedEventArgs e)
        {
            if (_ownLeft != null)
            {
                _ownLeft.Disconnect();
                _ownLeft.Dispose();
                _ownLeft = null;
                BtnLeftConnect.Content = "Connect";
                return;
            }

            string? port = CboLeftPort.SelectedItem as string;
            if (string.IsNullOrWhiteSpace(port))
            {
                MessageBox.Show(this, "Select the left PaXini COM port first.", "COM Port",
                    MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }

            _ownLeft = new SensorLink("L", port, _log);
            _ownLeft.Connect();
            BtnLeftConnect.Content = "Disconnect";
        }

        private void OnRightConnectClick(object sender, RoutedEventArgs e)
        {
            if (_ownRight != null)
            {
                _ownRight.Disconnect();
                _ownRight.Dispose();
                _ownRight = null;
                BtnRightConnect.Content = "Connect";
                return;
            }

            string? port = CboRightPort.SelectedItem as string;
            if (string.IsNullOrWhiteSpace(port))
            {
                MessageBox.Show(this, "Select the right PaXini COM port first.", "COM Port",
                    MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }

            _ownRight = new SensorLink("R", port, _log);
            _ownRight.Connect();
            BtnRightConnect.Content = "Disconnect";
        }

        private void DisposeOwnConnections()
        {
            SyncOwnHapticStreamer(false);
            if (_ownMotor != null)
            {
                _ownMotor.Shutdown();
                _ownMotor.Dispose();
                _ownMotor = null;
            }
            if (_ownLeft != null)
            {
                _ownLeft.Disconnect();
                _ownLeft.Dispose();
                _ownLeft = null;
            }
            if (_ownRight != null)
            {
                _ownRight.Disconnect();
                _ownRight.Dispose();
                _ownRight = null;
            }
        }

        private void OnBrowseDb(object sender, RoutedEventArgs e)
        {
            var dlg = new SaveFileDialog
            {
                Filter = "SQLite Database (*.db)|*.db|All files (*.*)|*.*",
                FileName = Path.GetFileName(TxtDbPath.Text),
                InitialDirectory = Path.GetDirectoryName(Path.GetFullPath(TxtDbPath.Text)) ?? ""
            };
            if (dlg.ShowDialog(this) == true)
            {
                TxtDbPath.Text = dlg.FileName;
            }
        }

        private void OnStartClick(object sender, RoutedEventArgs e)
        {
            var motor = ResolveMotor();
            if (motor == null)
            {
                MessageBox.Show(this, "STM32 motor/USB link is not connected.", "Not Connected",
                    MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }

            var snap = motor.Snapshot();
            if (!snap.ServoOnline)
            {
                MessageBox.Show(this, "The gripper servo is offline. Connect the STM32 link and wait for the servo handshake (or ping it on the main window).",
                    "Servo Offline", MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }

            if (!TryParseConfig(out var cfg, out string err))
            {
                MessageBox.Show(this, err, "Invalid Parameters", MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }

            if (File.Exists(Path.GetFullPath(cfg.DbPath)))
            {
                string prefix = cfg.MotionMode == GripperMotionMode.None ? "gripper_transition" : "gripper_motion";
                string fresh = Path.Combine(
                    Path.GetDirectoryName(cfg.DbPath) ?? "recordings",
                    $"{prefix}_{DateTime.Now:yyyyMMdd_HHmmss}.db");
                TxtDbPath.Text = fresh;
                cfg = cfg with { DbPath = fresh };
            }

            // Pause idle haptic streamer while the runner owns valve synthesis/streaming
            SyncOwnHapticStreamer(false);

            _runner?.Stop();
            _runner?.Dispose();
            _runner = new GripperTransitionRunner(motor, cfg, ResolveLeft, ResolveRight, _log);
            _runner.Start();

            BtnStart.IsEnabled = false;
            BtnStop.IsEnabled = true;
            TxtOnsets.Text = "Transitions: waiting for the closing phase...";
            TxtDbInfo.Text = cfg.DbPath;
        }

        private void OnStopClick(object sender, RoutedEventArgs e)
        {
            _runner?.Stop();
            BtnStop.IsEnabled = false;
        }

        private byte GetServoId()
        {
            return byte.TryParse(TxtServoId?.Text, out var id) && id > 0 ? id : MotorConstants.DefaultServoId;
        }

        private void OnServoIdTextChanged(object sender, TextChangedEventArgs e)
        {
            if (!_uiReady || _updatingServoUi) return;
            var motor = ResolveMotor();
            if (motor == null) return;
            if (byte.TryParse(TxtServoId?.Text, out var id) && id > 0)
            {
                motor.SetServoId(id);
            }
        }

        private void OnScanServoClick(object sender, RoutedEventArgs e)
        {
            ResolveMotor()?.ScanServo();
        }

        private void OnPingServoClick(object sender, RoutedEventArgs e)
        {
            ResolveMotor()?.PingServo(GetServoId());
        }

        private void SelectMotionMode(GripperMotionMode mode)
        {
            string tag = mode switch
            {
                GripperMotionMode.ZMinusReturn => "z_minus",
                GripperMotionMode.XPlusReturn => "x_plus",
                GripperMotionMode.YPlusReturn => "y_plus",
                _ => "none"
            };
            foreach (ComboBoxItem item in CboMotionMode.Items)
            {
                if (string.Equals(item.Tag?.ToString(), tag, StringComparison.OrdinalIgnoreCase))
                {
                    CboMotionMode.SelectedItem = item;
                    break;
                }
            }
        }

        private GripperMotionMode GetSelectedMotionMode()
        {
            string tag = (CboMotionMode?.SelectedItem as ComboBoxItem)?.Tag?.ToString() ?? "none";
            return tag switch
            {
                "z_minus" => GripperMotionMode.ZMinusReturn,
                "x_plus" => GripperMotionMode.XPlusReturn,
                "y_plus" => GripperMotionMode.YPlusReturn,
                _ => GripperMotionMode.None
            };
        }

        private void OnMotionModeChanged(object sender, SelectionChangedEventArgs e)
        {
            if (!_uiReady || TxtDbPath == null) return;
            var mode = GetSelectedMotionMode();
            string curName = Path.GetFileName(TxtDbPath.Text);
            if (curName.StartsWith("gripper_transition_", StringComparison.OrdinalIgnoreCase) ||
                curName.StartsWith("gripper_motion_", StringComparison.OrdinalIgnoreCase))
            {
                string prefix = mode == GripperMotionMode.None ? "gripper_transition" : "gripper_motion";
                TxtDbPath.Text = Path.Combine(
                    Path.GetDirectoryName(TxtDbPath.Text) ?? "recordings",
                    $"{prefix}_{DateTime.Now:yyyyMMdd_HHmmss}.db");
            }
        }

        private void OnStepEnableClick(object sender, RoutedEventArgs e)
        {
            var motor = ResolveMotor();
            if (motor == null) return;
            motor.Enable(!motor.Snapshot().Enabled);
        }

        private bool TryParseConfig(out GripperTransitionConfig cfg, out string error)
        {
            cfg = new GripperTransitionConfig();
            error = "";

            if (!int.TryParse(TxtOpenPos.Text, out int openPos))
            {
                error = "Open position must be an integer servo step.";
                return false;
            }
            if (!int.TryParse(TxtClosePos.Text, out int closePos))
            {
                error = "Close position must be an integer servo step.";
                return false;
            }
            openPos = Math.Clamp(openPos, MotorConstants.GripperPosMin, MotorConstants.GripperPosMax - 1);
            closePos = Math.Clamp(closePos, MotorConstants.GripperPosMin + 1, MotorConstants.GripperPosMax);
            if (closePos <= openPos)
            {
                error = "Close position must be greater than the open position.";
                return false;
            }

            if (!int.TryParse(TxtSpeed.Text, out int speed))
            {
                error = "Speed must be an integer (steps/s).";
                return false;
            }
            speed = Math.Clamp(speed, MotorConstants.GripperSpeedMin, MotorConstants.GripperSpeedMax);

            if (!int.TryParse(TxtAccel.Text, out int accel))
            {
                error = "Acceleration must be an integer.";
                return false;
            }
            accel = Math.Clamp(accel, MotorConstants.GripperAccelMin, MotorConstants.GripperAccelMax);

            if (!double.TryParse(TxtSettle.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double settle) || settle < 0 || settle > 60)
            {
                error = "Settle-open time must be 0..60 s.";
                return false;
            }
            if (!double.TryParse(TxtHold.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double hold) || hold < 0 || hold > 600)
            {
                error = "Hold-closed time must be 0..600 s.";
                return false;
            }
            if (!double.TryParse(TxtRateHz.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double rateHz) || rateHz < 5 || rateHz > 200)
            {
                error = "Sample rate must be 5..200 Hz.";
                return false;
            }
            if (!double.TryParse(TxtVref.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double vref) || vref <= 0)
            {
                error = "ADC Vref must be > 0 mV.";
                return false;
            }

            var motionMode = GetSelectedMotionMode();
            if (!double.TryParse(TxtMotionDist.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double motionDist) || motionDist <= 0 || motionDist > 200)
            {
                error = "Motion distance must be > 0 and <= 200 mm.";
                return false;
            }
            if (!double.TryParse(TxtHoldTarget.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double holdTarget) || holdTarget < 0 || holdTarget > 120)
            {
                error = "Hold at target must be 0..120 s.";
                return false;
            }
            if (!double.TryParse(TxtXyVel.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double xyVel) || xyVel < 1 || xyVel > 100)
            {
                error = "X/Y velocity must be 1..100 mm/s.";
                return false;
            }
            if (!double.TryParse(TxtXyAccel.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double xyAccel) || xyAccel < 10 || xyAccel > 1000)
            {
                error = "X/Y acceleration must be 10..1000 mm/s^2.";
                return false;
            }
            if (!double.TryParse(TxtZVel.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double zVel) || zVel < 1 || zVel > 50)
            {
                error = "Z velocity must be 1..50 mm/s.";
                return false;
            }
            if (!double.TryParse(TxtZAccel.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double zAccel) || zAccel < 10 || zAccel > 1000)
            {
                error = "Z acceleration must be 10..1000 mm/s^2.";
                return false;
            }

            byte csIdx = Convert.ToByte(((ComboBoxItem)CboSpiCs.SelectedItem).Tag);

            string dbPath = TxtDbPath.Text.Trim();
            if (string.IsNullOrEmpty(dbPath))
            {
                string prefix = motionMode == GripperMotionMode.None ? "gripper_transition" : "gripper_motion";
                dbPath = Path.Combine("recordings", $"{prefix}_{DateTime.Now:yyyyMMdd_HHmmss}.db");
                TxtDbPath.Text = dbPath;
            }

            cfg = new GripperTransitionConfig
            {
                OpenPos = openPos,
                ClosePos = closePos,
                Speed = speed,
                Accel = accel,
                SettleOpenSec = settle,
                HoldCloseSec = hold,
                ReleaseAtEnd = ChkRelease.IsChecked == true,
                MotionMode = motionMode,
                MotionDistanceMm = motionDist,
                HoldAtTargetSec = holdTarget,
                XyVelMmS = xyVel,
                XyAccelMmS2 = xyAccel,
                ZVelMmS = zVel,
                ZAccelMmS2 = zAccel,
                SampleRateHz = rateHz,
                SpiCsIdx = csIdx,
                VrefMv = vref,
                DbPath = dbPath
            };
            return true;
        }

        private void OnUiTick(object? sender, EventArgs e)
        {
            var external = _getMotor();
            bool externalConnected = external != null && external.Snapshot().Connected;

            BtnMotorConnect.IsEnabled = !externalConnected;
            if (!externalConnected)
            {
                BtnMotorConnect.Content = _ownMotor != null ? "Disconnect" : "Connect";
            }
            else
            {
                BtnMotorConnect.Content = "Connect";
            }

            var motor = ResolveMotor();
            if (motor != null && motor.Snapshot().Connected)
            {
                var ms = motor.Snapshot();
                string where = externalConnected ? "main window" : "this window";
                TxtComStatus.Text = $"STM32: connected ({motor.Port}, via {where})";
                TxtComStatus.Foreground = Brushes.ForestGreen;

                bool scanning = ms.ServoScanning;
                BtnScanServo.IsEnabled = !scanning;
                BtnPingServo.IsEnabled = !scanning;
                BtnScanServo.Content = scanning ? "Scanning..." : "Scan ID";
                TxtServoStatus.Text = ms.ServoStatusMsg;
                TxtServoStatus.Foreground = ms.ServoOnline
                    ? Brushes.ForestGreen
                    : (scanning ? Brushes.DarkOrange : Brushes.Crimson);

                if (TxtServoId != null && !TxtServoId.IsFocused && ms.ServoOnline && ms.ServoId > 0)
                {
                    string idStr = ms.ServoId.ToString();
                    if (TxtServoId.Text != idStr)
                    {
                        _updatingServoUi = true;
                        TxtServoId.Text = idStr;
                        _updatingServoUi = false;
                    }
                }

                if (_runner == null && ms.ServoOnline)
                {
                    TxtLivePos.Text = ms.ServoPos.ToString();
                    TxtLiveLoad.Text = $"{ms.ServoLoadPct:F1}";
                }

                BtnStepEnable.IsEnabled = _runner == null || !_runner.GetStatus().Running;
                BtnStepEnable.Content = ms.Enabled ? "Disable Stepper" : "Enable Stepper";
                TxtStepperPos.Text = $"X{ms.X:+0.0;-0.0;+0.0} Y{ms.Y:+0.0;-0.0;+0.0} Z{ms.Z:+0.0;-0.0;+0.0} ({(ms.Enabled ? "ON" : "OFF")})";
                TxtStepperPos.Foreground = ms.Enabled ? Brushes.ForestGreen : Brushes.DimGray;
                if (_runner == null || !_runner.GetStatus().Running)
                {
                    TxtLiveStep.Text = $"{ms.X:+0.0;-0.0;0.0} {ms.Y:+0.0;-0.0;0.0} {ms.Z:+0.0;-0.0;0.0}";
                }
            }
            else if (_ownMotor != null)
            {
                var ms = _ownMotor.Snapshot();
                TxtComStatus.Text = string.IsNullOrEmpty(ms.Error)
                    ? $"STM32: connecting to {_ownMotor.Port}..."
                    : $"STM32: error — {ms.Error}";
                TxtComStatus.Foreground = string.IsNullOrEmpty(ms.Error) ? Brushes.DarkOrange : Brushes.Crimson;
                BtnScanServo.IsEnabled = false;
                BtnPingServo.IsEnabled = false;
                BtnStepEnable.IsEnabled = false;
                TxtServoStatus.Text = "Offline";
                TxtServoStatus.Foreground = Brushes.Gray;
                TxtStepperPos.Text = "Stepper: ---";
            }
            else
            {
                TxtComStatus.Text = "STM32: not connected — pick a port above and click Connect";
                TxtComStatus.Foreground = Brushes.Gray;
                BtnScanServo.IsEnabled = false;
                BtnPingServo.IsEnabled = false;
                BtnStepEnable.IsEnabled = false;
                TxtServoStatus.Text = "Offline";
                TxtServoStatus.Foreground = Brushes.Gray;
                TxtStepperPos.Text = "Stepper: ---";
            }

            var externalL = _getLeft();
            var externalR = _getRight();
            BtnLeftConnect.IsEnabled = externalL == null || !externalL.IsActive();
            BtnRightConnect.IsEnabled = externalR == null || !externalR.IsActive();
            if (!BtnLeftConnect.IsEnabled) BtnLeftConnect.Content = "Connect";
            if (!BtnRightConnect.IsEnabled) BtnRightConnect.Content = "Connect";

            var linkL = ActiveLink(_getLeft, _ownLeft);
            var linkR = ActiveLink(_getRight, _ownRight);
            string L = linkL == null ? "off" : (linkL.IsActive() ? $"{linkL.Port} {linkL.RateHz:F0} Hz" : linkL.State);
            string R = linkR == null ? "off" : (linkR.IsActive() ? $"{linkR.Port} {linkR.RateHz:F0} Hz" : linkR.State);
            TxtSensorStatus.Text = $"PaXini sensors: L={L}, R={R}";

            bool runnerActive = _runner != null && _runner.GetStatus().Running;
            bool wantIdleStreamer = !externalConnected && motor != null && motor.Snapshot().Connected && !runnerActive;
            SyncOwnHapticStreamer(wantIdleStreamer);
            _ownHapticStreamer?.UpdateSensors(ResolveLeft(), ResolveRight());

            if (_runner == null)
            {
                return;
            }

            var st = _runner.GetStatus();
            ProgRun.IsIndeterminate = st.Running;
            if (!st.Running)
            {
                ProgRun.Value = st.State == "completed" ? 1.0 : 0.0;
                BtnStart.IsEnabled = true;
                BtnStop.IsEnabled = false;
            }

            TxtStatus.Text =
                $"State: {st.State} | Phase: {st.Phase} ({st.PhaseTimeSec:F2} s) | Elapsed: {st.ElapsedSec:F1} s\n" +
                $"Samples: {st.SampleCount} | Rate: {st.ActualRateHz:F1} Hz | Read errors: {st.ReadErrors}" +
                (string.IsNullOrEmpty(st.Error) ? "" : $"\nError: {st.Error}");

            TxtDbInfo.Text = st.DbPath;

            var onsets = _runner.GetOnsets();
            if (onsets != null)
            {
                TxtOnsets.Text = "Transitions: " + onsets.Describe();
            }
            else if (st.LeftOnsetSec.HasValue || st.RightOnsetSec.HasValue || st.ValveOnsetSec.HasValue)
            {
                TxtOnsets.Text = $"Transitions: L {st.LeftOnsetSec:F2} s | R {st.RightOnsetSec:F2} s | valve {st.ValveOnsetSec:F2} s";
            }

            var live = _runner.GetLiveSamples(1);
            if (live.Count > 0)
            {
                var s = live[^1];
                TxtLivePos.Text = s.ServoPos.ToString();
                TxtLiveLoad.Text = double.IsNaN(s.ServoLoadPct) ? "---" : $"{s.ServoLoadPct:F1}";
                TxtLiveLF.Text = s.Left != null ? $"{s.Left.FMag:F2}" : "---";
                TxtLiveLFz.Text = s.Left != null ? $"{s.Left.Fz:F2}" : "---";
                TxtLiveRF.Text = s.Right != null ? $"{s.Right.FMag:F2}" : "---";
                TxtLiveRFz.Text = s.Right != null ? $"{s.Right.Fz:F2}" : "---";
                double vmax = s.AdcMv.Where(v => !double.IsNaN(v)).DefaultIfEmpty(0).Max();
                TxtLiveValve.Text = $"{vmax:F0}";
                if (!double.IsNaN(s.StepX) && !double.IsNaN(s.StepZ))
                    TxtLiveStep.Text = $"{s.StepX:+0.0;-0.0;0.0} {s.StepY:+0.0;-0.0;0.0} {s.StepZ:+0.0;-0.0;0.0}";
            }

            var samples = _runner.GetLiveSamples(600);
            DrawForcePlot(samples);
            DrawValvePlot(samples);
        }

        private void OnCanvasSizeChanged(object sender, SizeChangedEventArgs e)
        {
            if (_runner == null) return;
            var samples = _runner.GetLiveSamples(600);
            DrawForcePlot(samples);
            DrawValvePlot(samples);
        }

        private void DrawForcePlot(List<GripperTransitionSample> samples)
        {
            CanvasForce.Children.Clear();
            double w = CanvasForce.ActualWidth;
            double h = CanvasForce.ActualHeight;
            if (w < 40 || h < 40 || samples.Count < 2) return;

            const double padL = 48, padR = 16, padT = 12, padB = 22;
            double pw = w - padL - padR;
            double ph = h - padT - padB;
            if (pw <= 10 || ph <= 10) return;

            double tMin = samples[0].TimeSec;
            double tMax = Math.Max(tMin + 0.1, samples[^1].TimeSec);

            double fMax = 0.5;
            foreach (var s in samples)
            {
                if (s.Left != null) fMax = Math.Max(fMax, s.Left.FMag);
                if (s.Right != null) fMax = Math.Max(fMax, s.Right.FMag);
            }
            fMax = Math.Ceiling(fMax * 2.0) / 2.0;

            DrawGrid(CanvasForce, padL, padT, pw, ph, 4);
            for (int i = 0; i <= 4; i++)
            {
                double f = fMax * (4 - i) / 4.0;
                AddLabel(CanvasForce, $"{f:F2} N", 2, padT + ph * i / 4.0 - 7, Brushes.DimGray);
            }
            AddLabel(CanvasForce, $"{tMin:F1} s", padL, padT + ph + 3, Brushes.DimGray);
            AddLabel(CanvasForce, $"{tMax:F1} s", padL + pw - 40, padT + ph + 3, Brushes.DimGray);

            var leftLine = new Polyline { Stroke = Brushes.RoyalBlue, StrokeThickness = 1.7 };
            var rightLine = new Polyline { Stroke = Brushes.ForestGreen, StrokeThickness = 1.7 };

            foreach (var s in samples)
            {
                double x = padL + (s.TimeSec - tMin) / (tMax - tMin) * pw;
                if (s.Left != null && !double.IsNaN(s.Left.FMag))
                {
                    double y = padT + (1.0 - Math.Clamp(s.Left.FMag / fMax, 0.0, 1.0)) * ph;
                    leftLine.Points.Add(new Point(x, y));
                }
                if (s.Right != null && !double.IsNaN(s.Right.FMag))
                {
                    double y = padT + (1.0 - Math.Clamp(s.Right.FMag / fMax, 0.0, 1.0)) * ph;
                    rightLine.Points.Add(new Point(x, y));
                }
            }

            CanvasForce.Children.Add(leftLine);
            CanvasForce.Children.Add(rightLine);
        }

        private void DrawValvePlot(List<GripperTransitionSample> samples)
        {
            CanvasValve.Children.Clear();
            double w = CanvasValve.ActualWidth;
            double h = CanvasValve.ActualHeight;
            if (w < 40 || h < 40 || samples.Count < 2) return;

            const double padL = 48, padR = 16, padT = 12, padB = 22;
            double pw = w - padL - padR;
            double ph = h - padT - padB;
            if (pw <= 10 || ph <= 10) return;

            double tMin = samples[0].TimeSec;
            double tMax = Math.Max(tMin + 0.1, samples[^1].TimeSec);

            double vMax = 100.0;
            foreach (var s in samples)
            {
                for (int c = 0; c < 5; c++)
                {
                    double v = s.AdcMv != null && c < s.AdcMv.Length ? s.AdcMv[c] : double.NaN;
                    if (!double.IsNaN(v)) vMax = Math.Max(vMax, v);
                }
            }
            vMax = Math.Min(3300.0, Math.Ceiling(vMax / 100.0) * 100.0);

            DrawGrid(CanvasValve, padL, padT, pw, ph, 4);
            for (int i = 0; i <= 4; i++)
            {
                double mv = vMax * (4 - i) / 4.0;
                AddLabel(CanvasValve, $"{mv:F0} mV", 2, padT + ph * i / 4.0 - 7, Brushes.DimGray);
            }
            AddLabel(CanvasValve, $"{tMin:F1} s", padL, padT + ph + 3, Brushes.DimGray);
            AddLabel(CanvasValve, $"{tMax:F1} s", padL + pw - 40, padT + ph + 3, Brushes.DimGray);

            for (int c = 0; c < 5; c++)
            {
                int ch = c;
                var line = new Polyline { Stroke = ValveBrushes[ch], StrokeThickness = 1.5 };
                foreach (var s in samples)
                {
                    double v = s.AdcMv != null && ch < s.AdcMv.Length ? s.AdcMv[ch] : double.NaN;
                    if (double.IsNaN(v)) continue;
                    double x = padL + (s.TimeSec - tMin) / (tMax - tMin) * pw;
                    double y = padT + (1.0 - Math.Clamp(v / vMax, 0.0, 1.0)) * ph;
                    line.Points.Add(new Point(x, y));
                }
                CanvasValve.Children.Add(line);
                AddLabel(CanvasValve, $"CH{ch}", padL + 4 + ch * 34, padT + 2, ValveBrushes[ch]);
            }
        }

        private static void DrawGrid(Canvas canvas, double padL, double padT, double pw, double ph, int rows)
        {
            for (int i = 0; i <= rows; i++)
            {
                double y = padT + ph * i / rows;
                canvas.Children.Add(new Line
                {
                    X1 = padL, Y1 = y, X2 = padL + pw, Y2 = y,
                    Stroke = new SolidColorBrush(Color.FromRgb(236, 236, 236)),
                    StrokeThickness = 1
                });
            }
        }

        private static void AddLabel(Canvas canvas, string text, double x, double y, Brush color)
        {
            var tb = new TextBlock
            {
                Text = text,
                FontSize = 10,
                FontFamily = new FontFamily("Consolas"),
                Foreground = color
            };
            Canvas.SetLeft(tb, x);
            Canvas.SetTop(tb, y);
            canvas.Children.Add(tb);
        }
    }
}
