using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Windows;
using System.Windows.Threading;
using ControlGui.Core.Haptic;
using ControlGui.Core.Motor;
using ControlGui.Core.Paxini;
using ControlGui.Core.Recording;

namespace ControlGui.Wpf
{
    public partial class MainWindow : Window
    {
        private const int RefreshFps = 30;

        private readonly ConcurrentQueue<string> _logQueue = new();
        private MotorWorker? _motor;
        private readonly Dictionary<string, SensorLink> _sensorLinks = new();
        private Stm32HapticStreamer? _hapticStreamer;
        private readonly HapticPayloadSynthesizer _hapticPreviewSynth = new();
        private readonly RecordingManager _recorder;

        private readonly DispatcherTimer _timer;

        public MainWindow()
        {
            InitializeComponent();

            string defaultRecordDir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Unified_Control");
            _recorder = new RecordingManager(PostLog, defaultRecordDir);
            PanelRecording.SetDefaultDirs(defaultRecordDir);

            WireEvents();

            _timer = new DispatcherTimer
            {
                Interval = TimeSpan.FromMilliseconds(1000.0 / RefreshFps)
            };
            _timer.Tick += OnTick;
            _timer.Start();

            PostLog("Control GUI initialized. Environment ready.");
            PostLog($"Recordings will be written to: {defaultRecordDir}");
        }

        public void PostLog(string msg)
        {
            string line = $"[{DateTime.Now:HH:mm:ss.fff}] {msg}";
            _logQueue.Enqueue(line);
        }

        private void WireEvents()
        {
            // Connection Panel
            PanelConnection.ConnectRequested += OnConnectRole;
            PanelConnection.DisconnectRequested += OnDisconnectRole;
            PanelConnection.ConnectAllRequested += OnConnectAll;
            PanelConnection.DisconnectAllRequested += OnDisconnectAll;

            // Haptic Panel
            PanelHaptic.EnabledChanged += OnHapticToggled;
            PanelHaptic.PingRequested += OnPingEsp32;

            // Recording Panel
            PanelRecording.StartRequested += OnStartRecording;
            PanelRecording.StopRequested += OnStopRecording;
            PanelRecording.OutputDirChanged += dir => _recorder.OutputDir = dir;

            // Sensor Monitor Panel (Tare requests)
            PanelSensorMonitor.TareLeftRequested += () => _sensorLinks.GetValueOrDefault("L")?.RequestTare();
            PanelSensorMonitor.TareRightRequested += () => _sensorLinks.GetValueOrDefault("R")?.RequestTare();
            PanelSensorMonitor.TareBothRequested += () =>
            {
                _sensorLinks.GetValueOrDefault("L")?.RequestTare();
                _sensorLinks.GetValueOrDefault("R")?.RequestTare();
            };
            PanelSensorMonitor.ResetTareRequested += () =>
            {
                _sensorLinks.GetValueOrDefault("L")?.ResetTare();
                _sensorLinks.GetValueOrDefault("R")?.ResetTare();
            };

            Closing += OnWindowClosing;
        }

        private void OnConnectRole(string role, string port, int baud)
        {
            try
            {
                if (role == "motor")
                {
                    PostLog($"[main] Connecting motor controller on {port} @ {baud}...");
                    _motor = new MotorWorker(port, baud, PostLog, pollHz: 15.0);
                    PanelStepper.Bind(_motor);
                    PanelGripper.Bind(_motor);
                    _motor.Start();

                    // Start or attach haptic streamer
                    RestartHapticStreamer();
                }
                else if (role == "sensor_l")
                {
                    PostLog($"[main] Connecting Left Sensor on {port} @ {baud}...");
                    var link = new SensorLink("L", port, PostLog, baudrate: baud);
                    _sensorLinks["L"] = link;
                    link.Connect();
                }
                else if (role == "sensor_r")
                {
                    PostLog($"[main] Connecting Right Sensor on {port} @ {baud}...");
                    var link = new SensorLink("R", port, PostLog, baudrate: baud);
                    _sensorLinks["R"] = link;
                    link.Connect();
                }
            }
            catch (Exception ex)
            {
                PostLog($"[main] Failed to connect {role}: {ex.Message}");
            }
        }

        private void OnDisconnectRole(string role)
        {
            if (role == "motor")
            {
                PostLog("[main] Disconnecting motor controller...");
                _hapticStreamer?.Stop();
                _hapticStreamer?.Dispose();
                _hapticStreamer = null;

                if (_motor != null)
                {
                    _motor.Shutdown();
                    _motor.Dispose();
                    _motor = null;
                }
                PanelStepper.Bind(null);
                PanelGripper.Bind(null);
                PanelConnection.SetRoleStatus("motor", false, false, "Disconnected", "off");
            }
            else if (role == "sensor_l" && _sensorLinks.TryGetValue("L", out var linkL))
            {
                PostLog("[main] Disconnecting Left Sensor...");
                linkL.Disconnect();
                _sensorLinks.Remove("L");
                PanelConnection.SetRoleStatus("sensor_l", false, false, "Disconnected", "off");
            }
            else if (role == "sensor_r" && _sensorLinks.TryGetValue("R", out var linkR))
            {
                PostLog("[main] Disconnecting Right Sensor...");
                linkR.Disconnect();
                _sensorLinks.Remove("R");
                PanelConnection.SetRoleStatus("sensor_r", false, false, "Disconnected", "off");
            }
        }

        private void OnConnectAll()
        {
            var motorCfg = PanelConnection.GetRoleConfig("motor");
            var lCfg = PanelConnection.GetRoleConfig("sensor_l");
            var rCfg = PanelConnection.GetRoleConfig("sensor_r");

            OnConnectRole("motor", motorCfg.port, motorCfg.baud);
            OnConnectRole("sensor_l", lCfg.port, lCfg.baud);
            OnConnectRole("sensor_r", rCfg.port, rCfg.baud);
        }

        private void OnDisconnectAll()
        {
            OnDisconnectRole("motor");
            OnDisconnectRole("sensor_l");
            OnDisconnectRole("sensor_r");
        }

        private void RestartHapticStreamer()
        {
            _hapticStreamer?.Stop();
            _hapticStreamer?.Dispose();
            _hapticStreamer = null;

            if (PanelHaptic.IsStreamingEnabled && _motor != null)
            {
                _hapticStreamer = new Stm32HapticStreamer(
                    writer: bytes => _motor?.WriteFrame(bytes) ?? 0,
                    log: PostLog
                );
                _hapticStreamer.Start();
            }
        }

        private void OnHapticToggled(bool enabled)
        {
            RestartHapticStreamer();
        }

        private void OnPingEsp32()
        {
            _motor?.PingHaptic();
        }

        private void OnStartRecording()
        {
            _recorder.Start(
                _motor,
                _sensorLinks.GetValueOrDefault("L"),
                _sensorLinks.GetValueOrDefault("R"),
                rateHz: PanelRecording.SampleRateHz
            );
        }

        private void OnStopRecording()
        {
            _recorder.Stop(
                _sensorLinks.GetValueOrDefault("L"),
                _sensorLinks.GetValueOrDefault("R")
            );
        }

        private void OnTick(object? sender, EventArgs e)
        {
            // 1. Drain background logs
            if (!_logQueue.IsEmpty)
            {
                var sb = new System.Text.StringBuilder();
                while (_logQueue.TryDequeue(out var line))
                {
                    sb.AppendLine(line);
                }
                TxtLog.AppendText(sb.ToString());
                TxtLog.ScrollToEnd();
            }

            // 2. Poll motor
            var motorSnap = _motor?.Snapshot();
            bool motorConnected = motorSnap != null && motorSnap.Connected;
            if (motorSnap != null)
            {
                PanelStepper.UpdateTelemetry(motorSnap);
                PanelGripper.UpdateTelemetry(motorSnap);

                string ledState = !motorSnap.Connected ? (motorSnap.Connecting ? "connecting" : "off")
                                : (motorSnap.Enabled ? "ready" : "warn");
                string desc = motorSnap.Connected
                    ? $"{motorSnap.Firmware} · {(motorSnap.Enabled ? "Drivers ON" : "Drivers OFF")}"
                    : (motorSnap.Connecting ? "Connecting..." : "Disconnected");

                PanelConnection.SetRoleStatus("motor", motorSnap.Connected, motorSnap.Connecting, desc, ledState);

                if (!string.IsNullOrEmpty(motorSnap.HapticPing))
                {
                    PanelHaptic.SetPingResult(motorSnap.HapticPing, motorSnap.HapticPingOk, motorSnap.HapticPingPending);
                }
            }

            // 3. Poll sensors
            SensorLink? linkL = _sensorLinks.GetValueOrDefault("L");
            SensorLink? linkR = _sensorLinks.GetValueOrDefault("R");

            SensorReading? readingL = linkL?.Reading;
            SensorReading? readingR = linkR?.Reading;

            if (linkL != null)
            {
                string ledState = linkL.State == "streaming" ? "ready" : (linkL.State == "connecting" ? "connecting" : (linkL.State == "error" ? "error" : "off"));
                string desc = linkL.State == "streaming"
                    ? $"ID {linkL.AddrText} · {linkL.RateHz:F0} Hz"
                    : (linkL.State == "error" ? "Error" : linkL.State);
                PanelConnection.SetRoleStatus("sensor_l", linkL.IsActive(), linkL.State == "connecting", desc, ledState);
            }

            if (linkR != null)
            {
                string ledState = linkR.State == "streaming" ? "ready" : (linkR.State == "connecting" ? "connecting" : (linkR.State == "error" ? "error" : "off"));
                string desc = linkR.State == "streaming"
                    ? $"ID {linkR.AddrText} · {linkR.RateHz:F0} Hz"
                    : (linkR.State == "error" ? "Error" : linkR.State);
                PanelConnection.SetRoleStatus("sensor_r", linkR.IsActive(), linkR.State == "connecting", desc, ledState);
            }

            // Update Sensor Monitor (2D Plot + Metrics)
            PanelSensorMonitor.UpdateTelemetry(readingL, readingR);

            // 4. Update Tabbed Views
            int tabIndex = TabsSurface.SelectedIndex;
            if (tabIndex == 0) // Taxel Map (Live)
            {
                ViewTaxelMap.Update(readingL, readingR);
            }
            else if (tabIndex == 1) // 3D Cloud
            {
                View3D.Update(readingL, readingR);
            }
            else if (tabIndex == 2) // Haptic Valves
            {
                // Generate live preview payload if not streaming
                double nowSec = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0;
                byte[] previewPayload = _hapticPreviewSynth.Synthesize(readingL, readingR, nowSec: nowSec);
                var snap = HapticSnapshot.FromPayload(previewPayload, nowSec);
                ViewValves.Update(snap, _hapticStreamer != null);
            }

            // 5. Update Haptic Streamer
            if (_hapticStreamer != null)
            {
                _hapticStreamer.UpdateSensors(readingL, readingR);
                var hapticSnap = _hapticStreamer.GetSnapshot();
                PanelHaptic.UpdateStatus(hapticSnap, true, motorConnected);
            }
            else
            {
                PanelHaptic.UpdateStatus(null, false, motorConnected);
            }

            // 6. Update Recording Panel
            PanelRecording.SetRecordingState(
                _recorder.IsRecording,
                _recorder.ElapsedSeconds,
                _recorder.RowsLogged,
                _recorder.CurrentFiles.Count > 0 ? _recorder.CurrentFiles[0] : ""
            );
        }

        private void OnWindowClosing(object? sender, System.ComponentModel.CancelEventArgs e)
        {
            _timer.Stop();
            _recorder.Stop();
            _hapticStreamer?.Stop();
            _hapticStreamer?.Dispose();
            _motor?.Shutdown();
            _motor?.Dispose();

            foreach (var link in _sensorLinks.Values)
            {
                link.Disconnect();
            }
        }
    }
}
