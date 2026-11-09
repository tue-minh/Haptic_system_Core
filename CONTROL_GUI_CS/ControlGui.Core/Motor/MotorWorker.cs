using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.Threading;
using ControlGui.Core.TueUsb;

namespace ControlGui.Core.Motor
{
    public class MotorWorker : IDisposable
    {
        private const int MaxConsecutiveErrors = 8;

        private readonly Thread _thread;
        private readonly CancellationTokenSource _cts = new();
        private readonly AutoResetEvent _wakeEvent = new(false);
        private readonly Action<string> _log;

        private readonly ConcurrentQueue<Action<TueUsbClient>> _urgentQueue = new();
        private readonly ConcurrentQueue<Action<TueUsbClient>> _commandQueue = new();

        private readonly object _stateLock = new object();
        private readonly object _writeLock = new object();
        private MotorSnapshot _snapshot = new() { Connecting = true };

        private TueUsbClient? _client;
        private ISerialPortTransport? _rawTransport;
        private byte _servoId = MotorConstants.DefaultServoId;
        private bool _servoOnline = false;
        private int _servoConsecutiveErrors = 0;
        private int _consecutiveErrors = 0;

        public string Port { get; }
        public int BaudRate { get; }
        public double PollHz { get; }
        public bool Mock { get; }

        public MotorWorker(
            string port,
            int baudrate = 1000000,
            Action<string>? log = null,
            double pollHz = 15.0,
            bool mock = false,
            byte defaultServoId = MotorConstants.DefaultServoId)
        {
            Port = port;
            BaudRate = baudrate;
            PollHz = pollHz;
            Mock = mock || port == "MOCK_PORT" || string.IsNullOrEmpty(port);
            _servoId = defaultServoId;
            _log = log ?? (_ => { });

            _snapshot = _snapshot with { ServoId = _servoId, PollHz = pollHz };

            _thread = new Thread(Run)
            {
                IsBackground = true,
                Name = $"MotorWorker-{port}"
            };
        }

        public void Start() => _thread.Start();

        public void Shutdown()
        {
            _cts.Cancel();
            _wakeEvent.Set();
        }

        public MotorSnapshot Snapshot()
        {
            lock (_stateLock)
            {
                return _snapshot;
            }
        }

        private void Update(Func<MotorSnapshot, MotorSnapshot> updateFn)
        {
            lock (_stateLock)
            {
                _snapshot = updateFn(_snapshot);
            }
        }

        public int WriteFrame(byte[] frame)
        {
            if (_rawTransport == null || !_rawTransport.IsOpen)
                return 0;

            lock (_writeLock)
            {
                try
                {
                    _rawTransport.Write(frame, 0, frame.Length);
                    return frame.Length;
                }
                catch (Exception ex)
                {
                    _log($"[motor {Port}] Haptic frame short/failed write: {ex.Message}");
                    return 0;
                }
            }
        }

        // --- Commands Submission ---
        public void Enable(bool on)
        {
            _commandQueue.Enqueue(c =>
            {
                c.StepEnable(on);
                _log($"[motor {Port}] Drivers {(on ? "energised" : "released")}");
            });
            _wakeEvent.Set();
        }

        public void Jog(string axis, double velocity, double accel = 50.0)
        {
            _commandQueue.Enqueue(c =>
            {
                if (!Snapshot().Enabled)
                {
                    _log($"[motor {Port}] Jog ignored - stepper drivers are disabled");
                    Update(s => s with { Jogging = null });
                    return;
                }

                if (axis.Equals("x", StringComparison.OrdinalIgnoreCase))
                    c.StepJogXY(velocity, 0.0, accel);
                else if (axis.Equals("y", StringComparison.OrdinalIgnoreCase))
                    c.StepJogXY(0.0, velocity, accel);
                else if (axis.Equals("z", StringComparison.OrdinalIgnoreCase))
                    c.StepJogZ(velocity, accel);
            });
            Update(s => s with { Jogging = axis });
            _wakeEvent.Set();
        }

        public void JogStop()
        {
            // Immediate priority: step_abort stops clock immediately without MCU stack wedge
            _urgentQueue.Enqueue(c => c.StepAbort());
            Update(s => s with { Jogging = null });
            _wakeEvent.Set();
        }

        public void Abort()
        {
            _urgentQueue.Enqueue(c =>
            {
                c.StepAbort();
                _log($"[motor {Port}] EMERGENCY ABORT");
            });
            Update(s => s with { Jogging = null });
            _wakeEvent.Set();
        }

        public void SetServoId(byte sid)
        {
            _servoId = sid;
            Update(s => s with { ServoId = sid });
            _log($"[motor {Port}] Servo target ID set to {sid}");
        }

        public void ScanServo(byte maxId = 16)
        {
            Update(s => s with { ServoScanning = true, ServoStatusMsg = $"Scanning IDs 1..{maxId}..." });
            _commandQueue.Enqueue(c =>
            {
                _log($"[motor {Port}] Scanning for servo (IDs 1..{maxId})...");
                byte? foundId = null;

                // Probe current ID first, then the rest
                var order = new System.Collections.Generic.List<byte> { _servoId };
                for (byte i = 1; i <= maxId; i++)
                {
                    if (i != _servoId) order.Add(i);
                }

                foreach (byte sid in order)
                {
                    if (_cts.Token.IsCancellationRequested || !_urgentQueue.IsEmpty) break;
                    Thread.Sleep(5);
                    if (c.ServoPing(sid, 0.08))
                    {
                        foundId = sid;
                        break;
                    }
                }

                if (foundId.HasValue)
                {
                    _servoId = foundId.Value;
                    _servoOnline = true;
                    _servoConsecutiveErrors = 0;
                    string msg = $"Servo found: ID {foundId.Value}";
                    _log($"[motor {Port}] {msg}");

                    try
                    {
                        Thread.Sleep(5);
                        var fb = c.ServoGetFeedback(foundId.Value);
                        Update(s => s with
                        {
                            ServoId = foundId.Value,
                            ServoOnline = true,
                            ServoScanning = false,
                            ServoStatusMsg = msg,
                            ServoPos = fb.Dynamics.Position,
                            ServoVoltV = fb.Health.VoltageV,
                            ServoLoadPct = fb.Dynamics.LoadPercent,
                            ServoTempC = fb.Health.TemperatureC,
                            ServoMoving = fb.Dynamics.IsMoving
                        });
                    }
                    catch
                    {
                        Update(s => s with
                        {
                            ServoId = foundId.Value,
                            ServoOnline = true,
                            ServoScanning = false,
                            ServoStatusMsg = msg
                        });
                    }
                }
                else
                {
                    string msg = $"No servo found (1..{maxId})";
                    _log($"[motor {Port}] {msg}");
                    _servoOnline = false;
                    Update(s => s with { ServoOnline = false, ServoScanning = false, ServoStatusMsg = msg });
                }
            });
            _wakeEvent.Set();
        }

        public void PingServo(byte? sid = null)
        {
            byte targetId = sid ?? _servoId;
            Update(s => s with { ServoScanning = true, ServoStatusMsg = $"Pinging ID {targetId}..." });
            _commandQueue.Enqueue(c =>
            {
                _log($"[motor {Port}] Pinging servo ID {targetId}...");
                Thread.Sleep(5);
                bool ok = c.ServoPing(targetId, 0.15);
                if (ok)
                {
                    _servoId = targetId;
                    _servoOnline = true;
                    _servoConsecutiveErrors = 0;
                    string msg = $"Servo ID {targetId}: ONLINE";
                    _log($"[motor {Port}] {msg}");

                    try
                    {
                        Thread.Sleep(5);
                        var fb = c.ServoGetFeedback(targetId);
                        Update(s => s with
                        {
                            ServoId = targetId,
                            ServoOnline = true,
                            ServoScanning = false,
                            ServoStatusMsg = msg,
                            ServoPos = fb.Dynamics.Position,
                            ServoVoltV = fb.Health.VoltageV,
                            ServoLoadPct = fb.Dynamics.LoadPercent,
                            ServoTempC = fb.Health.TemperatureC,
                            ServoMoving = fb.Dynamics.IsMoving
                        });
                    }
                    catch
                    {
                        Update(s => s with
                        {
                            ServoId = targetId,
                            ServoOnline = true,
                            ServoScanning = false,
                            ServoStatusMsg = msg
                        });
                    }
                }
                else
                {
                    string msg = $"Servo ID {targetId}: NO RESPONSE";
                    _log($"[motor {Port}] {msg}");
                    _servoOnline = false;
                    Update(s => s with { ServoOnline = false, ServoScanning = false, ServoStatusMsg = msg });
                }
            });
            _wakeEvent.Set();
        }

        public void ServoMove(int pos, int speed = 800, int accel = 30, byte? sid = null)
        {
            byte targetId = sid ?? _servoId;
            pos = Math.Clamp(pos, MotorConstants.GripperPosMin, MotorConstants.GripperPosMax);
            speed = Math.Clamp(speed, MotorConstants.GripperSpeedMin, MotorConstants.GripperSpeedMax);
            accel = Math.Clamp(accel, MotorConstants.GripperAccelMin, MotorConstants.GripperAccelMax);

            _commandQueue.Enqueue(c =>
            {
                _log($"[motor {Port}] ServoMove(id={targetId}, pos={pos}, speed={speed}, accel={accel})");

                // Small inter-transaction pause for half-duplex bus settle
                Thread.Sleep(5);

                // Ensure torque is enabled
                try
                {
                    c.ServoTorqueEnable(targetId, true);
                }
                catch (Exception ex)
                {
                    _log($"[motor {Port}] ServoTorqueEnable notice: {ex.Message}");
                }

                // Inter-frame delay for STS3215 internal write digestion
                Thread.Sleep(5);

                // Send goal position
                c.ServoSetPos(targetId, pos, speed, accel);
                _log($"[motor {Port}] ServoMove -> OK (pos={pos})");
            });
            _wakeEvent.Set();
        }

        public void ServoTorque(bool on, byte? sid = null)
        {
            byte targetId = sid ?? _servoId;
            _commandQueue.Enqueue(c =>
            {
                _log($"[motor {Port}] ServoTorque(id={targetId}, on={on})");
                Thread.Sleep(5);
                c.ServoTorqueEnable(targetId, on);
            });
            _wakeEvent.Set();
        }

        public void PingHaptic(byte token = 0)
        {
            if (_client == null || !_client.IsConnected)
            {
                Update(s => s with
                {
                    HapticPing = "ESP32 ping: STM32 link is not connected",
                    HapticPingOk = false,
                    HapticPingPending = false
                });
                return;
            }

            Update(s => s with { HapticPingPending = true });
            _commandQueue.Enqueue(c =>
            {
                try
                {
                    var info = c.HapticPing(token);
                    string text = info.ToString();
                    Update(s => s with { HapticPing = text, HapticPingOk = true, HapticPingPending = false });
                    _log($"[motor {Port}] ESP32 ping OK: {text}");
                }
                catch (Exception ex)
                {
                    string diagDetail = "";
                    try
                    {
                        var diag = c.HapticUartDiag();
                        diagDetail = $" | {diag}";
                    }
                    catch { }

                    string msg = $"ESP32 ping failed: {ex.Message}{diagDetail}";
                    Update(s => s with { HapticPing = msg, HapticPingOk = false, HapticPingPending = false });
                    _log($"[motor {Port}] {msg}");
                }
            });
            _wakeEvent.Set();
        }

        // --- Worker Loop ---
        private void Run()
        {
            try
            {
                _log($"[motor {Port}] Opening serial port @ {BaudRate} baud...");
                _client = OpenClient();
                if (!Handshake())
                {
                    return;
                }
            }
            catch (Exception ex)
            {
                Fail($"Connect failed: {ex.Message}");
                CloseClient();
                return;
            }

            WorkerLoop();
            CloseClient();
            Update(s => s with { Streaming = false, Connecting = false });
        }

        private TueUsbClient OpenClient()
        {
            var client = new TueUsbClient(Port, BaudRate, 0.35, Mock);
            if (!Mock)
            {
                _rawTransport = new SystemSerialPortTransport(Port, BaudRate, 350, 350);
                var lockedTransport = new LockedSerialPort(_rawTransport, _writeLock);
                client.SetCustomTransport(lockedTransport);
            }
            client.Connect();
            _log($"[motor {Port}] Port handle opened successfully.");
            return client;
        }

        private bool Handshake()
        {
            var c = _client!;
            _log($"[motor {Port}] Handshake: Ping / GetInfo / StepGetConfig");

            Exception? lastEx = null;
            bool pingOk = false;
            for (int attempt = 1; attempt <= 3; attempt++)
            {
                try
                {
                    _log($"[motor {Port}] Sending SYS_PING (attempt {attempt}/3)...");
                    c.Ping();
                    pingOk = true;
                    _log($"[motor {Port}] SYS_PING response OK");
                    break;
                }
                catch (Exception ex)
                {
                    lastEx = ex;
                    _log($"[motor {Port}] SYS_PING attempt {attempt} failed: {ex.Message}");
                    Thread.Sleep(60);
                }
            }

            if (!pingOk)
            {
                throw lastEx ?? new TimeoutException("MCU did not respond to SYS_PING");
            }

            var info = c.GetInfo();
            var cfg = c.StepGetConfig();

            Update(s => s with
            {
                Connected = true,
                Connecting = false,
                Streaming = true,
                Error = "",
                Board = $"HW {info.HwId}.{info.HwRev}",
                Firmware = info.VersionString,
                StepsPerMmXy = cfg.XyStepsPerMm,
                StepsPerMmZ = cfg.ZStepsPerMm
            });

            _log($"[motor {Port}] Connected: hw={info.HwId}.{info.HwRev} fw={info.VersionString} " +
                 $"xy={cfg.XyStepsPerMm} steps/mm z={cfg.ZStepsPerMm} steps/mm");

            // Initial probe of servo
            try
            {
                Thread.Sleep(20);
                if (c.ServoPing(_servoId, 0.12))
                {
                    _servoOnline = true;
                    _log($"[motor {Port}] Servo ID {_servoId}: ONLINE");
                    try
                    {
                        Thread.Sleep(5);
                        var fb = c.ServoGetFeedback(_servoId);
                        Update(s => s with
                        {
                            ServoId = _servoId,
                            ServoOnline = true,
                            ServoStatusMsg = $"Servo ID {_servoId}: Online",
                            ServoPos = fb.Dynamics.Position,
                            ServoVoltV = fb.Health.VoltageV,
                            ServoLoadPct = fb.Dynamics.LoadPercent,
                            ServoTempC = fb.Health.TemperatureC,
                            ServoMoving = fb.Dynamics.IsMoving
                        });
                    }
                    catch
                    {
                        Update(s => s with { ServoId = _servoId, ServoOnline = true, ServoStatusMsg = $"Servo ID {_servoId}: Online" });
                    }
                }
                else
                {
                    _servoOnline = false;
                    Update(s => s with { ServoOnline = false, ServoStatusMsg = "Servo offline (click Scan)" });
                    _log($"[motor {Port}] Servo ID {_servoId} not responding (use 'Scan Servo')");
                }
            }
            catch (Exception ex)
            {
                _servoOnline = false;
                Update(s => s with { ServoOnline = false, ServoStatusMsg = "Servo offline (click Scan)" });
                _log($"[motor {Port}] Servo probe: {ex.Message}");
            }

            return true;
        }

        private void WorkerLoop()
        {
            var c = _client!;
            double pollPeriodSec = PollHz > 0 ? 1.0 / PollHz : 0.05;
            var sw = Stopwatch.StartNew();
            double nextPoll = 0.0;

            var rateSw = Stopwatch.StartNew();
            int rateCount = 0;

            while (!_cts.Token.IsCancellationRequested)
            {
                bool handled = false;

                // 1. Drain urgent commands first
                while (_urgentQueue.TryDequeue(out var urgentCmd))
                {
                    try
                    {
                        urgentCmd(c);
                        NoteSuccess();
                    }
                    catch (Exception ex)
                    {
                        NoteError(ex, "urgent command");
                    }
                    handled = true;
                }

                // 2. Drain regular commands
                while (_commandQueue.TryDequeue(out var cmd))
                {
                    try
                    {
                        cmd(c);
                        NoteSuccess();
                    }
                    catch (Exception ex)
                    {
                        NoteError(ex, "command");
                    }
                    handled = true;
                }

                // 3. Periodic telemetry poll
                double now = sw.Elapsed.TotalSeconds;
                if (now >= nextPoll)
                {
                    PollTelemetry(c);
                    nextPoll = sw.Elapsed.TotalSeconds + pollPeriodSec;
                    handled = true;

                    rateCount++;
                    if (rateSw.ElapsedMilliseconds >= 1000)
                    {
                        double hz = rateCount / (rateSw.ElapsedMilliseconds / 1000.0);
                        rateCount = 0;
                        rateSw.Restart();
                        Update(s => s with { PollHz = hz });
                    }
                }

                if (!handled)
                {
                    _wakeEvent.WaitOne(30);
                }
            }
        }

        private void PollTelemetry(TueUsbClient c)
        {
            try
            {
                var (px, py) = c.StepGetPosXY();
                double pz = c.StepGetPosZ();
                var status = c.StepGetStatus();
                NoteSuccess();

                Update(s => s with
                {
                    X = px,
                    Y = py,
                    Z = pz,
                    Busy = status.IsBusy,
                    Enabled = status.IsEnabled
                });
            }
            catch (Exception ex)
            {
                NoteError(ex, "poll");
                return;
            }

            if (_servoOnline)
            {
                try
                {
                    var fb = c.ServoGetFeedback(_servoId);
                    _servoConsecutiveErrors = 0;
                    Update(s => s with
                    {
                        ServoPos = fb.Dynamics.Position,
                        ServoVoltV = fb.Health.VoltageV,
                        ServoLoadPct = fb.Dynamics.LoadPercent,
                        ServoTempC = fb.Health.TemperatureC,
                        ServoMoving = fb.Dynamics.IsMoving,
                        ServoOnline = true
                    });
                }
                catch (Exception ex)
                {
                    _servoConsecutiveErrors++;
                    if (_servoConsecutiveErrors >= 4)
                    {
                        _servoOnline = false;
                        Update(s => s with { ServoOnline = false, ServoStatusMsg = "Servo offline (sync lost)" });
                        _log($"[motor {Port}] Servo ID {_servoId} feedback lost: {ex.Message}");
                    }
                }
            }
        }

        private void NoteSuccess()
        {
            _consecutiveErrors = 0;
        }

        private void NoteError(Exception ex, string where)
        {
            _consecutiveErrors++;
            Update(s => s with { Errors = s.Errors + 1 });
            _log($"[motor {Port}] {where} failed ({_consecutiveErrors}): {ex.Message}");
            if (_consecutiveErrors >= MaxConsecutiveErrors)
            {
                Fail($"Lost contact with board: {ex.Message}");
            }
        }

        private void Fail(string msg)
        {
            Update(s => s with
            {
                Connected = false,
                Streaming = false,
                Connecting = false,
                Error = msg,
                Jogging = null
            });
            _log($"[motor {Port}] ERROR: {msg}");
            _cts.Cancel();
        }

        private void CloseClient()
        {
            if (_client != null)
            {
                try { _client.Disconnect(); } catch { }
                _client = null;
            }
            _rawTransport = null;
        }

        public void Dispose()
        {
            Shutdown();
            CloseClient();
            _wakeEvent.Dispose();
        }
    }
}
