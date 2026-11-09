using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Threading;
using ControlGui.Core.Paxini;
using ControlGui.Core.TueUsb;

namespace ControlGui.Core.Haptic
{
    public record HapticValve(
        string Name,
        string Label,
        double X,
        double Y,
        double Force,
        double Weight,
        byte Raw,
        double T,
        double Age,
        double Voltage = 0.0
    );

    public record HapticSnapshot(
        string State,
        bool Connected,
        string Error,
        byte Seq,
        double T,
        double Age,
        byte Flags,
        byte[] Frame,
        HapticValve[] Valves,
        long Sent,
        long Errors,
        double RateHz
    )
    {
        public static HapticSnapshot Empty => new(
            "idle", false, "", 0, 0, 0, 0, Array.Empty<byte>(), Array.Empty<HapticValve>(), 0, 0, 0);

        public string FrameHex => Frame.Length > 0 ? BitConverter.ToString(Frame).Replace("-", " ") : "---";

        public List<string> FlagNames
        {
            get
            {
                var list = new List<string>();
                if ((Flags & HapticConstants.FlagLeftContact) != 0) list.Add("L-contact");
                if ((Flags & HapticConstants.FlagRightContact) != 0) list.Add("R-contact");
                if ((Flags & HapticConstants.FlagSlip) != 0) list.Add("slip");
                if ((Flags & HapticConstants.FlagTare) != 0) list.Add("tare");
                return list;
            }
        }

        public static HapticSnapshot FromPayload(
            byte[] payload,
            double t,
            double age = 0.0,
            byte seq = 0,
            byte[]? frame = null,
            string state = "streaming",
            bool connected = true,
            string error = "",
            long sent = 0,
            long errors = 0,
            double rateHz = 0.0)
        {
            byte[] p = new byte[8];
            if (payload != null)
            {
                Array.Copy(payload, 0, p, 0, Math.Min(payload.Length, 8));
            }

            var valves = new HapticValve[5];
            for (int i = 0; i < 5; i++)
            {
                var item = HapticConstants.ValveLayout[i];
                byte rawVal = p[i];
                double forceVal = rawVal * HapticConstants.ForceLsbN;
                double weightVal = rawVal / 255.0;
                double voltVal = HapticConstants.ComputeValveVoltage(forceVal);
                valves[i] = new HapticValve(item.Name, item.Label, item.X, item.Y, forceVal, weightVal, rawVal, t, age, voltVal);
            }

            return new HapticSnapshot(
                state, connected, error, seq, t, age, p[6], frame ?? Array.Empty<byte>(), valves, sent, errors, rateHz
            );
        }
    }

    public class Stm32HapticStreamer : IDisposable
    {
        [DllImport("winmm.dll", EntryPoint = "timeBeginPeriod")]
        private static extern uint TimeBeginPeriod(uint uMilliseconds);

        [DllImport("winmm.dll", EntryPoint = "timeEndPeriod")]
        private static extern uint TimeEndPeriod(uint uMilliseconds);

        private readonly Thread _thread;
        private readonly CancellationTokenSource _cts = new();
        private readonly Func<byte[], int>? _writer;
        private readonly Action<string> _log;
        private readonly HapticPayloadSynthesizer _synth = new();
        private readonly TueBinaryEncoder _encoder = new();

        private readonly object _lock = new object();
        private SensorReading? _latestLeft;
        private SensorReading? _latestRight;
        private bool _slipAlert = false;
        private bool _tareActive = false;

        private HapticSnapshot _snapshot = HapticSnapshot.Empty;
        private long _sent = 0;
        private long _errors = 0;
        private double _rateHz = 0.0;

        public Stm32HapticStreamer(
            Func<byte[], int>? writer,
            Action<string> log)
        {
            _writer = writer;
            _log = log;

            _thread = new Thread(Run)
            {
                IsBackground = true,
                Name = "Stm32HapticStreamer"
            };
        }

        public void Start()
        {
            _thread.Start();
        }

        public void Stop()
        {
            _cts.Cancel();
        }

        public void UpdateSensors(SensorReading? left, SensorReading? right, bool slipAlert = false, bool tareActive = false)
        {
            lock (_lock)
            {
                _latestLeft = left;
                _latestRight = right;
                _slipAlert = slipAlert;
                _tareActive = tareActive;
            }
        }

        public HapticSnapshot GetSnapshot()
        {
            lock (_lock)
            {
                return _snapshot;
            }
        }

        private void Run()
        {
            bool timerRaised = false;
            if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
            {
                try
                {
                    timerRaised = TimeBeginPeriod(1) == 0;
                }
                catch { }
            }

            _log("[haptic] 100 Hz USB CDC streaming thread started");
            var rateSw = Stopwatch.StartNew();
            int rateCount = 0;

            var loopSw = Stopwatch.StartNew();
            long periodTicks = (long)(Stopwatch.Frequency * HapticConstants.PeriodSec);
            long nextTick = loopSw.ElapsedTicks;

            while (!_cts.Token.IsCancellationRequested)
            {
                long currentTicks = loopSw.ElapsedTicks;
                if (currentTicks < nextTick)
                {
                    long remainingMs = (nextTick - currentTicks) * 1000 / Stopwatch.Frequency;
                    if (remainingMs > 1)
                    {
                        Thread.Sleep((int)(remainingMs - 1));
                    }
                    while (loopSw.ElapsedTicks < nextTick)
                    {
                        Thread.SpinWait(10);
                    }
                }
                nextTick += periodTicks;

                SensorReading? left;
                SensorReading? right;
                bool slip;
                bool tare;

                lock (_lock)
                {
                    left = _latestLeft;
                    right = _latestRight;
                    slip = _slipAlert;
                    tare = _tareActive;
                }

                double nowSec = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0;
                byte[] payload = _synth.Synthesize(left, right, slip, tare, nowSec);
                byte[] wireFrame = _encoder.Encode(payload);

                bool writeOk = false;
                if (_writer != null)
                {
                    try
                    {
                        int written = _writer(wireFrame);
                        if (written == wireFrame.Length)
                        {
                            writeOk = true;
                            _sent++;
                        }
                        else
                        {
                            _errors++;
                        }
                    }
                    catch
                    {
                        _errors++;
                    }
                }
                else
                {
                    // Preview mode without physical writer
                    writeOk = true;
                    _sent++;
                }

                rateCount++;
                if (rateSw.ElapsedMilliseconds >= 1000)
                {
                    _rateHz = rateCount / (rateSw.ElapsedMilliseconds / 1000.0);
                    rateCount = 0;
                    rateSw.Restart();
                }

                var snap = HapticSnapshot.FromPayload(
                    payload,
                    nowSec,
                    age: 0,
                    seq: _encoder.Seq,
                    frame: wireFrame,
                    state: writeOk ? "streaming" : "error",
                    connected: _writer != null,
                    sent: _sent,
                    errors: _errors,
                    rateHz: _rateHz
                );

                lock (_lock)
                {
                    _snapshot = snap;
                }
            }

            if (timerRaised)
            {
                try
                {
                    TimeEndPeriod(1);
                }
                catch { }
            }

            _log("[haptic] streaming thread stopped");
        }

        public void Dispose()
        {
            Stop();
        }
    }
}
