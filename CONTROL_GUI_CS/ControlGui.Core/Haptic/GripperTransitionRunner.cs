using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Threading;
using ControlGui.Core.Motor;
using ControlGui.Core.Paxini;
using Microsoft.Data.Sqlite;

namespace ControlGui.Core.Haptic
{
    /// <summary>
    /// Second experiment: close the gripper slowly and watch for the transition
    /// where the two PaXini sensors start to register force while the ITV valve
    /// ADC channels follow. Samples are streamed to SQLite for offline analysis.
    /// </summary>
    public enum GripperPhase
    {
        Opening,
        SettleOpen,
        Closing,
        Holding,
        MovingOut,
        HoldAtTarget,
        Returning,
        Releasing,
        Done,
        Stopped,
        Error
    }

    public enum GripperMotionMode
    {
        None = 0,
        ZMinusReturn = 1,
        XPlusReturn = 2,
        YPlusReturn = 3
    }

    public record GripperTransitionConfig
    {
        public int OpenPos { get; init; } = MotorConstants.GripperOpen;
        public int ClosePos { get; init; } = MotorConstants.GripperClose;
        public int Speed { get; init; } = 150;
        public int Accel { get; init; } = 20;
        public double SettleOpenSec { get; init; } = 1.5;
        public double HoldCloseSec { get; init; } = 3.0;
        public bool ReleaseAtEnd { get; init; } = true;
        public GripperMotionMode MotionMode { get; init; } = GripperMotionMode.None;
        public double MotionDistanceMm { get; init; } = 10.0;
        public double HoldAtTargetSec { get; init; } = 1.0;
        public double XyVelMmS { get; init; } = 5.0;
        public double XyAccelMmS2 { get; init; } = 50.0;
        public double ZVelMmS { get; init; } = 2.0;
        public double ZAccelMmS2 { get; init; } = 50.0;
        public double SampleRateHz { get; init; } = 50.0;
        public byte SpiCsIdx { get; init; } = 0;
        public byte SpiPeriodMs { get; init; } = 5;
        public double VrefMv { get; init; } = 3300.0;
        public double ForceThresholdN { get; init; } = 0.10;
        public double ValveThresholdMv { get; init; } = 25.0;
        public bool StreamHapticValves { get; init; } = true;
        public string DbPath { get; init; } = "";
    }

    public record GripperTransitionSample(
        double TimeSec,
        double Epoch,
        GripperPhase Phase,
        double PhaseTimeSec,
        int TargetPos,
        int ServoPos,
        double ServoLoadPct,
        double ServoVoltV,
        ushort[] AdcRaw,
        double[] AdcMv,
        SensorReading? Left,
        SensorReading? Right,
        double StepX = double.NaN,
        double StepY = double.NaN,
        double StepZ = double.NaN
    );

    public record GripperTransitionOnsets(
        double? LeftOnsetSec,
        int LeftOnsetPos,
        double LeftBaselineN,
        double LeftOnsetN,
        double? RightOnsetSec,
        int RightOnsetPos,
        double RightBaselineN,
        double RightOnsetN,
        double? ValveOnsetSec,
        int ValveOnsetPos,
        double ValveBaselineMv,
        double ValveOnsetMv
    )
    {
        public string Describe()
        {
            string F(double? v, string unit) => v.HasValue ? $"{v.Value:F2} {unit}" : "---";
            return
                $"L onset {F(LeftOnsetSec, "s")} @ {LeftOnsetPos} (Fmag {LeftBaselineN:F2}→{LeftOnsetN:F2} N)  |  " +
                $"R onset {F(RightOnsetSec, "s")} @ {RightOnsetPos} (Fmag {RightBaselineN:F2}→{RightOnsetN:F2} N)  |  " +
                $"Valve onset {F(ValveOnsetSec, "s")} @ {ValveOnsetPos} ({ValveBaselineMv:F0}→{ValveOnsetMv:F0} mV)";
        }
    }

    public record GripperTransitionStatus(
        bool Running,
        string State,
        string Error,
        string Phase,
        double PhaseTimeSec,
        double ElapsedSec,
        long SampleCount,
        long ReadErrors,
        double ActualRateHz,
        int ServoPos,
        double ServoLoadPct,
        double[] ValveMv,
        double LeftFmag,
        double RightFmag,
        double? LeftOnsetSec,
        double? RightOnsetSec,
        double? ValveOnsetSec,
        string DbPath,
        double StepX = 0,
        double StepY = 0,
        double StepZ = 0
    );

    public sealed class GripperTransitionRunner : IDisposable
    {
        [DllImport("winmm.dll", EntryPoint = "timeBeginPeriod")]
        private static extern uint TimeBeginPeriod(uint uMilliseconds);

        [DllImport("winmm.dll", EntryPoint = "timeEndPeriod")]
        private static extern uint TimeEndPeriod(uint uMilliseconds);

        private const int LiveCapacity = 2000;

        private static readonly string[] SampleColumns =
        {
            "t_s", "epoch_s", "phase", "phase_t_s", "target_pos", "servo_pos",
            "servo_load_pct", "servo_volt_v",
            "step_x_mm", "step_y_mm", "step_z_mm",
            "adc0_raw", "adc1_raw", "adc2_raw", "adc3_raw", "adc4_raw",
            "adc0_mv", "adc1_mv", "adc2_mv", "adc3_mv", "adc4_mv",
            "L_Fx_N", "L_Fy_N", "L_Fz_N", "L_Fmag_N", "L_Fshear_N", "L_mu",
            "L_cop_x_mm", "L_cop_y_mm", "L_shear_dir_deg", "L_tilt_deg",
            "L_peak_taxel", "L_peak_Fz_N", "L_contact_taxels",
            "R_Fx_N", "R_Fy_N", "R_Fz_N", "R_Fmag_N", "R_Fshear_N", "R_mu",
            "R_cop_x_mm", "R_cop_y_mm", "R_shear_dir_deg", "R_tilt_deg",
            "R_peak_taxel", "R_peak_Fz_N", "R_contact_taxels"
        };

        private readonly MotorWorker _motor;
        private readonly GripperTransitionConfig _cfg;
        private readonly Func<SensorReading?>? _getLeft;
        private readonly Func<SensorReading?>? _getRight;
        private readonly Action<string> _log;
        private readonly HapticPayloadSynthesizer _synth = new();
        private readonly TueBinaryEncoder _encoder = new();
        private readonly CancellationTokenSource _cts = new();
        private readonly BlockingCollection<GripperTransitionSample> _dbQueue = new();
        private readonly object _lock = new();

        private readonly List<GripperTransitionSample> _liveRing = new(LiveCapacity);
        private readonly List<GripperTransitionSample> _allSamples = new();

        private Thread? _runThread;
        private Thread? _dbThread;
        private GripperTransitionOnsets? _onsets;
        private GripperTransitionStatus _status;
        private int _lastServoPos;

        public GripperTransitionRunner(
            MotorWorker motor,
            GripperTransitionConfig config,
            Func<SensorReading?>? getLeft = null,
            Func<SensorReading?>? getRight = null,
            Action<string>? log = null)
        {
            _motor = motor ?? throw new ArgumentNullException(nameof(motor));
            _cfg = config ?? throw new ArgumentNullException(nameof(config));
            _getLeft = getLeft;
            _getRight = getRight;
            _log = log ?? (_ => { });

            string dbPath = string.IsNullOrWhiteSpace(_cfg.DbPath)
                ? Path.Combine("recordings", $"gripper_transition_{DateTime.Now:yyyyMMdd_HHmmss}.db")
                : _cfg.DbPath;
            _cfg = _cfg with { DbPath = Path.GetFullPath(dbPath) };

            _status = new GripperTransitionStatus(
                Running: false,
                State: "idle",
                Error: "",
                Phase: GripperPhase.Opening.ToString(),
                PhaseTimeSec: 0,
                ElapsedSec: 0,
                SampleCount: 0,
                ReadErrors: 0,
                ActualRateHz: 0,
                ServoPos: 0,
                ServoLoadPct: 0,
                ValveMv: new double[5],
                LeftFmag: 0,
                RightFmag: 0,
                LeftOnsetSec: null,
                RightOnsetSec: null,
                ValveOnsetSec: null,
                DbPath: _cfg.DbPath);
        }

        public string DbPath => _cfg.DbPath;

        public GripperTransitionStatus GetStatus()
        {
            lock (_lock) { return _status; }
        }

        public GripperTransitionOnsets? GetOnsets()
        {
            lock (_lock) { return _onsets; }
        }

        public List<GripperTransitionSample> GetLiveSamples(int count)
        {
            lock (_lock)
            {
                if (count <= 0 || _liveRing.Count == 0) return new List<GripperTransitionSample>();
                int start = Math.Max(0, _liveRing.Count - count);
                return new List<GripperTransitionSample>(_liveRing.Skip(start));
            }
        }

        public void Start()
        {
            lock (_lock)
            {
                if (_status.Running) return;
            }

            InitDatabase();

            _dbThread = new Thread(DbWriterLoop)
            {
                IsBackground = true,
                Name = "GripperTransition-db"
            };
            _runThread = new Thread(ExperimentLoop)
            {
                IsBackground = true,
                Name = "GripperTransition-run"
            };

            _dbThread.Start();
            _runThread.Start();
        }

        public void Stop()
        {
            _cts.Cancel();
            _runThread?.Join(3000);
        }

        // ------------------------------------------------------------------
        // Experiment state machine
        // ------------------------------------------------------------------

        private sealed class LoopCounters
        {
            public required Stopwatch Sw { get; init; }
            public required Stopwatch RateSw { get; init; }
            public required double Dt { get; init; }
            public long TotalSamples;
            public long ReadErrors;
            public long RateSamples;
            public double ActualHz;
        }

        private void ExperimentLoop()
        {
            TimeBeginPeriod(1);
            bool pollingPaused = false;

            try
            {
                var snap = _motor.Snapshot();
                if (!snap.Connected)
                {
                    SetStatus(s => s with { Running = false, State = "error", Error = "STM32 motor link is not connected" });
                    return;
                }
                if (!snap.ServoOnline)
                {
                    SetStatus(s => s with { Running = false, State = "error", Error = "Gripper servo is offline (scan/ping it first)" });
                    return;
                }

                _motor.SetPollingPaused(true);
                pollingPaused = true;

                try
                {
                    _motor.ConfigureSpiAdcDirect(true, 1, _cfg.SpiCsIdx, _cfg.SpiPeriodMs);
                }
                catch (Exception ex)
                {
                    _log($"[gripper-exp] SPI ADC config warning: {ex.Message}");
                }

                double sampleRate = Math.Clamp(_cfg.SampleRateHz, 5.0, 200.0);
                var st = new LoopCounters
                {
                    Sw = Stopwatch.StartNew(),
                    RateSw = Stopwatch.StartNew(),
                    Dt = 1.0 / sampleRate
                };

                SetStatus(s => s with { Running = true, State = "running", Error = "", Phase = GripperPhase.Opening.ToString() });
                _log($"[gripper-exp] Started: open={_cfg.OpenPos}, close={_cfg.ClosePos}, speed={_cfg.Speed} steps/s, fs={sampleRate:F1} Hz -> {_cfg.DbPath}");

                _lastServoPos = snap.ServoPos;
                double travelSec = Math.Abs(_cfg.ClosePos - _cfg.OpenPos) / Math.Max(1.0, _cfg.Speed);
                double moveTimeout = Math.Max(4.0, travelSec * 2.5 + 2.0);

                // 1. Open fully and let the sensors settle at the baseline.
                _motor.ServoMoveDirect(_cfg.OpenPos, _cfg.Speed, _cfg.Accel);
                RunUntilReached(GripperPhase.Opening, _cfg.OpenPos, moveTimeout, st);
                RunTimedPhase(GripperPhase.SettleOpen, _cfg.OpenPos, _cfg.SettleOpenSec, st);

                // 2. Close at low velocity, sampling the whole transition.
                _motor.ServoMoveDirect(_cfg.ClosePos, _cfg.Speed, _cfg.Accel);
                RunUntilReached(GripperPhase.Closing, _cfg.ClosePos, moveTimeout, st, allowGripStall: true);

                // 3. Hold the closed position so the settle/creep shows up.
                RunTimedPhase(GripperPhase.Holding, _cfg.ClosePos, _cfg.HoldCloseSec, st);

                // 4. Optional post-close stepper excursion (e.g., Z- then return Z, or X+ then return X).
                if (_cfg.MotionMode != GripperMotionMode.None && !_cts.IsCancellationRequested)
                {
                    var origin = _motor.StepGetPosDirect() ?? (_motor.Snapshot().X, _motor.Snapshot().Y, _motor.Snapshot().Z);
                    double x0 = origin.Item1, y0 = origin.Item2, z0 = origin.Item3;
                    var (tx, ty, tz, velMmS, accelMmS2, isZ) = ResolveMotionTarget(x0, y0, z0);

                    _motor.StepEnableDirect(true);
                    ushort feed = (ushort)Math.Clamp(Math.Round(velMmS), 1, 200);
                    ushort accel = (ushort)Math.Clamp(Math.Round(accelMmS2), 10, 2000);
                    double dist = Math.Abs(_cfg.MotionDistanceMm);
                    double stepTimeout = Math.Max(5.0, (dist / Math.Max(0.2, velMmS)) * 2.5 + 3.0);

                    _log($"[gripper-exp] Stepper move ({_cfg.MotionMode}): origin=({x0:F2},{y0:F2},{z0:F2}) -> target=({tx:F2},{ty:F2},{tz:F2}) v={feed} mm/s a={accel} mm/s^2");
                    IssueStepperMove(tx, ty, tz, feed, accel, isZ);
                    RunUntilStepperIdle(GripperPhase.MovingOut, _cfg.ClosePos, tx, ty, tz, stepTimeout, st);

                    if (!_cts.IsCancellationRequested && _cfg.HoldAtTargetSec > 0)
                    {
                        RunTimedPhase(GripperPhase.HoldAtTarget, _cfg.ClosePos, _cfg.HoldAtTargetSec, st);
                    }

                    if (!_cts.IsCancellationRequested)
                    {
                        _log($"[gripper-exp] Stepper return to origin=({x0:F2},{y0:F2},{z0:F2}) v={feed} mm/s a={accel} mm/s^2");
                        IssueStepperMove(x0, y0, z0, feed, accel, isZ);
                        RunUntilStepperIdle(GripperPhase.Returning, _cfg.ClosePos, x0, y0, z0, stepTimeout, st);
                    }
                }

                // 5. Release gripper once back at starting stepper position.
                if (_cfg.ReleaseAtEnd && !_cts.IsCancellationRequested)
                {
                    _motor.ServoMoveDirect(_cfg.OpenPos, _cfg.Speed, _cfg.Accel);
                    RunUntilReached(GripperPhase.Releasing, _cfg.OpenPos, moveTimeout, st);
                }

                var onsets = ComputeOnsets(_allSamples);
                lock (_lock) { _onsets = onsets; }

                bool stopped = _cts.IsCancellationRequested;
                SetStatus(s => s with
                {
                    Running = false,
                    State = stopped ? "stopped" : "completed",
                    Phase = stopped ? GripperPhase.Stopped.ToString() : GripperPhase.Done.ToString(),
                    LeftOnsetSec = onsets.LeftOnsetSec,
                    RightOnsetSec = onsets.RightOnsetSec,
                    ValveOnsetSec = onsets.ValveOnsetSec
                });
                _log($"[gripper-exp] {_status.State}: {st.TotalSamples} samples, {st.ReadErrors} read errors");
                _log($"[gripper-exp] {onsets.Describe()}");
            }
            catch (Exception ex)
            {
                SetStatus(s => s with { Running = false, State = "error", Phase = GripperPhase.Error.ToString(), Error = ex.Message });
                _log($"[gripper-exp] Error: {ex.Message}");
            }
            finally
            {
                if (_cts.IsCancellationRequested && _cfg.MotionMode != GripperMotionMode.None)
                {
                    _motor.StepStopDirect();
                }
                if (_cfg.StreamHapticValves)
                {
                    try { _motor.WriteFrame(_encoder.Encode(new byte[8])); }
                    catch { }
                }
                if (pollingPaused) _motor.SetPollingPaused(false);
                try { _dbQueue.CompleteAdding(); } catch (ObjectDisposedException) { }
                _dbThread?.Join(3000);

                if (_onsets != null || _allSamples.Count > 0)
                {
                    try { WriteSummaryToDb(); }
                    catch (Exception ex) { _log($"[gripper-exp] Summary write failed: {ex.Message}"); }
                }

                TimeEndPeriod(1);
            }
        }

        private (double X, double Y, double Z, double Vel, double Accel, bool IsZ) ResolveMotionTarget(double x0, double y0, double z0)
        {
            double d = Math.Abs(_cfg.MotionDistanceMm);
            return _cfg.MotionMode switch
            {
                GripperMotionMode.ZMinusReturn => (x0, y0, z0 - d, _cfg.ZVelMmS, _cfg.ZAccelMmS2, true),
                GripperMotionMode.XPlusReturn => (x0 + d, y0, z0, _cfg.XyVelMmS, _cfg.XyAccelMmS2, false),
                GripperMotionMode.YPlusReturn => (x0, y0 + d, z0, _cfg.XyVelMmS, _cfg.XyAccelMmS2, false),
                _ => (x0, y0, z0, _cfg.XyVelMmS, _cfg.XyAccelMmS2, false),
            };
        }

        private void IssueStepperMove(double x, double y, double z, ushort feedMmS, ushort accelMmS2, bool isZ)
        {
            if (isZ)
                _motor.StepMoveZDirect(z, feedMmS, accelMmS2);
            else
                _motor.StepMoveXYDirect(x, y, feedMmS, accelMmS2);
        }

        private void RunUntilStepperIdle(
            GripperPhase phase,
            int targetServoPos,
            double targetX,
            double targetY,
            double targetZ,
            double timeoutSec,
            LoopCounters st)
        {
            double start = st.Sw.Elapsed.TotalSeconds;
            int idleStable = 0;
            SamplePhase(phase, targetServoPos, st, s =>
            {
                double elapsed = st.Sw.Elapsed.TotalSeconds - start;
                if (elapsed >= timeoutSec) return true;

                var status = _motor.StepGetStatusDirect();
                bool nearTarget =
                    !double.IsNaN(s.StepX) &&
                    Math.Abs(s.StepX - targetX) <= 0.15 &&
                    Math.Abs(s.StepY - targetY) <= 0.15 &&
                    Math.Abs(s.StepZ - targetZ) <= 0.15;

                if (status != null && !status.IsBusy && (nearTarget || elapsed >= 0.50))
                    idleStable++;
                else
                    idleStable = 0;

                return idleStable >= 2;
            });
        }

        private void RunTimedPhase(GripperPhase phase, int target, double durationSec, LoopCounters st)
        {
            double start = st.Sw.Elapsed.TotalSeconds;
            SamplePhase(phase, target, st, s =>
                (st.Sw.Elapsed.TotalSeconds - start) >= durationSec);
        }

        private void RunUntilReached(GripperPhase phase, int target, double timeoutSec, LoopCounters st, bool allowGripStall = false)
        {
            double start = st.Sw.Elapsed.TotalSeconds;
            int stable = 0;
            int stallStable = 0;
            int prevPos = _lastServoPos;
            SamplePhase(phase, target, st, s =>
            {
                double elapsed = st.Sw.Elapsed.TotalSeconds - start;
                if (Math.Abs(s.ServoPos - target) <= 6) stable++;
                else stable = 0;

                if (allowGripStall && elapsed >= 0.6)
                {
                    bool stoppedMoving = Math.Abs(s.ServoPos - prevPos) <= 1;
                    bool hasContactOrLoad =
                        (s.Left?.FMag ?? 0) >= _cfg.ForceThresholdN ||
                        (s.Right?.FMag ?? 0) >= _cfg.ForceThresholdN ||
                        (!double.IsNaN(s.ServoLoadPct) && Math.Abs(s.ServoLoadPct) >= 40.0);
                    if (stoppedMoving && hasContactOrLoad) stallStable++;
                    else stallStable = 0;
                }
                prevPos = s.ServoPos;

                int stallThreshold = Math.Max(10, (int)Math.Round(0.5 * Math.Clamp(_cfg.SampleRateHz, 5, 200)));
                return stable >= 2 || (allowGripStall && stallStable >= stallThreshold) || elapsed >= timeoutSec;
            });
        }

        private void SamplePhase(
            GripperPhase phase,
            int target,
            LoopCounters st,
            Func<GripperTransitionSample, bool> exit)
        {
            double phaseStart = st.Sw.Elapsed.TotalSeconds;
            double nextTick = phaseStart;
            int uiDivider = 0;

            while (!_cts.IsCancellationRequested)
            {
                double phaseT = st.Sw.Elapsed.TotalSeconds - phaseStart;
                var sample = Sample(phase, phaseT, target, st);

                if ((uiDivider++ % 5) == 0) PublishStatus(sample, phaseT, st);

                if (exit(sample)) break;

                nextTick += st.Dt;
                double remaining = nextTick - st.Sw.Elapsed.TotalSeconds;
                if (remaining > 0.002)
                {
                    Thread.Sleep((int)((remaining - 0.001) * 1000.0));
                }
                else if (remaining < -0.10)
                {
                    nextTick = st.Sw.Elapsed.TotalSeconds;
                }
                while (st.Sw.Elapsed.TotalSeconds < nextTick && !_cts.IsCancellationRequested)
                {
                    Thread.SpinWait(20);
                }
            }
        }

        private GripperTransitionSample Sample(GripperPhase phase, double phaseT, int target, LoopCounters st)
        {
            double t = st.Sw.Elapsed.TotalSeconds;
            double epoch = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0;

            ushort[] raw = new ushort[5];
            double[] mv = new double[5];
            try
            {
                var res = _motor.ReadSpiAdcChannelsDirect(_cfg.SpiCsIdx, 0.04);
                if (res.HasValue)
                {
                    raw = res.Value.Channels;
                    for (int c = 0; c < 5; c++)
                        mv[c] = raw[c] * _cfg.VrefMv / 4095.0;
                }
                else
                {
                    st.ReadErrors++;
                    Array.Fill(mv, double.NaN);
                }
            }
            catch
            {
                st.ReadErrors++;
                Array.Fill(mv, double.NaN);
            }

            var fb = ServoFeedbackSafe();
            if (fb != null) _lastServoPos = fb.Dynamics.Position;
            int servoPos = fb?.Dynamics.Position ?? _lastServoPos;
            double loadPct = fb?.Dynamics.LoadPercent ?? double.NaN;
            double voltV = fb?.Health.VoltageV ?? double.NaN;

            double stepX = double.NaN, stepY = double.NaN, stepZ = double.NaN;
            if (_cfg.MotionMode != GripperMotionMode.None)
            {
                var stepPos = _motor.StepGetPosDirect();
                if (stepPos.HasValue)
                {
                    stepX = stepPos.Value.X;
                    stepY = stepPos.Value.Y;
                    stepZ = stepPos.Value.Z;
                }
            }

            var left = SafeRead(_getLeft);
            var right = SafeRead(_getRight);

            if (_cfg.StreamHapticValves)
            {
                double nowForSynth = ((left?.Epoch ?? 0.0) >= 1_000_000_000.0 || (right?.Epoch ?? 0.0) >= 1_000_000_000.0)
                    ? epoch
                    : Math.Max(left?.T ?? 0.0, right?.T ?? 0.0);
                byte[] payload = _synth.Synthesize(left, right, slipAlert: false, tareActive: false, nowSec: nowForSynth);
                _motor.WriteFrame(_encoder.Encode(payload));
            }

            var sample = new GripperTransitionSample(
                t, epoch, phase, phaseT, target,
                servoPos, loadPct, voltV,
                raw, mv,
                left,
                right,
                stepX, stepY, stepZ);

            if (!_dbQueue.IsAddingCompleted)
            {
                try { _dbQueue.Add(sample); } catch (InvalidOperationException) { }
            }

            lock (_lock)
            {
                if (_liveRing.Count >= LiveCapacity) _liveRing.RemoveAt(0);
                _liveRing.Add(sample);
            }
            _allSamples.Add(sample);

            st.TotalSamples++;
            st.RateSamples++;
            if (st.RateSw.ElapsedMilliseconds >= 500)
            {
                st.ActualHz = st.RateSamples / (st.RateSw.ElapsedMilliseconds / 1000.0);
                st.RateSamples = 0;
                st.RateSw.Restart();
            }

            return sample;
        }

        private ControlGui.Core.TueUsb.ServoFeedback? ServoFeedbackSafe()
        {
            try { return _motor.ServoFeedbackDirect(); }
            catch { return null; }
        }

        private static SensorReading? SafeRead(Func<SensorReading?>? getter)
        {
            if (getter == null) return null;
            try { return getter(); }
            catch { return null; }
        }

        private void PublishStatus(GripperTransitionSample sample, double phaseT, LoopCounters st)
        {
            SetStatus(s => s with
            {
                Phase = sample.Phase.ToString(),
                PhaseTimeSec = phaseT,
                ElapsedSec = sample.TimeSec,
                SampleCount = st.TotalSamples,
                ReadErrors = st.ReadErrors,
                ActualRateHz = st.ActualHz,
                ServoPos = sample.ServoPos,
                ServoLoadPct = double.IsNaN(sample.ServoLoadPct) ? 0 : sample.ServoLoadPct,
                ValveMv = (double[])sample.AdcMv.Clone(),
                LeftFmag = sample.Left?.FMag ?? 0,
                RightFmag = sample.Right?.FMag ?? 0,
                StepX = double.IsNaN(sample.StepX) ? s.StepX : sample.StepX,
                StepY = double.IsNaN(sample.StepY) ? s.StepY : sample.StepY,
                StepZ = double.IsNaN(sample.StepZ) ? s.StepZ : sample.StepZ
            });
        }

        private void SetStatus(Func<GripperTransitionStatus, GripperTransitionStatus> fn)
        {
            lock (_lock) { _status = fn(_status); }
        }

        // ------------------------------------------------------------------
        // Transition detection
        // ------------------------------------------------------------------

        private GripperTransitionOnsets ComputeOnsets(List<GripperTransitionSample> samples)
        {
            var settle = samples.Where(s => s.Phase == GripperPhase.SettleOpen).ToList();
            if (settle.Count == 0) settle = samples.Take(Math.Min(10, samples.Count)).ToList();

            double lBase = MeanOf(settle, s => s.Left?.FMag);
            double rBase = MeanOf(settle, s => s.Right?.FMag);
            var vBase = new double[5];
            for (int c = 0; c < 5; c++)
            {
                int ch = c;
                vBase[c] = MeanOf(settle, s => s.AdcMv != null && ch < s.AdcMv.Length ? s.AdcMv[ch] : (double?)null);
            }

            var search = samples
                .Where(s => s.Phase is GripperPhase.Closing or GripperPhase.Holding or GripperPhase.Releasing)
                .ToList();

            var (lSec, lPos, lVal) = FindOnset(search, s => s.Left?.FMag, lBase + _cfg.ForceThresholdN);
            var (rSec, rPos, rVal) = FindOnset(search, s => s.Right?.FMag, rBase + _cfg.ForceThresholdN);

            double? vSec = null;
            int vPos = 0;
            double vVal = 0;
            double vBaseMean = vBase.Where(v => !double.IsNaN(v)).DefaultIfEmpty(0).Average();
            for (int c = 0; c < 5; c++)
            {
                int ch = c;
                var (sec, pos, val) = FindOnset(
                    search,
                    s => s.AdcMv != null && ch < s.AdcMv.Length ? s.AdcMv[ch] : (double?)null,
                    vBase[ch] + _cfg.ValveThresholdMv);
                if (sec.HasValue && (!vSec.HasValue || sec.Value < vSec.Value))
                {
                    vSec = sec;
                    vPos = pos;
                    vVal = val;
                }
            }

            return new GripperTransitionOnsets(
                lSec, lPos, lBase, lVal,
                rSec, rPos, rBase, rVal,
                vSec, vPos, vBaseMean, vVal);
        }

        private static double MeanOf(List<GripperTransitionSample> samples, Func<GripperTransitionSample, double?> selector)
        {
            var vals = samples.Select(selector).Where(v => v.HasValue && !double.IsNaN(v.Value)).Select(v => v!.Value).ToList();
            return vals.Count == 0 ? 0.0 : vals.Average();
        }

        private static (double? Sec, int Pos, double Value) FindOnset(
            List<GripperTransitionSample> samples,
            Func<GripperTransitionSample, double?> selector,
            double threshold)
        {
            int consec = 0;
            foreach (var s in samples)
            {
                double? v = selector(s);
                if (v.HasValue && !double.IsNaN(v.Value) && v.Value >= threshold)
                {
                    consec++;
                    if (consec >= 2) return (s.TimeSec, s.ServoPos, v.Value);
                }
                else
                {
                    consec = 0;
                }
            }
            return (null, 0, 0);
        }

        // ------------------------------------------------------------------
        // SQLite
        // ------------------------------------------------------------------

        private void InitDatabase()
        {
            Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(_cfg.DbPath))!);

            using var conn = new SqliteConnection($"Data Source={_cfg.DbPath}");
            conn.Open();

            string colDefs = string.Join(",\n", SampleColumns.Select(c =>
                c == "phase" ? "    [phase] TEXT" :
                (c.EndsWith("_raw", StringComparison.Ordinal) ? $"    [{c}] INTEGER" : $"    [{c}] REAL")));

            using var cmd = conn.CreateCommand();
            cmd.CommandText = $@"
                PRAGMA journal_mode=WAL;
                PRAGMA synchronous=NORMAL;
                CREATE TABLE IF NOT EXISTS experiment_meta (
                    key TEXT PRIMARY KEY,
                    value TEXT
                );
                CREATE TABLE IF NOT EXISTS samples (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
{colDefs}
                );
                CREATE TABLE IF NOT EXISTS transition_summary (
                    key TEXT PRIMARY KEY,
                    value REAL
                );";
            cmd.ExecuteNonQuery();

            var meta = new Dictionary<string, string>
            {
                ["created_iso"] = DateTime.UtcNow.ToString("O"),
                ["experiment"] = "gripper_closure_transition",
                ["open_pos"] = _cfg.OpenPos.ToString(CultureInfo.InvariantCulture),
                ["close_pos"] = _cfg.ClosePos.ToString(CultureInfo.InvariantCulture),
                ["speed_steps_s"] = _cfg.Speed.ToString(CultureInfo.InvariantCulture),
                ["accel"] = _cfg.Accel.ToString(CultureInfo.InvariantCulture),
                ["settle_open_sec"] = _cfg.SettleOpenSec.ToString(CultureInfo.InvariantCulture),
                ["hold_close_sec"] = _cfg.HoldCloseSec.ToString(CultureInfo.InvariantCulture),
                ["release_at_end"] = _cfg.ReleaseAtEnd ? "1" : "0",
                ["motion_mode"] = _cfg.MotionMode.ToString(),
                ["motion_distance_mm"] = _cfg.MotionDistanceMm.ToString(CultureInfo.InvariantCulture),
                ["hold_at_target_sec"] = _cfg.HoldAtTargetSec.ToString(CultureInfo.InvariantCulture),
                ["xy_vel_mm_s"] = _cfg.XyVelMmS.ToString(CultureInfo.InvariantCulture),
                ["xy_accel_mm_s2"] = _cfg.XyAccelMmS2.ToString(CultureInfo.InvariantCulture),
                ["z_vel_mm_s"] = _cfg.ZVelMmS.ToString(CultureInfo.InvariantCulture),
                ["z_accel_mm_s2"] = _cfg.ZAccelMmS2.ToString(CultureInfo.InvariantCulture),
                ["sample_rate_hz"] = _cfg.SampleRateHz.ToString(CultureInfo.InvariantCulture),
                ["spi_bus_id"] = "1",
                ["spi_cs_idx"] = _cfg.SpiCsIdx.ToString(CultureInfo.InvariantCulture),
                ["spi_period_ms"] = _cfg.SpiPeriodMs.ToString(CultureInfo.InvariantCulture),
                ["vref_mv"] = _cfg.VrefMv.ToString(CultureInfo.InvariantCulture),
                ["force_threshold_n"] = _cfg.ForceThresholdN.ToString(CultureInfo.InvariantCulture),
                ["valve_threshold_mv"] = _cfg.ValveThresholdMv.ToString(CultureInfo.InvariantCulture)
            };

            using var tx = conn.BeginTransaction();
            foreach (var kv in meta)
            {
                using var ins = conn.CreateCommand();
                ins.Transaction = tx;
                ins.CommandText = "INSERT OR REPLACE INTO experiment_meta (key, value) VALUES ($k, $v)";
                ins.Parameters.AddWithValue("$k", kv.Key);
                ins.Parameters.AddWithValue("$v", kv.Value);
                ins.ExecuteNonQuery();
            }
            tx.Commit();
        }

        private void DbWriterLoop()
        {
            using var conn = new SqliteConnection($"Data Source={_cfg.DbPath}");
            conn.Open();

            string cols = string.Join(", ", SampleColumns.Select(c => $"[{c}]"));
            string pars = string.Join(", ", SampleColumns.Select((_, i) => $"$p{i}"));

            using var ins = conn.CreateCommand();
            ins.CommandText = $"INSERT INTO samples ({cols}) VALUES ({pars});";
            var parameters = new SqliteParameter[SampleColumns.Length];
            for (int i = 0; i < SampleColumns.Length; i++)
            {
                parameters[i] = ins.Parameters.Add($"$p{i}", SampleColumns[i] == "phase" ? SqliteType.Text : SqliteType.Real);
            }

            var batch = new List<GripperTransitionSample>(64);
            while (!_dbQueue.IsCompleted)
            {
                if (_dbQueue.TryTake(out var first, 50))
                {
                    batch.Add(first);
                    while (batch.Count < 64 && _dbQueue.TryTake(out var more))
                        batch.Add(more);

                    using var tx = conn.BeginTransaction();
                    ins.Transaction = tx;
                    foreach (var s in batch)
                    {
                        BindRow(parameters, s);
                        ins.ExecuteNonQuery();
                    }
                    tx.Commit();
                    batch.Clear();
                }
            }
        }

        private static void BindRow(SqliteParameter[] p, GripperTransitionSample s)
        {
            p[0].Value = s.TimeSec;
            p[1].Value = s.Epoch;
            p[2].Value = s.Phase.ToString();
            p[3].Value = s.PhaseTimeSec;
            p[4].Value = s.TargetPos;
            p[5].Value = s.ServoPos;
            p[6].Value = Db(s.ServoLoadPct);
            p[7].Value = Db(s.ServoVoltV);
            p[8].Value = Db(s.StepX);
            p[9].Value = Db(s.StepY);
            p[10].Value = Db(s.StepZ);

            for (int c = 0; c < 5; c++)
                p[11 + c].Value = s.AdcRaw != null && c < s.AdcRaw.Length ? (int)s.AdcRaw[c] : DBNull.Value;
            for (int c = 0; c < 5; c++)
                p[16 + c].Value = Db(s.AdcMv != null && c < s.AdcMv.Length ? s.AdcMv[c] : double.NaN);

            int i = 21;
            i = BindSensor(p, i, s.Left);
            BindSensor(p, i, s.Right);
        }

        private static int BindSensor(SqliteParameter[] p, int i, SensorReading? r)
        {
            if (r == null)
            {
                for (int k = 0; k < 13; k++) p[i + k].Value = DBNull.Value;
                return i + 13;
            }

            p[i + 0].Value = Db(r.Fx);
            p[i + 1].Value = Db(r.Fy);
            p[i + 2].Value = Db(r.Fz);
            p[i + 3].Value = Db(r.FMag);
            p[i + 4].Value = Db(r.FShear);
            p[i + 5].Value = Db(r.Mu);
            p[i + 6].Value = Db(r.CopX);
            p[i + 7].Value = Db(r.CopY);
            p[i + 8].Value = Db(r.ShearDir);
            p[i + 9].Value = Db(r.Tilt);
            p[i + 10].Value = Db(r.PeakTaxel);
            p[i + 11].Value = Db(r.PeakFz);
            p[i + 12].Value = Db(r.ContactTaxels);
            return i + 13;
        }

        private static object Db(double v) => double.IsNaN(v) || double.IsInfinity(v) ? DBNull.Value : v;

        private void WriteSummaryToDb()
        {
            var onsets = _onsets;
            if (onsets == null) return;

            using var conn = new SqliteConnection($"Data Source={_cfg.DbPath}");
            conn.Open();
            using var tx = conn.BeginTransaction();

            void Put(string key, double? value)
            {
                using var cmd = conn.CreateCommand();
                cmd.Transaction = tx;
                cmd.CommandText = "INSERT OR REPLACE INTO transition_summary (key, value) VALUES ($k, $v)";
                cmd.Parameters.AddWithValue("$k", key);
                cmd.Parameters.AddWithValue("$v", value.HasValue && !double.IsNaN(value.Value) ? value.Value : DBNull.Value);
                cmd.ExecuteNonQuery();
            }

            Put("samples", _allSamples.Count);
            Put("L_onset_s", onsets.LeftOnsetSec);
            Put("L_onset_pos", onsets.LeftOnsetPos);
            Put("L_baseline_fmag_n", onsets.LeftBaselineN);
            Put("L_onset_fmag_n", onsets.LeftOnsetN);
            Put("R_onset_s", onsets.RightOnsetSec);
            Put("R_onset_pos", onsets.RightOnsetPos);
            Put("R_baseline_fmag_n", onsets.RightBaselineN);
            Put("R_onset_fmag_n", onsets.RightOnsetN);
            Put("valve_onset_s", onsets.ValveOnsetSec);
            Put("valve_onset_pos", onsets.ValveOnsetPos);
            Put("valve_baseline_mv", onsets.ValveBaselineMv);
            Put("valve_onset_mv", onsets.ValveOnsetMv);

            tx.Commit();
        }

        public void Dispose()
        {
            Stop();
            _dbQueue.Dispose();
            _cts.Dispose();
        }
    }
}
