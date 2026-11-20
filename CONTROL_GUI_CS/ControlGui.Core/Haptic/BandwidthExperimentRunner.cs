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
using Microsoft.Data.Sqlite;

namespace ControlGui.Core.Haptic
{
    public enum BandwidthSweepMode
    {
        SteppedSine = 0,
        Chirp = 1
    }

    public enum SteppedTimingMode
    {
        FixedCycles = 0,
        TotalDurationSec = 1
    }

    public record SteppedScheduleItem(
        int StepIdx,
        double FreqHz,
        double SettleSec,
        double MeasureSec
    );

    public record BandwidthExperimentConfig
    {
        public int ChannelIndex { get; init; } = 0; // 0..4 (A..E), or -1 for all 5
        public BandwidthSweepMode SweepMode { get; init; } = BandwidthSweepMode.SteppedSine;
        public SteppedTimingMode TimingMode { get; init; } = SteppedTimingMode.TotalDurationSec;
        public double TotalDurationSec { get; init; } = 90.0;
        public double FreqMinHz { get; init; } = 0.1;
        public double FreqMaxHz { get; init; } = 10.0;
        public int FreqPoints { get; init; } = 25;
        public bool FreqDescending { get; init; } = false;
        public double ChirpSettleSec { get; init; } = 1.0;
        public double U0Percent { get; init; } = 50.0;
        public double UaPercent { get; init; } = 25.0;
        public double[] FrequenciesHz { get; init; } = BandwidthExperimentRunner.BuildLogSpacedFrequencies(0.1, 10.0, 25, false);
        public int CyclesPerFreq { get; init; } = 10;
        public int SettleCycles { get; init; } = 2;
        public double SampleRateHz { get; init; } = 100.0;
        public byte SpiBusId { get; init; } = 1;
        public byte SpiCsIdx { get; init; } = 0;
        public byte SpiPeriodMs { get; init; } = 5;
        public double VrefMv { get; init; } = 3300.0;
        public bool ZeroInactiveChannels { get; init; } = true;
        public string DbPath { get; init; } = "";
    }

    public record BandwidthSample(
        double TimeSec,
        double FreqHz,
        int StepIdx,
        bool IsSettling,
        double UPct,
        byte UByte,
        ushort[] Raw,
        double[] Mv
    );

    public record BodePoint(
        int StepIdx,
        double FreqHz,
        int NSamples,
        double AmpMv,
        double PhaseDeg,
        double MagDb,
        double DcOffsetMv
    );

    public record BandwidthExperimentStatus(
        bool Running,
        string State,
        string Error,
        int CurrentStep,
        int TotalSteps,
        double CurrentFreqHz,
        double Progress01,
        long SampleCount,
        long ReadErrors,
        double ActualRateHz,
        double? BandwidthFcHz,
        string DbPath
    );

    public sealed class BandwidthExperimentRunner : IDisposable
    {
        [DllImport("winmm.dll", EntryPoint = "timeBeginPeriod")]
        private static extern uint TimeBeginPeriod(uint uMilliseconds);

        [DllImport("winmm.dll", EntryPoint = "timeEndPeriod")]
        private static extern uint TimeEndPeriod(uint uMilliseconds);

        private const int LiveCapacity = 2000;

        private readonly MotorWorker _motor;
        private readonly BandwidthExperimentConfig _cfg;
        private readonly Action<string> _log;
        private readonly TueBinaryEncoder _encoder = new();
        private readonly CancellationTokenSource _cts = new();
        private readonly BlockingCollection<BandwidthSample> _dbQueue = new();
        private readonly object _lock = new();

        private readonly List<BandwidthSample> _liveRing = new(LiveCapacity);
        private readonly List<BodePoint> _bodePoints = new();

        private Thread? _runThread;
        private Thread? _dbThread;
        private volatile bool _bodeDirty = false;
        private BandwidthExperimentStatus _status;

        public BandwidthExperimentRunner(
            MotorWorker motor,
            BandwidthExperimentConfig config,
            Action<string>? log = null)
        {
            _motor = motor ?? throw new ArgumentNullException(nameof(motor));
            _cfg = config ?? throw new ArgumentNullException(nameof(config));
            _log = log ?? (_ => { });

            string dbPath = string.IsNullOrWhiteSpace(_cfg.DbPath)
                ? Path.Combine("recordings", $"bandwidth_{DateTime.Now:yyyyMMdd_HHmmss}.db")
                : _cfg.DbPath;
            _cfg = _cfg with { DbPath = Path.GetFullPath(dbPath) };

            int initSteps = _cfg.SweepMode == BandwidthSweepMode.Chirp
                ? Math.Clamp(_cfg.FreqPoints, 10, 120)
                : _cfg.FrequenciesHz.Length;

            _status = new BandwidthExperimentStatus(
                Running: false,
                State: "idle",
                Error: "",
                CurrentStep: 0,
                TotalSteps: initSteps,
                CurrentFreqHz: 0,
                Progress01: 0,
                SampleCount: 0,
                ReadErrors: 0,
                ActualRateHz: 0,
                BandwidthFcHz: null,
                DbPath: _cfg.DbPath);
        }

        public void Start()
        {
            if (_runThread != null) return;

            Directory.CreateDirectory(Path.GetDirectoryName(_cfg.DbPath)!);
            InitDatabase();

            _dbThread = new Thread(DbWriterLoop)
            {
                IsBackground = true,
                Name = "BandwidthDbWriter"
            };
            _dbThread.Start();

            _runThread = new Thread(ExperimentLoop)
            {
                IsBackground = true,
                Name = "BandwidthExperimentRunner",
                Priority = ThreadPriority.AboveNormal
            };
            _runThread.Start();
        }

        public void Stop()
        {
            _cts.Cancel();
        }

        public BandwidthExperimentStatus GetStatus()
        {
            lock (_lock) return _status;
        }

        public List<BandwidthSample> GetLiveSamples(int maxCount = 600)
        {
            lock (_lock)
            {
                if (_liveRing.Count <= maxCount)
                    return new List<BandwidthSample>(_liveRing);
                return _liveRing.GetRange(_liveRing.Count - maxCount, maxCount);
            }
        }

        public List<BodePoint> GetBodePoints()
        {
            lock (_lock) return new List<BodePoint>(_bodePoints);
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

                // Pause normal 15 Hz polling so the experiment owns the USB bus cleanly
                _motor.SetPollingPaused(true);
                pollingPaused = true;

                try
                {
                    _motor.ConfigureSpiAdcDirect(true, _cfg.SpiBusId, _cfg.SpiCsIdx, _cfg.SpiPeriodMs);
                }
                catch (Exception ex)
                {
                    _log($"[bandwidth] SPI config warning: {ex.Message}");
                }

                double sampleRate = Math.Clamp(_cfg.SampleRateHz, 10.0, 250.0);
                double dt = 1.0 / sampleRate;
                int activeCh = _cfg.ChannelIndex;
                int measureCh = activeCh >= 0 && activeCh < 5 ? activeCh : 0;

                var state = new LoopCounters
                {
                    GlobalSw = Stopwatch.StartNew(),
                    RateSw = Stopwatch.StartNew()
                };

                if (_cfg.SweepMode == BandwidthSweepMode.Chirp)
                    RunChirpLoop(sampleRate, dt, activeCh, measureCh, state);
                else
                    RunSteppedSineLoop(sampleRate, dt, activeCh, measureCh, state);

                // Return valves safely to 0% when finished
                byte[] zeroPayload = new byte[8];
                _motor.WriteFrame(_encoder.Encode(zeroPayload));

                double? fc;
                lock (_lock)
                {
                    fc = ComputeBandwidthFcHz(_bodePoints);
                }

                string finalState = _cts.IsCancellationRequested ? "stopped" : "completed";
                SetStatus(s => s with
                {
                    Running = false,
                    State = finalState,
                    Progress01 = _cts.IsCancellationRequested ? s.Progress01 : 1.0,
                    BandwidthFcHz = fc
                });
                _log($"[bandwidth] {finalState}: {state.TotalSamples} samples, {state.ReadErrors} read errors, fc={(fc.HasValue ? $"{fc.Value:F2} Hz" : "N/A")}");
            }
            catch (Exception ex)
            {
                SetStatus(s => s with { Running = false, State = "error", Error = ex.Message });
                _log($"[bandwidth] Error: {ex.Message}");
            }
            finally
            {
                if (pollingPaused)
                    _motor.SetPollingPaused(false);
                _dbQueue.CompleteAdding();
                _dbThread?.Join(2000);
                TimeEndPeriod(1);
            }
        }

        private sealed class LoopCounters
        {
            public required Stopwatch GlobalSw { get; init; }
            public required Stopwatch RateSw { get; init; }
            public long TotalSamples;
            public long ReadErrors;
            public long RateSamples;
            public double ActualHz;
        }

        private void RunSteppedSineLoop(double sampleRate, double dt, int activeCh, int measureCh, LoopCounters st)
        {
            var schedule = BuildSteppedSchedule(_cfg);
            int totalSteps = schedule.Count;
            double totalExpectedSec = Math.Max(0.1, schedule.Sum(s => s.SettleSec + s.MeasureSec));

            SetStatus(s => s with { Running = true, State = "running", Error = "", TotalSteps = totalSteps });
            _log($"[bandwidth] Started Stepped-Sine ({totalSteps} steps, est {totalExpectedSec:F1} s, fs={sampleRate:F1} Hz) -> {_cfg.DbPath}");

            // Carry phase continuously across frequency transitions so u(t) never jumps discontinuously
            double phaseRad = 0.0;
            double nextTickGlobal = st.GlobalSw.Elapsed.TotalSeconds;

            foreach (var item in schedule)
            {
                if (_cts.IsCancellationRequested) break;

                double f = item.FreqHz;
                double stepDur = item.SettleSec + item.MeasureSec;
                var stepTRel = new List<double>((int)(stepDur * sampleRate) + 16);
                var stepY = new List<double>((int)(stepDur * sampleRate) + 16);

                double stepStartGlobal = st.GlobalSw.Elapsed.TotalSeconds;
                if (nextTickGlobal < stepStartGlobal) nextTickGlobal = stepStartGlobal;
                double stepPhaseOffset = phaseRad;

                while (st.GlobalSw.Elapsed.TotalSeconds - stepStartGlobal < stepDur && !_cts.IsCancellationRequested)
                {
                    double tGlobal = st.GlobalSw.Elapsed.TotalSeconds;
                    double tStep = tGlobal - stepStartGlobal;
                    bool isSettling = tStep < item.SettleSec;
                    double phi = stepPhaseOffset + 2.0 * Math.PI * f * tStep;
                    double uPct = Math.Clamp(_cfg.U0Percent + _cfg.UaPercent * Math.Sin(phi), 0.0, 100.0);

                    var sample = SendAndSample(activeCh, uPct, tGlobal, f, item.StepIdx, isSettling, st);
                    if (sample != null && !isSettling)
                    {
                        stepTRel.Add(tStep);
                        stepY.Add(sample.Mv[measureCh]);
                    }

                    UpdateLoopStatus(item.StepIdx + 1, f, tGlobal / totalExpectedSec, st);
                    nextTickGlobal += dt;
                    WaitForNextTick(st.GlobalSw, nextTickGlobal);
                }

                double stepEndGlobal = st.GlobalSw.Elapsed.TotalSeconds;
                phaseRad = (stepPhaseOffset + 2.0 * Math.PI * f * (stepEndGlobal - stepStartGlobal)) % (2.0 * Math.PI);
                if (phaseRad < 0.0) phaseRad += 2.0 * Math.PI;

                if (stepTRel.Count >= 6)
                {
                    var (dc, amp, phaseDeg) = FitSinusoid(stepTRel.ToArray(), stepY.ToArray(), f, stepPhaseOffset);
                    lock (_lock)
                    {
                        _bodePoints.Add(new BodePoint(item.StepIdx, f, stepTRel.Count, amp, phaseDeg, 0.0, dc));
                        RecomputeBodeMagnitudesLocked();
                    }
                    UpdateBodeSummaryInDb();
                }
            }
        }

        private void RunChirpLoop(double sampleRate, double dt, int activeCh, int measureCh, LoopCounters st)
        {
            double fMin = Math.Max(0.02, Math.Min(_cfg.FreqMinHz, _cfg.FreqMaxHz));
            double fMax = Math.Max(fMin * 1.05, Math.Max(_cfg.FreqMinHz, _cfg.FreqMaxHz));
            double f0 = _cfg.FreqDescending ? fMax : fMin;
            double f1 = _cfg.FreqDescending ? fMin : fMax;

            double totalDur = Math.Max(5.0, _cfg.TotalDurationSec);
            double preSettleSec = Math.Clamp(_cfg.ChirpSettleSec, 0.0, totalDur * 0.25);
            double chirpDur = Math.Max(2.0, totalDur - preSettleSec);

            int nBins = Math.Clamp(_cfg.FreqPoints, 10, 120);
            double[] evalFreqs = BuildLogSpacedFrequencies(fMin, fMax, nBins, descending: _cfg.FreqDescending);
            bool[] binDone = new bool[nBins];

            SetStatus(s => s with { Running = true, State = "running", Error = "", TotalSteps = nBins });
            _log($"[bandwidth] Started Log-Chirp ({f0:G3}->{f1:G3} Hz, T={totalDur:F1} s, settle={preSettleSec:F1} s, {nBins} eval bins, fs={sampleRate:F1} Hz) -> {_cfg.DbPath}");

            int estCap = (int)(chirpDur * sampleRate) + 64;
            var chirpT = new List<double>(estCap);
            var chirpU = new List<double>(estCap);
            var chirpY = new List<double>(estCap);

            var sweepSw = Stopwatch.StartNew();
            double nextTick = 0.0;
            int completedBins = 0;

            while (sweepSw.Elapsed.TotalSeconds < totalDur && !_cts.IsCancellationRequested)
            {
                double tElapsed = sweepSw.Elapsed.TotalSeconds;
                double tGlobal = st.GlobalSw.Elapsed.TotalSeconds;
                bool isSettling = tElapsed < preSettleSec;
                double tChirp = Math.Max(0.0, tElapsed - preSettleSec);

                double fInst = isSettling ? f0 : ChirpInstantFreq(tChirp, f0, f1, chirpDur);
                double phi = isSettling ? 0.0 : ChirpPhaseRad(tChirp, f0, f1, chirpDur);
                double uPct = Math.Clamp(_cfg.U0Percent + _cfg.UaPercent * Math.Sin(phi), 0.0, 100.0);

                var sample = SendAndSample(activeCh, uPct, tGlobal, fInst, 0, isSettling, st);
                if (sample != null && !isSettling)
                {
                    chirpT.Add(tChirp);
                    chirpU.Add(uPct);
                    chirpY.Add(sample.Mv[measureCh]);

                    // Incrementally demodulate bins whose right window tail has passed
                    bool addedAny = false;
                    for (int b = 0; b < nBins; b++)
                    {
                        if (binDone[b]) continue;
                        double fc = evalFreqs[b];
                        double tc = ChirpTimeAtFreq(fc, f0, f1, chirpDur);
                        double halfW = ChirpHalfWindowSec(fc, chirpDur);
                        if (tChirp >= tc + halfW)
                        {
                            binDone[b] = true;
                            completedBins++;
                            var pt = EvaluateChirpBin(
                                chirpT, chirpU, chirpY, f0, f1, chirpDur, _cfg.UaPercent, fc, b);
                            if (pt != null)
                            {
                                lock (_lock)
                                {
                                    _bodePoints.Add(pt);
                                    RecomputeBodeMagnitudesLocked();
                                }
                                addedAny = true;
                            }
                        }
                    }
                    if (addedAny) UpdateBodeSummaryInDb();
                }

                UpdateLoopStatus(Math.Max(1, completedBins), fInst, tElapsed / totalDur, st);
                nextTick += dt;
                WaitForNextTick(sweepSw, nextTick);
            }

            // Finalize remaining tail bins
            if (chirpT.Count >= 12)
            {
                bool addedAny = false;
                double tMaxMeasured = chirpT[^1];
                for (int b = 0; b < nBins; b++)
                {
                    if (binDone[b]) continue;
                    double fc = evalFreqs[b];
                    double tc = ChirpTimeAtFreq(fc, f0, f1, chirpDur);
                    if (_cts.IsCancellationRequested && tc > tMaxMeasured) continue;
                    binDone[b] = true;
                    var pt = EvaluateChirpBin(
                        chirpT, chirpU, chirpY, f0, f1, chirpDur, _cfg.UaPercent, fc, b);
                    if (pt != null)
                    {
                        lock (_lock)
                        {
                            _bodePoints.Add(pt);
                            RecomputeBodeMagnitudesLocked();
                        }
                        addedAny = true;
                    }
                }
                if (addedAny) UpdateBodeSummaryInDb();
            }
        }

        private BandwidthSample? SendAndSample(
            int activeCh,
            double uPct,
            double tGlobal,
            double freqHz,
            int stepIdx,
            bool isSettling,
            LoopCounters st)
        {
            byte uByte = (byte)Math.Clamp((int)Math.Round(uPct * 255.0 / 100.0), 0, 255);

            byte[] payload = new byte[8];
            byte inactiveByte = _cfg.ZeroInactiveChannels
                ? (byte)0
                : (byte)Math.Clamp((int)Math.Round(_cfg.U0Percent * 255.0 / 100.0), 0, 255);
            for (int c = 0; c < 5; c++)
                payload[c] = (activeCh < 0 || activeCh == c) ? uByte : inactiveByte;

            _motor.WriteFrame(_encoder.Encode(payload));

            ushort[] raw = new ushort[5];
            double[] mv = new double[5];
            bool readOk = false;
            try
            {
                var res = _motor.ReadSpiAdcChannelsDirect(_cfg.SpiCsIdx, 0.04);
                if (res.HasValue)
                {
                    raw = res.Value.Channels;
                    for (int c = 0; c < 5; c++)
                        mv[c] = raw[c] * _cfg.VrefMv / 4095.0;
                    readOk = true;
                }
                else
                {
                    st.ReadErrors++;
                }
            }
            catch
            {
                st.ReadErrors++;
            }

            BandwidthSample? sample = null;
            if (readOk)
            {
                sample = new BandwidthSample(tGlobal, freqHz, stepIdx, isSettling, uPct, uByte, raw, mv);
                _dbQueue.Add(sample);
                lock (_lock)
                {
                    if (_liveRing.Count >= LiveCapacity)
                        _liveRing.RemoveAt(0);
                    _liveRing.Add(sample);
                }
            }

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

        private void UpdateLoopStatus(int currentStep, double currentFreqHz, double progress01, LoopCounters st)
        {
            SetStatus(s => s with
            {
                CurrentStep = currentStep,
                CurrentFreqHz = currentFreqHz,
                Progress01 = Math.Clamp(progress01, 0.0, 1.0),
                SampleCount = st.TotalSamples,
                ReadErrors = st.ReadErrors,
                ActualRateHz = st.ActualHz
            });
        }

        private void WaitForNextTick(Stopwatch sw, double nextTickSec)
        {
            double remaining = nextTickSec - sw.Elapsed.TotalSeconds;
            if (remaining > 0.002)
                Thread.Sleep((int)((remaining - 0.001) * 1000.0));
            while (sw.Elapsed.TotalSeconds < nextTickSec && !_cts.IsCancellationRequested)
                Thread.SpinWait(20);
        }

        /// <summary>
        /// Generates logarithmically spaced frequencies between fMin and fMax.
        /// </summary>
        public static double[] BuildLogSpacedFrequencies(double fMin, double fMax, int count, bool descending = false)
        {
            fMin = Math.Max(0.02, fMin);
            fMax = Math.Max(fMin, fMax);
            count = Math.Clamp(count, 2, 200);
            var arr = new double[count];
            double logMin = Math.Log10(fMin);
            double logMax = Math.Log10(fMax);
            for (int i = 0; i < count; i++)
            {
                double frac = (double)i / (count - 1);
                double f = Math.Pow(10.0, logMin + frac * (logMax - logMin));
                arr[i] = Math.Round(f, 4);
            }
            if (descending) Array.Reverse(arr);
            return arr;
        }

        /// <summary>
        /// Builds the stepped-sine schedule according to FixedCycles or TotalDurationSec mode.
        /// </summary>
        public static List<SteppedScheduleItem> BuildSteppedSchedule(BandwidthExperimentConfig cfg)
        {
            double[] freqs = cfg.FrequenciesHz != null && cfg.FrequenciesHz.Length > 0
                ? cfg.FrequenciesHz
                : BuildLogSpacedFrequencies(cfg.FreqMinHz, cfg.FreqMaxHz, cfg.FreqPoints, cfg.FreqDescending);

            int n = freqs.Length;
            var result = new List<SteppedScheduleItem>(n);

            if (cfg.TimingMode == SteppedTimingMode.FixedCycles || cfg.TotalDurationSec <= 0)
            {
                for (int i = 0; i < n; i++)
                {
                    double f = Math.Max(0.02, freqs[i]);
                    double settle = Math.Max(0, cfg.SettleCycles) / f;
                    double meas = Math.Max(1, cfg.CyclesPerFreq) / f;
                    result.Add(new SteppedScheduleItem(i, f, settle, meas));
                }
                return result;
            }

            double target = Math.Max(5.0, cfg.TotalDurationSec);
            double[] minSettle = new double[n];
            double[] minMeas = new double[n];
            double[] fullSettle = new double[n];
            double[] fullMeas = new double[n];

            for (int i = 0; i < n; i++)
            {
                double f = Math.Max(0.02, freqs[i]);
                minSettle[i] = cfg.SettleCycles <= 0 ? 0.0 : Math.Clamp(1.0 / f, 0.25, 1.5);
                minMeas[i] = Math.Max(1.0 / f, 0.8);
                fullSettle[i] = cfg.SettleCycles <= 0 ? 0.0 : Math.Max(minSettle[i], cfg.SettleCycles / f);
                fullMeas[i] = Math.Max(minMeas[i], Math.Max(1, cfg.CyclesPerFreq) / f);
            }

            double sumMin = 0, sumFull = 0;
            for (int i = 0; i < n; i++)
            {
                sumMin += minSettle[i] + minMeas[i];
                sumFull += fullSettle[i] + fullMeas[i];
            }

            if (target >= sumFull)
            {
                double scale = target / Math.Max(1e-6, sumFull);
                for (int i = 0; i < n; i++)
                    result.Add(new SteppedScheduleItem(i, Math.Max(0.02, freqs[i]), fullSettle[i] * scale, fullMeas[i] * scale));
            }
            else if (target >= sumMin)
            {
                double alpha = (target - sumMin) / Math.Max(1e-6, sumFull - sumMin);
                for (int i = 0; i < n; i++)
                {
                    double s = minSettle[i] + alpha * (fullSettle[i] - minSettle[i]);
                    double m = minMeas[i] + alpha * (fullMeas[i] - minMeas[i]);
                    result.Add(new SteppedScheduleItem(i, Math.Max(0.02, freqs[i]), s, m));
                }
            }
            else
            {
                // Tight budget: trim settle first, then scale measurement proportionally
                double sumMinMeas = minMeas.Sum();
                double sumMinSettle = minSettle.Sum();
                double availSettle = Math.Max(0.0, target - sumMinMeas);
                if (sumMinSettle > 1e-6 && availSettle >= 0.25 * sumMinSettle)
                {
                    double sScale = availSettle / sumMinSettle;
                    for (int i = 0; i < n; i++)
                        result.Add(new SteppedScheduleItem(i, Math.Max(0.02, freqs[i]), minSettle[i] * sScale, minMeas[i]));
                }
                else
                {
                    double totalScale = target / Math.Max(1e-6, sumMin);
                    for (int i = 0; i < n; i++)
                        result.Add(new SteppedScheduleItem(i, Math.Max(0.02, freqs[i]), minSettle[i] * totalScale, minMeas[i] * totalScale));
                }
            }

            return result;
        }

        /// <summary>
        /// Estimates total experiment runtime in seconds for the given config.
        /// </summary>
        public static double EstimateDurationSec(BandwidthExperimentConfig cfg)
        {
            if (cfg.SweepMode == BandwidthSweepMode.Chirp)
                return Math.Max(5.0, cfg.TotalDurationSec);
            return BuildSteppedSchedule(cfg).Sum(s => s.SettleSec + s.MeasureSec);
        }

        public static double ChirpInstantFreq(double tChirp, double f0, double f1, double chirpDur)
        {
            double frac = Math.Clamp(tChirp / Math.Max(1e-6, chirpDur), 0.0, 1.0);
            return f0 * Math.Pow(f1 / f0, frac);
        }

        public static double ChirpPhaseRad(double tChirp, double f0, double f1, double chirpDur)
        {
            if (Math.Abs(f1 - f0) <= 1e-6 * Math.Max(f0, f1))
                return 2.0 * Math.PI * f0 * tChirp;
            double lnRatio = Math.Log(f1 / f0);
            return (2.0 * Math.PI * f0 * chirpDur / lnRatio) * (Math.Exp(lnRatio * (tChirp / chirpDur)) - 1.0);
        }

        private static double ChirpTimeAtFreq(double freq, double f0, double f1, double chirpDur)
        {
            if (Math.Abs(f1 - f0) <= 1e-6 * Math.Max(f0, f1))
                return 0.5 * chirpDur;
            double frac = Math.Log(freq / f0) / Math.Log(f1 / f0);
            return Math.Clamp(frac, 0.0, 1.0) * chirpDur;
        }

        private static double ChirpHalfWindowSec(double evalFreq, double chirpDur)
        {
            return Math.Clamp(1.75 / Math.Max(0.02, evalFreq), 0.6, Math.Max(1.5, chirpDur * 0.22));
        }

        /// <summary>
        /// Demodulates one frequency bin from a continuous log-chirp recording using Hann-weighted
        /// local least squares against exact instantaneous chirp phase.
        /// </summary>
        public static BodePoint? EvaluateChirpBin(
            IReadOnlyList<double> tChirp,
            IReadOnlyList<double> uPct,
            IReadOnlyList<double> yMv,
            double f0,
            double f1,
            double chirpDur,
            double uaPct,
            double evalFreq,
            int stepIdx)
        {
            double tc = ChirpTimeAtFreq(evalFreq, f0, f1, chirpDur);
            double halfW = ChirpHalfWindowSec(evalFreq, chirpDur);
            double tSkip = Math.Min(0.35, 0.02 * chirpDur);

            var idxList = new List<int>(256);
            for (int i = 0; i < tChirp.Count; i++)
            {
                double t = tChirp[i];
                if (t >= tc - halfW && t <= tc + halfW && (t >= tSkip || tc < 0.25))
                    idxList.Add(i);
            }
            if (idxList.Count < 8)
            {
                idxList.Clear();
                for (int i = 0; i < tChirp.Count; i++)
                {
                    double t = tChirp[i];
                    if (t >= tc - halfW && t <= tc + halfW)
                        idxList.Add(i);
                }
            }
            if (idxList.Count < 6) return null;

            double tMean = 0.0;
            foreach (int i in idxList) tMean += tChirp[i];
            tMean /= idxList.Count;

            double[,] Au = new double[4, 5];
            double[,] Ay = new double[4, 5];
            double[] r = new double[4];

            foreach (int i in idxList)
            {
                double t = tChirp[i];
                double phi = ChirpPhaseRad(t, f0, f1, chirpDur);
                double w = 0.5 * (1.0 + Math.Cos(Math.PI * Math.Clamp((t - tc) / halfW, -1.0, 1.0)));
                w = Math.Max(w, 1e-3);

                r[0] = 1.0;
                r[1] = t - tMean;
                r[2] = Math.Cos(phi);
                r[3] = Math.Sin(phi);

                double ui = uPct[i];
                double yi = yMv[i];

                for (int row = 0; row < 4; row++)
                {
                    double wr = w * r[row];
                    for (int col = 0; col < 4; col++)
                    {
                        double val = wr * r[col];
                        Au[row, col] += val;
                        Ay[row, col] += val;
                    }
                    Au[row, 4] += wr * ui;
                    Ay[row, 4] += wr * yi;
                }
            }

            if (!SolveLinearSystem(Au, 4, out double[] bu) || !SolveLinearSystem(Ay, 4, out double[] by))
                return null;

            double ampU = Math.Sqrt(bu[2] * bu[2] + bu[3] * bu[3]);
            double ampY = Math.Sqrt(by[2] * by[2] + by[3] * by[3]);
            if (ampU < 1e-4) return null;

            double ampScaled = ampY * (uaPct / ampU);
            double phaseU = Math.Atan2(bu[2], bu[3]);
            double phaseY = Math.Atan2(by[2], by[3]);
            double phaseDeg = (phaseY - phaseU) * (180.0 / Math.PI);
            while (phaseDeg > 180.0) phaseDeg -= 360.0;
            while (phaseDeg <= -180.0) phaseDeg += 360.0;

            return new BodePoint(stepIdx, evalFreq, idxList.Count, ampScaled, phaseDeg, 0.0, by[0]);
        }

        private void RecomputeBodeMagnitudesLocked()
        {
            if (_bodePoints.Count == 0) return;
            // Use the lowest-frequency point measured so far as A_ref (quasi-static reference)
            var refPt = _bodePoints.OrderBy(p => p.FreqHz).First();
            double aRef = Math.Max(1e-6, refPt.AmpMv);

            for (int i = 0; i < _bodePoints.Count; i++)
            {
                var p = _bodePoints[i];
                double magDb = 20.0 * Math.Log10(Math.Max(1e-6, p.AmpMv) / aRef);
                _bodePoints[i] = p with { MagDb = magDb };
            }

            double? fc = ComputeBandwidthFcHz(_bodePoints);
            _status = _status with { BandwidthFcHz = fc };
        }

        public static (double DcOffset, double Amplitude, double PhaseDeg) FitSinusoid(
            double[] t, double[] y, double freqHz, double phaseOffsetRad = 0.0)
        {
            int n = Math.Min(t.Length, y.Length);
            if (n < 3) return (0, 0, 0);

            double w = 2.0 * Math.PI * freqHz;
            // Normal equations for y ≈ c0 + a*cos(wt + phi0) + b*sin(wt + phi0)
            double[,] A = new double[3, 4];
            for (int i = 0; i < n; i++)
            {
                double ang = w * t[i] + phaseOffsetRad;
                double c = Math.Cos(ang);
                double s = Math.Sin(ang);
                double yi = y[i];

                A[0, 0] += 1.0; A[0, 1] += c;     A[0, 2] += s;     A[0, 3] += yi;
                A[1, 0] += c;   A[1, 1] += c * c; A[1, 2] += c * s; A[1, 3] += c * yi;
                A[2, 0] += s;   A[2, 1] += c * s; A[2, 2] += s * s; A[2, 3] += s * yi;
            }

            if (!SolveLinearSystem(A, 3, out double[] x))
                return (0, 0, 0);

            double c0 = x[0];
            double a = x[1];
            double b = x[2];
            double amp = Math.Sqrt(a * a + b * b);
            // Relative to sin(wt): a*cos(wt) + b*sin(wt) = amp * sin(wt + phi) => phi = atan2(a, b)
            double phaseDeg = Math.Atan2(a, b) * (180.0 / Math.PI);
            return (c0, amp, phaseDeg);
        }

        private static bool SolveLinearSystem(double[,] A, int dim, out double[] x)
        {
            x = new double[dim];
            for (int col = 0; col < dim; col++)
            {
                int pivot = col;
                for (int r = col + 1; r < dim; r++)
                {
                    if (Math.Abs(A[r, col]) > Math.Abs(A[pivot, col])) pivot = r;
                }
                for (int k = col; k <= dim; k++)
                {
                    (A[col, k], A[pivot, k]) = (A[pivot, k], A[col, k]);
                }
                double diag = A[col, col];
                if (Math.Abs(diag) < 1e-12) return false;
                for (int r = col + 1; r < dim; r++)
                {
                    double factor = A[r, col] / diag;
                    for (int k = col; k <= dim; k++)
                        A[r, k] -= factor * A[col, k];
                }
            }

            for (int r = dim - 1; r >= 0; r--)
            {
                double sum = A[r, dim];
                for (int k = r + 1; k < dim; k++) sum -= A[r, k] * x[k];
                x[r] = sum / A[r, r];
            }
            return true;
        }

        public static double? ComputeBandwidthFcHz(IReadOnlyList<BodePoint> points)
        {
            if (points.Count < 2) return null;
            var sorted = points.OrderBy(p => p.FreqHz).ToList();
            for (int i = 0; i < sorted.Count - 1; i++)
            {
                var p0 = sorted[i];
                var p1 = sorted[i + 1];
                if (p0.MagDb >= -3.0 && p1.MagDb <= -3.0 && Math.Abs(p1.MagDb - p0.MagDb) > 1e-9)
                {
                    double frac = (-3.0 - p0.MagDb) / (p1.MagDb - p0.MagDb);
                    return p0.FreqHz + frac * (p1.FreqHz - p0.FreqHz);
                }
            }
            return null;
        }

        private void InitDatabase()
        {
            using var conn = new SqliteConnection($"Data Source={_cfg.DbPath}");
            conn.Open();
            using var cmd = conn.CreateCommand();
            cmd.CommandText = @"
                PRAGMA journal_mode=WAL;
                PRAGMA synchronous=NORMAL;
                CREATE TABLE IF NOT EXISTS experiment_meta (
                    key TEXT PRIMARY KEY,
                    value TEXT
                );
                CREATE TABLE IF NOT EXISTS samples (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    t_s REAL NOT NULL,
                    freq_hz REAL NOT NULL,
                    step_idx INTEGER NOT NULL,
                    is_settling INTEGER NOT NULL,
                    u_pct REAL NOT NULL,
                    u_byte INTEGER NOT NULL,
                    ch0_raw INTEGER NOT NULL,
                    ch1_raw INTEGER NOT NULL,
                    ch2_raw INTEGER NOT NULL,
                    ch3_raw INTEGER NOT NULL,
                    ch4_raw INTEGER NOT NULL,
                    ch0_mv REAL NOT NULL,
                    ch1_mv REAL NOT NULL,
                    ch2_mv REAL NOT NULL,
                    ch3_mv REAL NOT NULL,
                    ch4_mv REAL NOT NULL
                );
                CREATE TABLE IF NOT EXISTS bode_summary (
                    step_idx INTEGER PRIMARY KEY,
                    freq_hz REAL NOT NULL,
                    n_samples INTEGER NOT NULL,
                    amp_mv REAL NOT NULL,
                    phase_deg REAL NOT NULL,
                    mag_db REAL NOT NULL,
                    dc_offset_mv REAL NOT NULL
                );";
            cmd.ExecuteNonQuery();

            double fMin = Math.Max(0.02, Math.Min(_cfg.FreqMinHz, _cfg.FreqMaxHz));
            double fMax = Math.Max(fMin * 1.05, Math.Max(_cfg.FreqMinHz, _cfg.FreqMaxHz));
            double f0 = _cfg.FreqDescending ? fMax : fMin;
            double f1 = _cfg.FreqDescending ? fMin : fMax;
            double totalDur = Math.Max(5.0, _cfg.TotalDurationSec);
            double preSettleSec = Math.Clamp(_cfg.ChirpSettleSec, 0.0, totalDur * 0.25);

            var meta = new Dictionary<string, string>
            {
                ["created_iso"] = DateTime.UtcNow.ToString("O"),
                ["channel_index"] = _cfg.ChannelIndex.ToString(CultureInfo.InvariantCulture),
                ["sweep_mode"] = _cfg.SweepMode == BandwidthSweepMode.Chirp ? "chirp" : "stepped_sine",
                ["timing_mode"] = _cfg.TimingMode == SteppedTimingMode.TotalDurationSec ? "total_duration" : "fixed_cycles",
                ["total_duration_sec"] = totalDur.ToString(CultureInfo.InvariantCulture),
                ["freq_min_hz"] = fMin.ToString(CultureInfo.InvariantCulture),
                ["freq_max_hz"] = fMax.ToString(CultureInfo.InvariantCulture),
                ["freq_points"] = _cfg.FreqPoints.ToString(CultureInfo.InvariantCulture),
                ["freq_descending"] = _cfg.FreqDescending ? "1" : "0",
                ["chirp_f0_hz"] = f0.ToString(CultureInfo.InvariantCulture),
                ["chirp_f1_hz"] = f1.ToString(CultureInfo.InvariantCulture),
                ["chirp_settle_sec"] = preSettleSec.ToString(CultureInfo.InvariantCulture),
                ["u0_pct"] = _cfg.U0Percent.ToString(CultureInfo.InvariantCulture),
                ["ua_pct"] = _cfg.UaPercent.ToString(CultureInfo.InvariantCulture),
                ["frequencies_hz"] = string.Join(",", _cfg.FrequenciesHz.Select(f => f.ToString(CultureInfo.InvariantCulture))),
                ["cycles_per_freq"] = _cfg.CyclesPerFreq.ToString(CultureInfo.InvariantCulture),
                ["settle_cycles"] = _cfg.SettleCycles.ToString(CultureInfo.InvariantCulture),
                ["sample_rate_hz"] = _cfg.SampleRateHz.ToString(CultureInfo.InvariantCulture),
                ["spi_bus_id"] = _cfg.SpiBusId.ToString(CultureInfo.InvariantCulture),
                ["spi_cs_idx"] = _cfg.SpiCsIdx.ToString(CultureInfo.InvariantCulture),
                ["spi_period_ms"] = _cfg.SpiPeriodMs.ToString(CultureInfo.InvariantCulture),
                ["vref_mv"] = _cfg.VrefMv.ToString(CultureInfo.InvariantCulture)
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

            using var ins = conn.CreateCommand();
            ins.CommandText = @"
                INSERT INTO samples
                (t_s, freq_hz, step_idx, is_settling, u_pct, u_byte,
                 ch0_raw, ch1_raw, ch2_raw, ch3_raw, ch4_raw,
                 ch0_mv, ch1_mv, ch2_mv, ch3_mv, ch4_mv)
                VALUES
                ($t, $f, $st, $settle, $upct, $uby,
                 $r0, $r1, $r2, $r3, $r4,
                 $m0, $m1, $m2, $m3, $m4);";
            var pT = ins.Parameters.Add("$t", SqliteType.Real);
            var pF = ins.Parameters.Add("$f", SqliteType.Real);
            var pSt = ins.Parameters.Add("$st", SqliteType.Integer);
            var pSet = ins.Parameters.Add("$settle", SqliteType.Integer);
            var pUp = ins.Parameters.Add("$upct", SqliteType.Real);
            var pUb = ins.Parameters.Add("$uby", SqliteType.Integer);
            var pR = Enumerable.Range(0, 5).Select(i => ins.Parameters.Add($"$r{i}", SqliteType.Integer)).ToArray();
            var pM = Enumerable.Range(0, 5).Select(i => ins.Parameters.Add($"$m{i}", SqliteType.Real)).ToArray();

            var batch = new List<BandwidthSample>(64);
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
                        pT.Value = s.TimeSec;
                        pF.Value = s.FreqHz;
                        pSt.Value = s.StepIdx;
                        pSet.Value = s.IsSettling ? 1 : 0;
                        pUp.Value = s.UPct;
                        pUb.Value = (int)s.UByte;
                        for (int i = 0; i < 5; i++)
                        {
                            pR[i].Value = (int)s.Raw[i];
                            pM[i].Value = s.Mv[i];
                        }
                        ins.ExecuteNonQuery();
                    }
                    tx.Commit();
                    batch.Clear();
                }

                if (_bodeDirty)
                {
                    _bodeDirty = false;
                    FlushBodeSummary(conn);
                }
            }

            if (_bodeDirty)
            {
                _bodeDirty = false;
                FlushBodeSummary(conn);
            }
        }

        private void UpdateBodeSummaryInDb()
        {
            _bodeDirty = true;
        }

        private void FlushBodeSummary(SqliteConnection conn)
        {
            List<BodePoint> copy;
            lock (_lock) copy = new List<BodePoint>(_bodePoints);
            if (copy.Count == 0) return;

            using var tx = conn.BeginTransaction();
            foreach (var p in copy)
            {
                using var cmd = conn.CreateCommand();
                cmd.Transaction = tx;
                cmd.CommandText = @"
                    INSERT OR REPLACE INTO bode_summary
                    (step_idx, freq_hz, n_samples, amp_mv, phase_deg, mag_db, dc_offset_mv)
                    VALUES ($idx, $f, $n, $amp, $ph, $mag, $dc);";
                cmd.Parameters.AddWithValue("$idx", p.StepIdx);
                cmd.Parameters.AddWithValue("$f", p.FreqHz);
                cmd.Parameters.AddWithValue("$n", p.NSamples);
                cmd.Parameters.AddWithValue("$amp", p.AmpMv);
                cmd.Parameters.AddWithValue("$ph", p.PhaseDeg);
                cmd.Parameters.AddWithValue("$mag", p.MagDb);
                cmd.Parameters.AddWithValue("$dc", p.DcOffsetMv);
                cmd.ExecuteNonQuery();
            }
            tx.Commit();
        }

        private void SetStatus(Func<BandwidthExperimentStatus, BandwidthExperimentStatus> fn)
        {
            lock (_lock) _status = fn(_status);
        }

        public void Dispose()
        {
            Stop();
            _runThread?.Join(500);
            _dbThread?.Join(1000);
            _cts.Dispose();
            _dbQueue.Dispose();
        }
    }
}
