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
    public record BandwidthExperimentConfig
    {
        public int ChannelIndex { get; init; } = 0; // 0..4 (A..E), or -1 for all 5
        public double U0Percent { get; init; } = 50.0;
        public double UaPercent { get; init; } = 25.0;
        public double[] FrequenciesHz { get; init; } = { 10.0, 7.0, 5.0, 4.0, 3.0, 2.0, 1.0, 0.5, 0.2, 0.1 };
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

            _status = new BandwidthExperimentStatus(
                Running: false,
                State: "idle",
                Error: "",
                CurrentStep: 0,
                TotalSteps: _cfg.FrequenciesHz.Length,
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
                double[] freqs = _cfg.FrequenciesHz;
                int totalSteps = freqs.Length;

                // Estimate total duration for progress bar
                double totalExpectedSec = freqs.Sum(f => (_cfg.CyclesPerFreq + _cfg.SettleCycles) / Math.Max(0.05, f));

                var globalSw = Stopwatch.StartNew();
                long totalSamples = 0;
                long readErrors = 0;
                var rateSw = Stopwatch.StartNew();
                long rateSamples = 0;
                double actualHz = 0.0;

                SetStatus(s => s with { Running = true, State = "running", Error = "", TotalSteps = totalSteps });
                _log($"[bandwidth] Started sweep ({totalSteps} steps, fs={sampleRate:F1} Hz) -> {_cfg.DbPath}");

                for (int stepIdx = 0; stepIdx < totalSteps && !_cts.IsCancellationRequested; stepIdx++)
                {
                    double f = Math.Max(0.02, freqs[stepIdx]);
                    int totalCycles = Math.Max(1, _cfg.CyclesPerFreq + _cfg.SettleCycles);
                    double stepDur = totalCycles / f;
                    double settleDur = _cfg.SettleCycles / f;

                    var stepTRel = new List<double>((int)(stepDur * sampleRate) + 16);
                    var stepY = new List<double>((int)(stepDur * sampleRate) + 16);

                    var stepSw = Stopwatch.StartNew();
                    double nextTick = 0.0;

                    while (stepSw.Elapsed.TotalSeconds < stepDur && !_cts.IsCancellationRequested)
                    {
                        double tStep = stepSw.Elapsed.TotalSeconds;
                        double tGlobal = globalSw.Elapsed.TotalSeconds;
                        bool isSettling = tStep < settleDur;

                        double uPct = Math.Clamp(_cfg.U0Percent + _cfg.UaPercent * Math.Sin(2.0 * Math.PI * f * tStep), 0.0, 100.0);
                        byte uByte = (byte)Math.Clamp((int)Math.Round(uPct * 255.0 / 100.0), 0, 255);

                        // 1. Send 0x300 command frame to ESP32 via STM32 master
                        byte[] payload = new byte[8];
                        for (int c = 0; c < 5; c++)
                        {
                            if (activeCh < 0 || activeCh == c)
                                payload[c] = uByte;
                            else
                                payload[c] = _cfg.ZeroInactiveChannels
                                    ? (byte)0
                                    : (byte)Math.Clamp((int)Math.Round(_cfg.U0Percent * 255.0 / 100.0), 0, 255);
                        }
                        payload[5] = 0; // force_lsb reserved
                        payload[6] = 0; // flags
                        payload[7] = 0; // checksum / reserved
                        _motor.WriteFrame(_encoder.Encode(payload));

                        // 2. Read 5-channel ADC back over SPI (0x503)
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
                                readErrors++;
                            }
                        }
                        catch
                        {
                            readErrors++;
                        }

                        if (readOk)
                        {
                            var sample = new BandwidthSample(tGlobal, f, stepIdx, isSettling, uPct, uByte, raw, mv);
                            _dbQueue.Add(sample);
                            lock (_lock)
                            {
                                if (_liveRing.Count >= LiveCapacity)
                                    _liveRing.RemoveAt(0);
                                _liveRing.Add(sample);
                            }

                            if (!isSettling)
                            {
                                stepTRel.Add(tStep);
                                stepY.Add(mv[measureCh]);
                            }
                        }

                        totalSamples++;
                        rateSamples++;
                        if (rateSw.ElapsedMilliseconds >= 500)
                        {
                            actualHz = rateSamples / (rateSw.ElapsedMilliseconds / 1000.0);
                            rateSamples = 0;
                            rateSw.Restart();
                        }

                        double progress = Math.Clamp(globalSw.Elapsed.TotalSeconds / Math.Max(0.1, totalExpectedSec), 0.0, 1.0);
                        SetStatus(s => s with
                        {
                            CurrentStep = stepIdx + 1,
                            CurrentFreqHz = f,
                            Progress01 = progress,
                            SampleCount = totalSamples,
                            ReadErrors = readErrors,
                            ActualRateHz = actualHz
                        });

                        // Precise hybrid sleep/spin for next sample period
                        nextTick += dt;
                        double remaining = nextTick - stepSw.Elapsed.TotalSeconds;
                        if (remaining > 0.002)
                            Thread.Sleep((int)((remaining - 0.001) * 1000.0));
                        while (stepSw.Elapsed.TotalSeconds < nextTick && !_cts.IsCancellationRequested)
                            Thread.SpinWait(20);
                    }

                    // Fit sinusoid at frequency f on the settled portion
                    if (stepTRel.Count >= 6)
                    {
                        var (dc, amp, phaseDeg) = FitSinusoid(stepTRel.ToArray(), stepY.ToArray(), f);
                        lock (_lock)
                        {
                            _bodePoints.Add(new BodePoint(stepIdx, f, stepTRel.Count, amp, phaseDeg, 0.0, dc));
                            RecomputeBodeMagnitudesLocked();
                        }
                        UpdateBodeSummaryInDb();
                    }
                }

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
                _log($"[bandwidth] {finalState}: {totalSamples} samples, {readErrors} read errors, fc={(fc.HasValue ? $"{fc.Value:F2} Hz" : "N/A")}");
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
                TimeEndPeriod(1);
            }
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
            double[] t, double[] y, double freqHz)
        {
            int n = Math.Min(t.Length, y.Length);
            if (n < 3) return (0, 0, 0);

            double w = 2.0 * Math.PI * freqHz;
            // Normal equations for y ≈ c0 + a*cos(wt) + b*sin(wt)
            double[,] A = new double[3, 4];
            for (int i = 0; i < n; i++)
            {
                double c = Math.Cos(w * t[i]);
                double s = Math.Sin(w * t[i]);
                double yi = y[i];

                A[0, 0] += 1.0; A[0, 1] += c;     A[0, 2] += s;     A[0, 3] += yi;
                A[1, 0] += c;   A[1, 1] += c * c; A[1, 2] += c * s; A[1, 3] += c * yi;
                A[2, 0] += s;   A[2, 1] += c * s; A[2, 2] += s * s; A[2, 3] += s * yi;
            }

            // Gaussian elimination with partial pivoting
            for (int col = 0; col < 3; col++)
            {
                int pivot = col;
                for (int r = col + 1; r < 3; r++)
                {
                    if (Math.Abs(A[r, col]) > Math.Abs(A[pivot, col])) pivot = r;
                }
                for (int k = col; k < 4; k++)
                {
                    (A[col, k], A[pivot, k]) = (A[pivot, k], A[col, k]);
                }
                double diag = A[col, col];
                if (Math.Abs(diag) < 1e-12) return (0, 0, 0);
                for (int r = col + 1; r < 3; r++)
                {
                    double factor = A[r, col] / diag;
                    for (int k = col; k < 4; k++)
                        A[r, k] -= factor * A[col, k];
                }
            }

            double[] x = new double[3];
            for (int r = 2; r >= 0; r--)
            {
                double sum = A[r, 3];
                for (int k = r + 1; k < 3; k++) sum -= A[r, k] * x[k];
                x[r] = sum / A[r, r];
            }

            double c0 = x[0];
            double a = x[1];
            double b = x[2];
            double amp = Math.Sqrt(a * a + b * b);
            // Relative to sin(wt): a*cos(wt) + b*sin(wt) = amp * sin(wt + phi) => phi = atan2(a, b)
            double phaseDeg = Math.Atan2(a, b) * (180.0 / Math.PI);
            return (c0, amp, phaseDeg);
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

            var meta = new Dictionary<string, string>
            {
                ["created_iso"] = DateTime.UtcNow.ToString("O"),
                ["channel_index"] = _cfg.ChannelIndex.ToString(CultureInfo.InvariantCulture),
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
            }
        }

        private void UpdateBodeSummaryInDb()
        {
            List<BodePoint> copy;
            lock (_lock) copy = new List<BodePoint>(_bodePoints);

            using var conn = new SqliteConnection($"Data Source={_cfg.DbPath}");
            conn.Open();
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
