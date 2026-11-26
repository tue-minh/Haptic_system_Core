using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Threading;
using Microsoft.Data.Sqlite;

namespace ControlGui.Core.Haptic
{
    public record SupRingState
    {
        public double Fx { get; init; } = 0.0; // -25.5..+25.5 N (-Fx = Slide Down -> Valve A, +Fx = Slide Up -> Valve C)
        public double Fy { get; init; } = 0.0; // -25.5..+25.5 N (-Fy = Pull -> Valve D, +Fy = Push -> Valve E)
        public double Fz { get; init; } = 0.0; //   0.0..+25.5 N (Fz = Squeeze -> Valve B)
        public bool AutoFlags { get; init; } = true;
        public byte ManualFlags { get; init; } = 0;
        public byte ReservedByte { get; init; } = 0;
    }

    public static class SupRingMapper
    {
        public static double ShearMagnitude(double fx, double fy)
        {
            return Math.Sqrt(fx * fx + fy * fy);
        }

        /// <summary>
        /// Angle in [0, 360) deg where:
        ///   0 deg = +Fy (Push -> Valve E, right on 2D pad)
        ///  90 deg = -Fx (Slide Down -> Valve A, top on 2D pad)
        /// 180 deg = -Fy (Pull -> Valve D, left on 2D pad)
        /// 270 deg = +Fx (Slide Up -> Valve C, bottom on 2D pad)
        /// </summary>
        public static double ShearAngleDeg(double fx, double fy)
        {
            if (Math.Abs(fx) < 1e-9 && Math.Abs(fy) < 1e-9)
                return 0.0;
            double deg = Math.Atan2(-fx, fy) * (180.0 / Math.PI);
            if (deg < 0.0) deg += 360.0;
            if (deg >= 360.0) deg -= 360.0;
            return deg;
        }

        public static (double Fx, double Fy) FromPolar(double shearMag, double angleDeg)
        {
            double mag = Math.Clamp(shearMag, 0.0, HapticConstants.MaxForceN);
            double rad = angleDeg * (Math.PI / 180.0);
            double fx = Math.Clamp(-mag * Math.Sin(rad), -HapticConstants.MaxForceN, HapticConstants.MaxForceN);
            double fy = Math.Clamp(mag * Math.Cos(rad), -HapticConstants.MaxForceN, HapticConstants.MaxForceN);
            return (fx, fy);
        }

        public static byte ComputeFlags(SupRingState state)
        {
            if (!state.AutoFlags)
                return state.ManualFlags;

            byte flags = 0;
            if (state.Fz > HapticConstants.ForceThresh)
            {
                flags |= HapticConstants.FlagLeftContact;
                flags |= HapticConstants.FlagRightContact;
                double shear = ShearMagnitude(state.Fx, state.Fy);
                if (shear / state.Fz > 0.50)
                    flags |= HapticConstants.FlagSlip;
            }
            return flags;
        }

        /// <summary>
        /// Build 8-byte CAN 0x300 payload following paper I01_3 DOF Pneumatic Tactile Display_v01.docx:
        ///   byte 0 (A, distal tip)   = max(0, -Fx)  [Slide Down]
        ///   byte 1 (B, medial mid)   = max(0,  Fz)  [Squeeze]
        ///   byte 2 (C, proximal)     = max(0, +Fx)  [Slide Up]
        ///   byte 3 (D, left flank)   = max(0, -Fy)  [Pull toward operator]
        ///   byte 4 (E, right flank)  = max(0, +Fy)  [Push away from operator]
        ///   byte 5                   = ReservedByte
        ///   byte 6                   = Status_Flags
        ///   byte 7                   = Frame_Counter
        /// </summary>
        public static byte[] BuildPayload(SupRingState state, byte counter)
        {
            double fA = state.Fx < 0.0 ? Math.Abs(state.Fx) : 0.0;
            double fB = state.Fz > 0.0 ? state.Fz : 0.0;
            double fC = state.Fx > 0.0 ? state.Fx : 0.0;
            double fD = state.Fy < 0.0 ? Math.Abs(state.Fy) : 0.0;
            double fE = state.Fy > 0.0 ? state.Fy : 0.0;

            return new byte[8]
            {
                HapticPayloadSynthesizer.ClampU8(fA),
                HapticPayloadSynthesizer.ClampU8(fB),
                HapticPayloadSynthesizer.ClampU8(fC),
                HapticPayloadSynthesizer.ClampU8(fD),
                HapticPayloadSynthesizer.ClampU8(fE),
                state.ReservedByte,
                ComputeFlags(state),
                counter
            };
        }
    }

    public record SupRingLogSample(
        double TimeSec,
        double Epoch,
        double Fx,
        double Fy,
        double Fz,
        double ShearMag,
        double ShearAngleDeg,
        byte[] Payload,
        ushort[] AdcRaw,
        double[] AdcMv
    );

    public sealed class SupRingSqliteLogger : IDisposable
    {
        private static readonly string[] SampleColumns =
        {
            "t_s", "epoch_s",
            "fx_n", "fy_n", "fz_n", "shear_mag_n", "shear_angle_deg",
            "byte_a", "byte_b", "byte_c", "byte_d", "byte_e",
            "dac_a_v", "dac_b_v", "dac_c_v", "dac_d_v", "dac_e_v",
            "reserved_byte", "flags_byte", "frame_counter",
            "adc0_raw", "adc1_raw", "adc2_raw", "adc3_raw", "adc4_raw",
            "adc0_mv", "adc1_mv", "adc2_mv", "adc3_mv", "adc4_mv"
        };

        private static readonly HashSet<string> IntColumns = new(StringComparer.Ordinal)
        {
            "byte_a", "byte_b", "byte_c", "byte_d", "byte_e",
            "reserved_byte", "flags_byte", "frame_counter",
            "adc0_raw", "adc1_raw", "adc2_raw", "adc3_raw", "adc4_raw"
        };

        private readonly object _lock = new();
        private readonly Stopwatch _sw = new();
        private BlockingCollection<SupRingLogSample>? _queue;
        private Thread? _writerThread;
        private volatile bool _isRecording;
        private long _rowsLogged;
        private string _dbPath = "";
        private TueUsb.HapticMappingConfig _activeMapping = TueUsb.HapticMappingConfig.Default;

        public bool IsRecording => _isRecording;

        public long RowsLogged => Interlocked.Read(ref _rowsLogged);

        public double ElapsedSeconds => _isRecording ? _sw.Elapsed.TotalSeconds : 0.0;

        public string DbPath
        {
            get { lock (_lock) return _dbPath; }
        }

        public void Start(string dbPath, double txRateHz, double vrefMv = 3300.0, TueUsb.HapticMappingConfig? mapping = null)
        {
            lock (_lock)
            {
                if (_isRecording) return;

                _activeMapping = mapping ?? TueUsb.HapticMappingConfig.Default;
                string fullPath = Path.GetFullPath(dbPath);
                Directory.CreateDirectory(Path.GetDirectoryName(fullPath)!);
                InitDatabase(fullPath, txRateHz, vrefMv, _activeMapping);

                _dbPath = fullPath;
                Interlocked.Exchange(ref _rowsLogged, 0);
                _sw.Restart();
                _queue = new BlockingCollection<SupRingLogSample>();
                _isRecording = true;

                var localQueue = _queue;
                _writerThread = new Thread(() => WriterLoop(fullPath, localQueue))
                {
                    IsBackground = true,
                    Name = "SupRingSqliteLogger"
                };
                _writerThread.Start();
            }
        }

        public void Enqueue(SupRingLogSample sample)
        {
            if (!_isRecording) return;
            var q = _queue;
            if (q == null || q.IsAddingCompleted) return;
            try
            {
                q.Add(sample);
            }
            catch (InvalidOperationException) { }
        }

        public void Stop()
        {
            Thread? threadToJoin;
            BlockingCollection<SupRingLogSample>? qToDispose;
            lock (_lock)
            {
                if (!_isRecording && _writerThread == null) return;
                _isRecording = false;
                _sw.Stop();
                qToDispose = _queue;
                threadToJoin = _writerThread;
                _queue = null;
                _writerThread = null;
            }

            if (qToDispose != null)
            {
                try { qToDispose.CompleteAdding(); } catch (ObjectDisposedException) { }
            }
            threadToJoin?.Join(3000);
            qToDispose?.Dispose();
        }

        private static void InitDatabase(string fullPath, double txRateHz, double vrefMv, TueUsb.HapticMappingConfig map)
        {
            using var conn = new SqliteConnection($"Data Source={fullPath}");
            conn.Open();

            string colDefs = string.Join(",\n", SampleColumns.Select(c =>
                IntColumns.Contains(c) ? $"    [{c}] INTEGER" : $"    [{c}] REAL"));

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
                );";
            cmd.ExecuteNonQuery();

            var meta = new Dictionary<string, string>
            {
                ["created_iso"] = DateTime.UtcNow.ToString("O"),
                ["experiment"] = "sup_ring_manual",
                ["mapping_convention"] = "I01_paper_v01 (-Fx->A, Fz->B, +Fx->C, -Fy->D, +Fy->E)",
                ["tx_rate_hz"] = txRateHz.ToString(CultureInfo.InvariantCulture),
                ["vref_mv"] = vrefMv.ToString(CultureInfo.InvariantCulture),
                ["esp32_v_min"] = map.VMinV.ToString(CultureInfo.InvariantCulture),
                ["esp32_v_max"] = map.VMaxV.ToString(CultureInfo.InvariantCulture),
                ["esp32_f_thresh"] = map.FThreshN.ToString(CultureInfo.InvariantCulture),
                ["esp32_f_rated"] = map.FRatedN.ToString(CultureInfo.InvariantCulture),
                ["esp32_eeprom_saved"] = map.SavedInEeprom ? "1" : "0"
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

        private void WriterLoop(string fullPath, BlockingCollection<SupRingLogSample> queue)
        {
            using var conn = new SqliteConnection($"Data Source={fullPath}");
            conn.Open();

            string cols = string.Join(", ", SampleColumns.Select(c => $"[{c}]"));
            string pars = string.Join(", ", SampleColumns.Select((_, i) => $"$p{i}"));

            using var ins = conn.CreateCommand();
            ins.CommandText = $"INSERT INTO samples ({cols}) VALUES ({pars});";
            var parameters = new SqliteParameter[SampleColumns.Length];
            for (int i = 0; i < SampleColumns.Length; i++)
            {
                parameters[i] = ins.Parameters.Add(
                    $"$p{i}",
                    IntColumns.Contains(SampleColumns[i]) ? SqliteType.Integer : SqliteType.Real);
            }

            var mapSnapshot = _activeMapping;
            var batch = new List<SupRingLogSample>(64);
            while (!queue.IsCompleted)
            {
                if (queue.TryTake(out var first, 50))
                {
                    batch.Add(first);
                    while (batch.Count < 64 && queue.TryTake(out var more))
                        batch.Add(more);

                    using var tx = conn.BeginTransaction();
                    ins.Transaction = tx;
                    foreach (var s in batch)
                    {
                        BindRow(parameters, s, mapSnapshot);
                        ins.ExecuteNonQuery();
                    }
                    tx.Commit();
                    Interlocked.Add(ref _rowsLogged, batch.Count);
                    batch.Clear();
                }
            }
        }

        private static void BindRow(SqliteParameter[] p, SupRingLogSample s, TueUsb.HapticMappingConfig map)
        {
            p[0].Value = s.TimeSec;
            p[1].Value = s.Epoch;
            p[2].Value = s.Fx;
            p[3].Value = s.Fy;
            p[4].Value = s.Fz;
            p[5].Value = s.ShearMag;
            p[6].Value = s.ShearAngleDeg;

            byte[] pl = s.Payload ?? Array.Empty<byte>();
            for (int i = 0; i < 5; i++)
            {
                byte b = i < pl.Length ? pl[i] : (byte)0;
                p[7 + i].Value = (int)b;
                double dacV = map.ComputeVoltage(b * HapticConstants.ForceLsbN);
                p[12 + i].Value = dacV;
            }

            p[17].Value = (int)(pl.Length > 5 ? pl[5] : 0);
            p[18].Value = (int)(pl.Length > 6 ? pl[6] : 0);
            p[19].Value = (int)(pl.Length > 7 ? pl[7] : 0);

            for (int c = 0; c < 5; c++)
                p[20 + c].Value = s.AdcRaw != null && c < s.AdcRaw.Length ? (int)s.AdcRaw[c] : DBNull.Value;
            for (int c = 0; c < 5; c++)
                p[25 + c].Value = s.AdcMv != null && c < s.AdcMv.Length && !double.IsNaN(s.AdcMv[c])
                    ? s.AdcMv[c]
                    : DBNull.Value;
        }

        public void Dispose()
        {
            Stop();
        }
    }
}
