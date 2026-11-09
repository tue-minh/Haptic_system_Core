using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Threading;
using Microsoft.Data.Sqlite;

namespace ControlGui.Core.Recording
{
    public class SqliteRecorder : IDisposable
    {
        private readonly string _dbPath;
        private readonly string _tableName;
        private readonly double _t0;
        private readonly double _rateHz;
        private readonly Func<List<double>> _sampleFn;
        private readonly string _meta;

        private readonly BlockingCollection<List<double>?> _queue = new();
        private readonly Thread _samplerThread;
        private readonly Thread _writerThread;
        private readonly CancellationTokenSource _cts = new();
        private long _rows = 0;
        private bool _stopped = false;

        public long Rows => Interlocked.Read(ref _rows);
        public string DbPath => _dbPath;
        public string TableName => _tableName;

        public SqliteRecorder(
            string dbPath,
            string tableName,
            double t0,
            Func<List<double>> sampleFn,
            double rateHz = 100.0,
            string meta = "")
        {
            _dbPath = dbPath;
            _tableName = tableName;
            _t0 = t0;
            _rateHz = Math.Max(1.0, rateHz);
            _sampleFn = sampleFn;
            _meta = meta;

            Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(_dbPath))!);

            InitDatabaseAndTable();

            _samplerThread = new Thread(SamplerLoop)
            {
                IsBackground = true,
                Name = $"SqliteRecorder-sampler-{_tableName}"
            };

            _writerThread = new Thread(WriterLoop)
            {
                IsBackground = true,
                Name = $"SqliteRecorder-writer-{_tableName}"
            };

            _samplerThread.Start();
            _writerThread.Start();
        }

        private void InitDatabaseAndTable()
        {
            using var conn = new SqliteConnection($"Data Source={_dbPath};Cache=Shared;");
            conn.Open();

            using var cmdPragma = conn.CreateCommand();
            cmdPragma.CommandText = @"
                PRAGMA journal_mode = WAL;
                PRAGMA synchronous = NORMAL;
                PRAGMA temp_store = MEMORY;
            ";
            cmdPragma.ExecuteNonQuery();

            // 1. Session catalog table
            using var cmdCatalog = conn.CreateCommand();
            cmdCatalog.CommandText = @"
                CREATE TABLE IF NOT EXISTS _recording_sessions (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    table_name TEXT NOT NULL UNIQUE,
                    created_at TEXT NOT NULL,
                    sample_rate_hz REAL,
                    row_count INTEGER DEFAULT 0,
                    duration_s REAL DEFAULT 0,
                    meta TEXT
                );
            ";
            cmdCatalog.ExecuteNonQuery();

            // Insert initial record into catalog
            using var cmdInsertSession = conn.CreateCommand();
            cmdInsertSession.CommandText = @"
                INSERT OR IGNORE INTO _recording_sessions (table_name, created_at, sample_rate_hz, meta)
                VALUES ($tbl, $created, $rate, $meta);
            ";
            cmdInsertSession.Parameters.AddWithValue("$tbl", _tableName);
            cmdInsertSession.Parameters.AddWithValue("$created", DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss"));
            cmdInsertSession.Parameters.AddWithValue("$rate", _rateHz);
            cmdInsertSession.Parameters.AddWithValue("$meta", _meta);
            cmdInsertSession.ExecuteNonQuery();

            // 2. Telemetry table named at timestamp
            var cols = RecordingSchema.GetMasterColumns();
            var colDefs = string.Join(",\n", cols.Select(c => $"    [{c}] REAL"));

            using var cmdCreateTbl = conn.CreateCommand();
            cmdCreateTbl.CommandText = $@"
                CREATE TABLE IF NOT EXISTS [{_tableName}] (
                    row_id INTEGER PRIMARY KEY AUTOINCREMENT,
{colDefs}
                );
            ";
            cmdCreateTbl.ExecuteNonQuery();
        }

        private void SamplerLoop()
        {
            var sw = Stopwatch.StartNew();
            long periodTicks = (long)(Stopwatch.Frequency / _rateHz);
            long nextTick = sw.ElapsedTicks;

            while (!_cts.Token.IsCancellationRequested)
            {
                long current = sw.ElapsedTicks;
                if (current < nextTick)
                {
                    long remMs = (nextTick - current) * 1000 / Stopwatch.Frequency;
                    if (remMs > 1)
                    {
                        Thread.Sleep((int)(remMs - 1));
                    }
                    while (sw.ElapsedTicks < nextTick)
                    {
                        Thread.SpinWait(10);
                    }
                }
                nextTick += periodTicks;
                if (nextTick < sw.ElapsedTicks)
                {
                    nextTick = sw.ElapsedTicks + periodTicks;
                }

                double t = sw.Elapsed.TotalSeconds;
                double epoch = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0;

                List<double> values;
                try
                {
                    values = _sampleFn();
                }
                catch
                {
                    values = new List<double>();
                }

                // Row format: [t_rel_s, epoch_s, Stepper_X, Stepper_Y, Stepper_Z, Gripper_Pos, Gripper_Volt, L_metrics..., R_metrics...]
                var fullRow = new List<double> { t - _t0, epoch };
                fullRow.AddRange(values);

                if (!_queue.IsAddingCompleted)
                {
                    _queue.Add(fullRow);
                }
            }

            _queue.CompleteAdding();
        }

        private void WriterLoop()
        {
            using var conn = new SqliteConnection($"Data Source={_dbPath};Cache=Shared;");
            conn.Open();

            using var cmdPragma = conn.CreateCommand();
            cmdPragma.CommandText = "PRAGMA synchronous = NORMAL;";
            cmdPragma.ExecuteNonQuery();

            var cols = RecordingSchema.GetMasterColumns();
            string colList = string.Join(", ", cols.Select(c => $"[{c}]"));
            string paramList = string.Join(", ", Enumerable.Range(0, cols.Count).Select(i => $"$p{i}"));

            using var cmd = conn.CreateCommand();
            cmd.CommandText = $"INSERT INTO [{_tableName}] ({colList}) VALUES ({paramList});";

            for (int i = 0; i < cols.Count; i++)
            {
                var p = cmd.CreateParameter();
                p.ParameterName = $"$p{i}";
                p.SqliteType = SqliteType.Real;
                cmd.Parameters.Add(p);
            }

            void BindRow(List<double> row)
            {
                for (int i = 0; i < cols.Count; i++)
                {
                    double val = i < row.Count ? row[i] : double.NaN;
                    cmd.Parameters[i].Value = double.IsNaN(val) ? (object)DBNull.Value : val;
                }
            }

            while (!_queue.IsCompleted)
            {
                List<double>? firstRow;
                try
                {
                    firstRow = _queue.Take();
                }
                catch (InvalidOperationException)
                {
                    break;
                }

                if (firstRow == null) break;

                using var tx = conn.BeginTransaction();
                cmd.Transaction = tx;

                BindRow(firstRow);
                cmd.ExecuteNonQuery();
                Interlocked.Increment(ref _rows);

                int drained = 0;
                while (drained < 200 && _queue.TryTake(out var nextRow))
                {
                    if (nextRow == null) break;
                    BindRow(nextRow);
                    cmd.ExecuteNonQuery();
                    Interlocked.Increment(ref _rows);
                    drained++;
                }

                tx.Commit();
            }

            // Update session catalog summary
            try
            {
                using var cmdUpdate = conn.CreateCommand();
                cmdUpdate.CommandText = @"
                    UPDATE _recording_sessions
                    SET row_count = $rows, duration_s = (SELECT MAX(t_rel_s) FROM [" + _tableName + @"])
                    WHERE table_name = $tbl;
                ";
                cmdUpdate.Parameters.AddWithValue("$rows", Rows);
                cmdUpdate.Parameters.AddWithValue("$tbl", _tableName);
                cmdUpdate.ExecuteNonQuery();
            }
            catch { }
        }

        public long Stop()
        {
            if (_stopped) return Rows;
            _stopped = true;

            _cts.Cancel();
            _samplerThread.Join(5000);
            _writerThread.Join(5000);

            return Rows;
        }

        public void Dispose()
        {
            Stop();
            _queue.Dispose();
        }
    }
}
