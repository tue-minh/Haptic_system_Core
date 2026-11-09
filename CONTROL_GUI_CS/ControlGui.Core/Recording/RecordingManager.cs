using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using ControlGui.Core.Motor;
using ControlGui.Core.Paxini;

namespace ControlGui.Core.Recording
{
    public class RecordingManager : IDisposable
    {
        private readonly Action<string> _log;
        private string _targetPath;
        private SqliteRecorder? _sqliteRecorder;
        private readonly Stopwatch _elapsedSw = new();

        public bool IsRecording => _sqliteRecorder != null;
        public double ElapsedSeconds => _elapsedSw.Elapsed.TotalSeconds;
        public long RowsLogged => _sqliteRecorder?.Rows ?? 0;
        public string CurrentTable => _sqliteRecorder?.TableName ?? "";
        public string DbPath => ResolveDbPath(_targetPath);

        public string OutputDir
        {
            get => _targetPath;
            set
            {
                _targetPath = value;
                string dir = Path.GetDirectoryName(Path.GetFullPath(DbPath))!;
                Directory.CreateDirectory(dir);
            }
        }

        public RecordingManager(Action<string> log, string? defaultPath = null)
        {
            _log = log;
            _targetPath = defaultPath ?? Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Unified_Control", "recordings.db");
            string dir = Path.GetDirectoryName(Path.GetFullPath(DbPath))!;
            Directory.CreateDirectory(dir);
        }

        private static string ResolveDbPath(string inputPath)
        {
            if (string.IsNullOrWhiteSpace(inputPath))
            {
                return Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Unified_Control", "recordings.db");
            }

            if (inputPath.EndsWith(".db", StringComparison.OrdinalIgnoreCase) ||
                inputPath.EndsWith(".sqlite", StringComparison.OrdinalIgnoreCase))
            {
                return inputPath;
            }

            return Path.Combine(inputPath, "recordings.db");
        }

        public void Start(
            MotorWorker? motor,
            SensorLink? linkL,
            SensorLink? linkR,
            double rateHz = 100.0)
        {
            if (IsRecording) return;

            string dbFile = DbPath;
            string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            string tableName = $"rec_{stamp}";
            double t0 = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0;

            Func<List<double>> sampleFn = () =>
            {
                var row = new List<double>();
                // Lead columns
                var motorSnap = motor?.Snapshot();
                row.Add(motorSnap?.X ?? double.NaN);
                row.Add(motorSnap?.Y ?? double.NaN);
                row.Add(motorSnap?.Z ?? double.NaN);
                row.Add(motorSnap?.ServoPos ?? double.NaN);
                row.Add(motorSnap?.ServoVoltV ?? double.NaN);

                // Left sensor block (13 metrics + 52*3 taxels)
                AddSensorBlock(row, linkL);

                // Right sensor block (13 metrics + 52*3 taxels)
                AddSensorBlock(row, linkR);

                return row;
            };

            _sqliteRecorder = new SqliteRecorder(
                dbPath: dbFile,
                tableName: tableName,
                t0: t0,
                sampleFn: sampleFn,
                rateHz: rateHz,
                meta: $"Dual Tactile & Motor Session at {DateTime.Now:yyyy-MM-dd HH:mm:ss}"
            );

            _elapsedSw.Restart();
            _log($"[recording] Started recording -> SQLite DB: '{Path.GetFileName(dbFile)}', Table: '[{tableName}]'");
        }

        private static void AddSensorBlock(List<double> row, SensorLink? link)
        {
            if (link == null || !link.IsActive())
            {
                for (int i = 0; i < 13 + 52 * 3; i++) row.Add(double.NaN);
                return;
            }

            var reading = link.Reading;
            row.Add(reading.Fx);
            row.Add(reading.Fy);
            row.Add(reading.Fz);
            row.Add(reading.FMag);
            row.Add(reading.FShear);
            row.Add(reading.Mu);
            row.Add(reading.CopX);
            row.Add(reading.CopY);
            row.Add(reading.ShearDir);
            row.Add(reading.Tilt);
            row.Add(reading.PeakTaxel);
            row.Add(reading.PeakFz);
            row.Add(reading.ContactTaxels);

            var tax = reading.Taxels;
            if (tax != null && tax.GetLength(0) == 52)
            {
                for (int k = 0; k < 52; k++)
                {
                    row.Add(tax[k, 0]);
                    row.Add(tax[k, 1]);
                    row.Add(tax[k, 2]);
                }
            }
            else
            {
                for (int k = 0; k < 52 * 3; k++) row.Add(double.NaN);
            }
        }

        public long Stop(SensorLink? linkL = null, SensorLink? linkR = null)
        {
            if (!IsRecording) return 0;

            _elapsedSw.Stop();
            string tbl = _sqliteRecorder!.TableName;
            long totalRows = _sqliteRecorder.Stop();
            _sqliteRecorder.Dispose();
            _sqliteRecorder = null;

            _log($"[recording] Stopped recording session table '[{tbl}]'. Total rows logged: {totalRows}");
            return totalRows;
        }

        public void Dispose()
        {
            Stop();
        }
    }
}
