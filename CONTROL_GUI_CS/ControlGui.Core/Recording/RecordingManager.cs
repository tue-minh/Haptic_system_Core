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
        private string _outputDir;
        private UnifiedRecorder? _masterRecorder;
        private readonly Dictionary<string, CsvRecorder> _sensorRecorders = new();
        private readonly Stopwatch _elapsedSw = new();

        public bool IsRecording => _masterRecorder != null;
        public double ElapsedSeconds => _elapsedSw.Elapsed.TotalSeconds;
        public long RowsLogged => _masterRecorder?.Rows ?? 0;
        public List<string> CurrentFiles { get; } = new();

        public string OutputDir
        {
            get => _outputDir;
            set
            {
                _outputDir = value;
                Directory.CreateDirectory(_outputDir);
            }
        }

        public RecordingManager(Action<string> log, string? defaultOutputDir = null)
        {
            _log = log;
            _outputDir = defaultOutputDir ?? Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Unified_Control");
            Directory.CreateDirectory(_outputDir);
        }

        public void Start(
            MotorWorker? motor,
            SensorLink? linkL,
            SensorLink? linkR,
            double rateHz = 100.0)
        {
            if (IsRecording) return;

            string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            CurrentFiles.Clear();

            // 1. Per-sensor recorders
            _sensorRecorders.Clear();
            double t0 = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0;

            if (linkL != null && linkL.IsActive() && linkL.Worker != null)
            {
                string portSanitized = linkL.Port.Replace("/", "_").Replace("\\", "_");
                string pathL = Path.Combine(_outputDir, $"paxini_{stamp}_L_{portSanitized}.csv");
                var recL = new CsvRecorder(pathL, t0, $"PaXini Left Sensor on {linkL.Port}");
                linkL.Worker.OnRecordedRow = recL.Push;
                _sensorRecorders["L"] = recL;
                CurrentFiles.Add(pathL);
            }

            if (linkR != null && linkR.IsActive() && linkR.Worker != null)
            {
                string portSanitized = linkR.Port.Replace("/", "_").Replace("\\", "_");
                string pathR = Path.Combine(_outputDir, $"paxini_{stamp}_R_{portSanitized}.csv");
                var recR = new CsvRecorder(pathR, t0, $"PaXini Right Sensor on {linkR.Port}");
                linkR.Worker.OnRecordedRow = recR.Push;
                _sensorRecorders["R"] = recR;
                CurrentFiles.Add(pathR);
            }

            // 2. Master unified recorder
            string masterPath = Path.Combine(_outputDir, $"data_record_{stamp}.csv");
            CurrentFiles.Insert(0, masterPath);

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

            _masterRecorder = new UnifiedRecorder(masterPath, t0, sampleFn, rateHz, "Unified Motor & Dual Tactile Data Log");
            _elapsedSw.Restart();
            _log($"[recording] Started recording session -> {masterPath}");
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
            long totalRows = _masterRecorder!.Stop();
            _masterRecorder.Dispose();
            _masterRecorder = null;

            if (linkL?.Worker != null) linkL.Worker.OnRecordedRow = null;
            if (linkR?.Worker != null) linkR.Worker.OnRecordedRow = null;

            foreach (var rec in _sensorRecorders.Values)
            {
                rec.Stop();
                rec.Dispose();
            }
            _sensorRecorders.Clear();

            _log($"[recording] Stopped recording session. Total rows: {totalRows}");
            return totalRows;
        }

        public void Dispose()
        {
            Stop();
        }
    }
}
