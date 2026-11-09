using System;
using System.Collections.Concurrent;
using System.Globalization;
using System.IO;
using System.Text;
using System.Threading;
using ControlGui.Core.Paxini;

namespace ControlGui.Core.Recording
{
    public class CsvRecorder : IDisposable
    {
        private readonly string _path;
        private readonly double _t0;
        private readonly StreamWriter _writer;
        private readonly BlockingCollection<Tuple<SensorReading, double, float[,]>?> _queue = new();
        private readonly Thread _workerThread;
        private long _rows = 0;

        public long Rows => Interlocked.Read(ref _rows);
        public string Path => _path;

        public CsvRecorder(string path, double t0, string meta = "")
        {
            _path = path;
            _t0 = t0;

            Directory.CreateDirectory(System.IO.Path.GetDirectoryName(_path)!);
            _writer = new StreamWriter(_path, false, Encoding.UTF8, 65536);

            if (!string.IsNullOrEmpty(meta))
            {
                _writer.WriteLine($"# {meta}");
            }

            var cols = RecordingSchema.GetPerSensorColumns();
            _writer.WriteLine(string.Join(",", cols));

            _workerThread = new Thread(ProcessQueue)
            {
                IsBackground = true,
                Name = $"CsvRecorder-{System.IO.Path.GetFileName(path)}"
            };
            _workerThread.Start();
        }

        public void Push(SensorReading reading, double epoch, float[,] taxels)
        {
            if (!_queue.IsAddingCompleted)
            {
                _queue.Add(Tuple.Create(reading, epoch, taxels));
            }
        }

        private void ProcessQueue()
        {
            var sb = new StringBuilder(1024);
            var inv = CultureInfo.InvariantCulture;

            while (!_queue.IsCompleted)
            {
                Tuple<SensorReading, double, float[,]>? item;
                try
                {
                    item = _queue.Take();
                }
                catch (InvalidOperationException)
                {
                    break;
                }

                if (item == null) break;

                var reading = item.Item1;
                double epoch = item.Item2;
                var tax = item.Item3;

                sb.Clear();
                sb.Append((reading.T - _t0).ToString("F6", inv)).Append(',');
                sb.Append(epoch.ToString("F6", inv)).Append(',');

                sb.Append(reading.Fx.ToString("F3", inv)).Append(',');
                sb.Append(reading.Fy.ToString("F3", inv)).Append(',');
                sb.Append(reading.Fz.ToString("F3", inv)).Append(',');
                sb.Append(reading.FMag.ToString("F3", inv)).Append(',');
                sb.Append(reading.FShear.ToString("F3", inv)).Append(',');
                sb.Append(double.IsNaN(reading.Mu) ? "nan" : reading.Mu.ToString("F3", inv)).Append(',');
                sb.Append(double.IsNaN(reading.CopX) ? "nan" : reading.CopX.ToString("F3", inv)).Append(',');
                sb.Append(double.IsNaN(reading.CopY) ? "nan" : reading.CopY.ToString("F3", inv)).Append(',');
                sb.Append(double.IsNaN(reading.ShearDir) ? "nan" : reading.ShearDir.ToString("F3", inv)).Append(',');
                sb.Append(double.IsNaN(reading.Tilt) ? "nan" : reading.Tilt.ToString("F3", inv)).Append(',');
                sb.Append(double.IsNaN(reading.PeakTaxel) ? "nan" : reading.PeakTaxel.ToString("F0", inv)).Append(',');
                sb.Append(reading.PeakFz.ToString("F3", inv)).Append(',');
                sb.Append(reading.ContactTaxels.ToString("F0", inv));

                if (tax != null && tax.GetLength(0) == 52)
                {
                    for (int k = 0; k < 52; k++)
                    {
                        sb.Append(',').Append(tax[k, 0].ToString("F1", inv));
                        sb.Append(',').Append(tax[k, 1].ToString("F1", inv));
                        sb.Append(',').Append(tax[k, 2].ToString("F1", inv));
                    }
                }
                else
                {
                    for (int k = 0; k < 52 * 3; k++)
                    {
                        sb.Append(",nan");
                    }
                }

                _writer.WriteLine(sb.ToString());
                Interlocked.Increment(ref _rows);
            }

            _writer.Flush();
        }

        private bool _stopped = false;

        public long Stop()
        {
            if (_stopped) return Rows;
            _stopped = true;

            _queue.CompleteAdding();
            _workerThread.Join(5000);
            try
            {
                _writer.Flush();
                _writer.Dispose();
            }
            catch { }
            return Rows;
        }

        public void Dispose()
        {
            Stop();
            _queue.Dispose();
        }
    }
}
