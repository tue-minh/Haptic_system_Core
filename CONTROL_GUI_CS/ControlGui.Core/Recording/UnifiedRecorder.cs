using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Text;
using System.Threading;

namespace ControlGui.Core.Recording
{
    public class UnifiedRecorder : IDisposable
    {
        private readonly string _path;
        private readonly double _t0;
        private readonly double _rateHz;
        private readonly Func<List<double>> _sampleFn;
        private readonly StreamWriter _writer;

        private readonly BlockingCollection<List<double>?> _queue = new();
        private readonly Thread _samplerThread;
        private readonly Thread _writerThread;
        private readonly CancellationTokenSource _cts = new();
        private long _rows = 0;

        public long Rows => Interlocked.Read(ref _rows);
        public string Path => _path;

        public UnifiedRecorder(
            string path,
            double t0,
            Func<List<double>> sampleFn,
            double rateHz = 100.0,
            string meta = "")
        {
            _path = path;
            _t0 = t0;
            _rateHz = Math.Max(1.0, rateHz);
            _sampleFn = sampleFn;

            Directory.CreateDirectory(System.IO.Path.GetDirectoryName(_path)!);
            _writer = new StreamWriter(_path, false, Encoding.UTF8, 65536);

            if (!string.IsNullOrEmpty(meta))
            {
                _writer.WriteLine($"# {meta}");
            }

            var cols = RecordingSchema.GetMasterColumns();
            _writer.WriteLine(string.Join(",", cols));

            _samplerThread = new Thread(SamplerLoop)
            {
                IsBackground = true,
                Name = "UnifiedRecorder-sampler"
            };

            _writerThread = new Thread(WriterLoop)
            {
                IsBackground = true,
                Name = "UnifiedRecorder-writer"
            };

            _samplerThread.Start();
            _writerThread.Start();
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
            var sb = new StringBuilder(2048);
            var inv = CultureInfo.InvariantCulture;

            while (!_queue.IsCompleted)
            {
                List<double>? row;
                try
                {
                    row = _queue.Take();
                }
                catch (InvalidOperationException)
                {
                    break;
                }

                if (row == null) break;

                sb.Clear();
                for (int i = 0; i < row.Count; i++)
                {
                    if (i > 0) sb.Append(',');
                    double val = row[i];
                    if (double.IsNaN(val))
                    {
                        sb.Append("nan");
                    }
                    else if (i < 2) // timestamps
                    {
                        sb.Append(val.ToString("F6", inv));
                    }
                    else if (i >= 2 && i < 5) // Stepper XYZ
                    {
                        sb.Append(val.ToString("F4", inv));
                    }
                    else
                    {
                        sb.Append(val.ToString("F3", inv));
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

            _cts.Cancel();
            _samplerThread.Join(5000);
            _writerThread.Join(5000);
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
