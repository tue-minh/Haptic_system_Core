using System;
using System.Diagnostics;
using System.Threading;
using ControlGui.Core.TueUsb;

namespace ControlGui.Core.Paxini
{
    public class PaxiniSensorWorker : IDisposable
    {
        private readonly Thread _thread;
        private readonly CancellationTokenSource _cts = new();
        private readonly Action<string> _log;
        private ISerialPortTransport? _transport;

        public string Port { get; }
        public int BaudRate { get; }
        public byte DeviceAddr { get; private set; }
        public string Model { get; }
        public bool Mock { get; }

        public string State { get; private set; } = "disconnected";
        public string Error { get; private set; } = "";
        public long Frames { get; private set; } = 0;
        public double RateHz { get; private set; } = 0.0;
        public long Timeouts { get; private set; } = 0;

        public RingBuffer Ring { get; } = new(65536, 14);
        public SensorReading LatestReading { get; private set; } = SensorReading.Empty;
        public Action<SensorReading, double, float[,]>? OnRecordedRow { get; set; }

        private double _tareResFx = 0;
        private double _tareResFy = 0;
        private double _tareResFz = 0;
        private readonly float[,] _tareTax = new float[52, 3];
        private int _tareLeft = 0;
        private int _tareN = 50;
        private double _tareAccFx = 0;
        private double _tareAccFy = 0;
        private double _tareAccFz = 0;
        private readonly double[,] _tareAccTax = new double[52, 3];

        private static readonly Stopwatch AppTimer = Stopwatch.StartNew();

        public PaxiniSensorWorker(
            string port,
            Action<string> log,
            byte deviceAddr = 0,
            int baudRate = PaxiniConstants.DefaultBaudRate,
            string model = "S2015_elite",
            bool mock = false,
            ISerialPortTransport? customTransport = null)
        {
            Port = port;
            _log = log;
            DeviceAddr = deviceAddr;
            BaudRate = baudRate;
            Model = model;
            Mock = mock || port == "MOCK_PORT" || string.IsNullOrEmpty(port);
            _transport = customTransport;

            _thread = new Thread(Run)
            {
                IsBackground = true,
                Name = $"PaxiniWorker-{port}"
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

        public void RequestTare(int nFrames = 50)
        {
            _tareN = nFrames;
            _tareAccFx = 0;
            _tareAccFy = 0;
            _tareAccFz = 0;
            Array.Clear(_tareAccTax, 0, _tareAccTax.Length);
            _tareLeft = nFrames;
        }

        public void ResetTare()
        {
            _tareLeft = 0;
            _tareResFx = 0;
            _tareResFy = 0;
            _tareResFz = 0;
            Array.Clear(_tareTax, 0, _tareTax.Length);
        }

        private void Run()
        {
            State = "connecting";
            _log($"[{Port}] Starting PaXini worker thread @ {BaudRate} baud");

            if (Mock)
            {
                DeviceAddr = DeviceAddr == 0 ? (byte)0x01 : DeviceAddr;
                State = "streaming";
                _log($"[{Port}] Running in simulation (mock) mode, address 0x{DeviceAddr:X2}");
                RunMockStream();
                State = "stopped";
                return;
            }

            try
            {
                if (_transport == null)
                {
                    _transport = new SystemSerialPortTransport(Port, BaudRate, 100, 100);
                }

                if (!_transport.IsOpen)
                {
                    _transport.Open();
                }

                Thread.Sleep(300);
                _log($"[{Port}] Port opened. Probing for device address...");

                byte? found = PaxiniProtocol.ProbeDeviceAddress(
                    _transport,
                    msg => _log($"[{Port}] {msg}"),
                    DeviceAddr == 0 ? null : DeviceAddr,
                    timeoutSec: 0.25
                );

                if (!found.HasValue)
                {
                    State = "error";
                    Error = "No sensor answered on addresses 01-08";
                    _log($"[{Port}] ERROR: {Error}");
                    return;
                }

                DeviceAddr = found.Value;
                State = "streaming";
                _log($"[{Port}] Connected on device address 0x{DeviceAddr:X2}. Starting acquisition...");

                RunRealStream(_transport);
            }
            catch (Exception ex)
            {
                State = "error";
                Error = ex.Message;
                _log($"[{Port}] Error: {ex.Message}");
            }
            finally
            {
                try { _transport?.Close(); } catch { }
                if (State != "error") State = "stopped";
                _log($"[{Port}] Port closed");
            }
        }

        private void RunRealStream(ISerialPortTransport transport)
        {
            byte[] resReq = PaxiniProtocol.BuildResultantReq(DeviceAddr);
            byte[] distReq = PaxiniProtocol.BuildDistributedReq(DeviceAddr, 52);

            var rateSw = Stopwatch.StartNew();
            int rateCount = 0;

            float[,] tax = new float[52, 3];
            double[] ringRow = new double[14];

            while (!_cts.Token.IsCancellationRequested)
            {
                double t = AppTimer.Elapsed.TotalSeconds;
                double epoch = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0;

                // 1. Read Resultant
                byte[] rResp = PaxiniProtocol.TransactUsb(transport, resReq, 14 + 3 + 1, 0.1);
                if (rResp.Length < 17)
                {
                    Timeouts++;
                    continue;
                }

                int rOffset = rResp.Length - 4;
                sbyte fxRaw = (sbyte)rResp[rOffset];
                sbyte fyRaw = (sbyte)rResp[rOffset + 1];
                byte fzRaw = rResp[rOffset + 2];

                double fx = fxRaw * PaxiniConstants.ForceLsbN;
                double fy = fyRaw * PaxiniConstants.ForceLsbN;
                double fz = fzRaw * PaxiniConstants.ForceLsbN;

                // 2. Read Distributed (52 taxels * 3 = 156 bytes)
                byte[] dResp = PaxiniProtocol.TransactUsb(transport, distReq, 14 + 156 + 1, 0.1);
                if (dResp.Length < 14 + 156)
                {
                    Timeouts++;
                    continue;
                }

                int dOffset = dResp.Length - 156 - 1;
                for (int i = 0; i < 52; i++)
                {
                    int idx = dOffset + i * 3;
                    tax[i, 0] = (float)((sbyte)dResp[idx] * PaxiniConstants.ForceLsbN);
                    tax[i, 1] = (float)((sbyte)dResp[idx + 1] * PaxiniConstants.ForceLsbN);
                    tax[i, 2] = (float)(dResp[idx + 2] * PaxiniConstants.ForceLsbN);
                }

                // Tare accumulation
                if (_tareLeft > 0)
                {
                    _tareAccFx += fx;
                    _tareAccFy += fy;
                    _tareAccFz += fz;
                    for (int i = 0; i < 52; i++)
                    {
                        _tareAccTax[i, 0] += tax[i, 0];
                        _tareAccTax[i, 1] += tax[i, 1];
                        _tareAccTax[i, 2] += tax[i, 2];
                    }
                    _tareLeft--;
                    if (_tareLeft == 0)
                    {
                        _tareResFx = _tareAccFx / _tareN;
                        _tareResFy = _tareAccFy / _tareN;
                        _tareResFz = _tareAccFz / _tareN;
                        for (int i = 0; i < 52; i++)
                        {
                            _tareTax[i, 0] = (float)(_tareAccTax[i, 0] / _tareN);
                            _tareTax[i, 1] = (float)(_tareAccTax[i, 1] / _tareN);
                            _tareTax[i, 2] = (float)(_tareAccTax[i, 2] / _tareN);
                        }
                    }
                }

                fx -= _tareResFx;
                fy -= _tareResFy;
                fz -= _tareResFz;
                for (int i = 0; i < 52; i++)
                {
                    tax[i, 0] -= _tareTax[i, 0];
                    tax[i, 1] -= _tareTax[i, 1];
                    tax[i, 2] -= _tareTax[i, 2];
                }

                // Compute metrics
                var reading = SensorMetrics.Compute(
                    t, epoch, fx, fy, fz, tax, S2015EliteGeometry.Coordinates,
                    PaxiniConstants.TaxelThresholdN, PaxiniConstants.ForceThresholdN);

                // Populate RingBuffer row: [t, fx, fy, fz, f_mag, f_shear, mu, cop_x, cop_y, shear_dir, tilt, peak_idx, peak_fz, contact_n]
                ringRow[0] = t;
                ringRow[1] = fx;
                ringRow[2] = fy;
                ringRow[3] = fz;
                ringRow[4] = reading.FMag;
                ringRow[5] = reading.FShear;
                ringRow[6] = reading.Mu;
                ringRow[7] = reading.CopX;
                ringRow[8] = reading.CopY;
                ringRow[9] = reading.ShearDir;
                ringRow[10] = reading.Tilt;
                ringRow[11] = reading.PeakTaxel;
                ringRow[12] = reading.PeakFz;
                ringRow[13] = reading.ContactTaxels;

                Ring.Append(ringRow);
                LatestReading = reading;
                OnRecordedRow?.Invoke(reading, epoch, tax);

                Frames++;
                rateCount++;
                if (rateSw.ElapsedMilliseconds >= 1000)
                {
                    RateHz = rateCount / (rateSw.ElapsedMilliseconds / 1000.0);
                    rateCount = 0;
                    rateSw.Restart();
                }
            }
        }

        private void RunMockStream()
        {
            var sw = Stopwatch.StartNew();
            float[,] tax = new float[52, 3];
            double[] ringRow = new double[14];

            while (!_cts.Token.IsCancellationRequested)
            {
                double t = AppTimer.Elapsed.TotalSeconds;
                double epoch = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0;

                // Generate simulated gentle sine pressure pattern
                double wave = Math.Max(0.0, Math.Sin(t * 1.5));
                double fx = 0.5 * Math.Cos(t * 0.8) * wave;
                double fy = 0.8 * Math.Sin(t * 0.8) * wave;
                double fz = 4.5 * wave;

                for (int i = 0; i < 52; i++)
                {
                    // Center taxels feel more force
                    double dist = Math.Sqrt(Math.Pow(S2015EliteGeometry.Coordinates[i].X, 2) + Math.Pow(S2015EliteGeometry.Coordinates[i].Y - 8.0, 2));
                    float p = (float)Math.Max(0.0, (1.2 * wave) * Math.Exp(-dist / 8.0));
                    tax[i, 0] = (float)(fx / 52.0);
                    tax[i, 1] = (float)(fy / 52.0);
                    tax[i, 2] = p;
                }

                var reading = SensorMetrics.Compute(
                    t, epoch, fx, fy, fz, tax, S2015EliteGeometry.Coordinates,
                    PaxiniConstants.TaxelThresholdN, PaxiniConstants.ForceThresholdN);

                ringRow[0] = t;
                ringRow[1] = fx;
                ringRow[2] = fy;
                ringRow[3] = fz;
                ringRow[4] = reading.FMag;
                ringRow[5] = reading.FShear;
                ringRow[6] = reading.Mu;
                ringRow[7] = reading.CopX;
                ringRow[8] = reading.CopY;
                ringRow[9] = reading.ShearDir;
                ringRow[10] = reading.Tilt;
                ringRow[11] = reading.PeakTaxel;
                ringRow[12] = reading.PeakFz;
                ringRow[13] = reading.ContactTaxels;

                Ring.Append(ringRow);
                LatestReading = reading;
                OnRecordedRow?.Invoke(reading, epoch, tax);

                Frames++;
                RateHz = 60.0;

                Thread.Sleep(16); // ~60 Hz simulation
            }
        }

        public void Dispose()
        {
            Stop();
            _transport?.Dispose();
        }
    }
}
