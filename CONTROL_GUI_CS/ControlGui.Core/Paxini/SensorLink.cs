using System;
using System.Diagnostics;
using ControlGui.Core.TueUsb;

namespace ControlGui.Core.Paxini
{
    public class SensorLink : IDisposable
    {
        public string Tag { get; }      // "L" or "R"
        public string Port { get; }
        public int BaudRate { get; }
        public byte DeviceAddr { get; set; }
        public string Model { get; }
        public string Label => Tag == "L" ? "Left" : "Right";

        private readonly Action<string> _log;
        private readonly bool _mock;
        private readonly ISerialPortTransport? _customTransport;

        public PaxiniSensorWorker? Worker { get; private set; }

        public Point3D[] Geometry => S2015EliteGeometry.Coordinates;
        public int NPoints => 52;

        public SensorLink(
            string tag,
            string port,
            Action<string> log,
            int baudrate = PaxiniConstants.DefaultBaudRate,
            byte deviceAddr = 0,
            string model = "S2015_elite",
            bool mock = false,
            ISerialPortTransport? customTransport = null)
        {
            Tag = tag;
            Port = port;
            _log = log;
            BaudRate = baudrate;
            DeviceAddr = deviceAddr;
            Model = model;
            _mock = mock;
            _customTransport = customTransport;
        }

        public void Connect()
        {
            if (IsActive()) return;

            Worker = new PaxiniSensorWorker(
                Port,
                _log,
                deviceAddr: DeviceAddr,
                baudRate: BaudRate,
                model: Model,
                mock: _mock,
                customTransport: _customTransport
            );
            Worker.Start();
            _log($"[{Tag}] {Port} @ {BaudRate} baud, {(DeviceAddr != 0 ? $"address 0x{DeviceAddr:X2}" : "address auto-detect")}");
        }

        public void Disconnect()
        {
            if (Worker != null)
            {
                Worker.Stop();
                Worker.Dispose();
                Worker = null;
            }
        }

        public bool IsActive() => Worker != null && Worker.State == "streaming";

        public string State => Worker?.State ?? "disconnected";
        public byte DetectedAddr => Worker?.DeviceAddr ?? 0;
        public string AddrText => DetectedAddr != 0 ? $"0x{DetectedAddr:X2}" : "auto";
        public double RateHz => Worker?.RateHz ?? 0.0;
        public long Frames => Worker?.Frames ?? 0;
        public long Timeouts => Worker?.Timeouts ?? 0;

        public SensorReading Reading => Worker?.LatestReading ?? SensorReading.Empty;

        public void RequestTare(int nFrames = 50) => Worker?.RequestTare(nFrames);
        public void ResetTare() => Worker?.ResetTare();

        public void Dispose()
        {
            Disconnect();
        }
    }
}
