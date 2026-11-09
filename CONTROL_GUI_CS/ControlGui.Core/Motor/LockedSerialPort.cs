using System;
using ControlGui.Core.TueUsb;

namespace ControlGui.Core.Motor
{
    public class LockedSerialPort : ISerialPortTransport
    {
        private readonly ISerialPortTransport _inner;
        private readonly object _writeLock;

        public LockedSerialPort(ISerialPortTransport inner, object writeLock)
        {
            _inner = inner;
            _writeLock = writeLock;
        }

        public bool IsOpen => _inner.IsOpen;
        public int BytesToRead => _inner.BytesToRead;

        public void Open() => _inner.Open();
        public void Close() => _inner.Close();
        public void DiscardInBuffer() => _inner.DiscardInBuffer();

        public int Read(byte[] buffer, int offset, int count)
        {
            // Only motor worker reads from serial port
            return _inner.Read(buffer, offset, count);
        }

        public void Write(byte[] buffer, int offset, int count)
        {
            lock (_writeLock)
            {
                _inner.Write(buffer, offset, count);
            }
        }

        public void Dispose()
        {
            _inner.Dispose();
        }
    }
}
