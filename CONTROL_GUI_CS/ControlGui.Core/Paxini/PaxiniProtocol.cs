using System;
using System.Diagnostics;
using System.Globalization;
using System.Threading;
using ControlGui.Core.TueUsb;

namespace ControlGui.Core.Paxini
{
    public static class PaxiniProtocol
    {
        public static byte CalcLrc(ReadOnlySpan<byte> data)
        {
            int sum = 0;
            for (int i = 0; i < data.Length; i++)
            {
                sum += data[i];
            }
            return (byte)((-sum) & 0xFF);
        }

        public static byte CalcUsbLrc(ReadOnlySpan<byte> data)
        {
            int sum = 0;
            for (int i = 0; i < data.Length; i++)
            {
                sum = (sum + data[i]) & 0xFF;
            }
            return (byte)(((~sum) + 1) & 0xFF);
        }

        public static byte[] HexToBytes(string hex)
        {
            string clean = hex.Replace(" ", "").Replace("-", "");
            byte[] bytes = new byte[clean.Length / 2];
            for (int i = 0; i < bytes.Length; i++)
            {
                bytes[i] = byte.Parse(clean.Substring(i * 2, 2), NumberStyles.HexNumber);
            }
            return bytes;
        }

        public static byte[] BuildUsbFrame(string hexStr)
        {
            byte[] frame = HexToBytes(hexStr);
            byte lrc = CalcUsbLrc(frame);
            byte[] result = new byte[frame.Length + 1];
            Array.Copy(frame, 0, result, 0, frame.Length);
            result[result.Length - 1] = lrc;
            return result;
        }

        public static byte[] BuildCalibrationFrame(byte addr)
        {
            return BuildUsbFrame($"55 AA 0A 00 {addr:X2} 00 79 03 00 00 00 01 00 01");
        }

        public static byte[] BuildResultantReq(byte addr)
        {
            return BuildUsbFrame($"55 AA 09 00 {addr:X2} 00 FB F0 03 00 00 03 00");
        }

        public static byte[] BuildDistributedReq(byte addr, int nPoints = 52)
        {
            int distLen = nPoints * 3;
            byte lo = (byte)(distLen & 0xFF);
            byte hi = (byte)((distLen >> 8) & 0xFF);
            return BuildUsbFrame($"55 AA 09 00 {addr:X2} 00 FB 0E 04 00 00 {lo:X2} {hi:X2}");
        }

        public static byte[] TransactUsb(ISerialPortTransport port, byte[] frame, int minRespLen, double timeoutSec = 0.5)
        {
            port.DiscardInBuffer();
            port.Write(frame, 0, frame.Length);
            Thread.Sleep(10);

            byte[] buffer = new byte[1024];
            int totalRead = 0;
            var sw = Stopwatch.StartNew();

            while (sw.Elapsed.TotalSeconds < timeoutSec)
            {
                int avail = port.BytesToRead;
                if (avail > 0)
                {
                    int r = port.Read(buffer, totalRead, Math.Min(avail, buffer.Length - totalRead));
                    totalRead += r;

                    // Look for AA 55 header
                    for (int i = 0; i <= totalRead - 2; i++)
                    {
                        if (buffer[i] == 0xAA && buffer[i + 1] == 0x55)
                        {
                            if (totalRead - i >= minRespLen)
                            {
                                byte[] resp = new byte[minRespLen];
                                Array.Copy(buffer, i, resp, 0, minRespLen);
                                return resp;
                            }
                        }
                    }
                }
                else
                {
                    Thread.Sleep(1);
                }
            }

            byte[] partial = new byte[totalRead];
            Array.Copy(buffer, 0, partial, 0, totalRead);
            return partial;
        }

        public static byte? ProbeDeviceAddress(
            ISerialPortTransport port,
            Action<string>? log = null,
            byte? preferred = null,
            byte[]? addresses = null,
            int passes = 2,
            double timeoutSec = 0.25)
        {
            byte[] candidates = addresses ?? PaxiniConstants.ProbeAddresses;
            var order = new System.Collections.Generic.List<byte>(candidates);
            if (preferred.HasValue && preferred.Value != 0)
            {
                order.Remove(preferred.Value);
                order.Insert(0, preferred.Value);
            }

            for (int sweep = 1; sweep <= Math.Max(1, passes); sweep++)
            {
                foreach (byte addr in order)
                {
                    byte[] calibReq = BuildCalibrationFrame(addr);
                    byte[] resp = TransactUsb(port, calibReq, 15, timeoutSec);
                    if (resp.Length >= 2 && resp[0] == 0xAA && resp[1] == 0x55)
                    {
                        log?.Invoke($"Device address 0x{addr:X2} answered (sweep {sweep})");
                        return addr;
                    }
                }
                log?.Invoke($"No reply from tested addresses (sweep {sweep}/{passes})");
            }

            return null;
        }
    }
}
