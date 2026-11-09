using System;
using System.Buffers.Binary;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO.Ports;
using System.Threading;

namespace ControlGui.Core.TueUsb
{
    public class TueUsbException : Exception
    {
        public ErrorCode Code { get; }

        public TueUsbException(ErrorCode code, string message = "")
            : base($"MCU error [{code}] (0x{(byte)code:X2}): {(string.IsNullOrEmpty(message) ? code.Describe() : message)}")
        {
            Code = code;
        }
    }

    public interface ISerialPortTransport : IDisposable
    {
        bool IsOpen { get; }
        int BytesToRead { get; }
        void Open();
        void Close();
        int Read(byte[] buffer, int offset, int count);
        void Write(byte[] buffer, int offset, int count);
        void DiscardInBuffer();
    }

    public class SystemSerialPortTransport : ISerialPortTransport
    {
        private readonly SerialPort _port;

        public SystemSerialPortTransport(string portName, int baudRate, int readTimeoutMs = 500, int writeTimeoutMs = 500)
        {
            _port = new SerialPort(portName, baudRate, Parity.None, 8, StopBits.One)
            {
                ReadTimeout = readTimeoutMs,
                WriteTimeout = writeTimeoutMs,
                DtrEnable = true,
                RtsEnable = true
            };
        }

        public bool IsOpen => _port.IsOpen;
        public int BytesToRead => _port.BytesToRead;
        public void Open() => _port.Open();
        public void Close() => _port.Close();
        public int Read(byte[] buffer, int offset, int count) => _port.Read(buffer, offset, count);
        public void Write(byte[] buffer, int offset, int count) => _port.Write(buffer, offset, count);
        public void DiscardInBuffer() => _port.DiscardInBuffer();
        public void Dispose() => _port.Dispose();
    }

    public class TueUsbClient : IDisposable
    {
        private readonly object _lock = new object();
        private readonly FrameParser _parser = new FrameParser();
        private byte _seq = 0;
        private ISerialPortTransport? _transport;
        private bool _isConnected = false;

        public string Port { get; set; }
        public int BaudRate { get; set; }
        public double TimeoutSeconds { get; set; }
        public bool Mock { get; set; }

        // Mock state
        private int _mockPosXUm = 0;
        private int _mockPosYUm = 0;
        private int _mockPosZUm = 0;
        private bool _mockStepperEnabled = true;
        private byte _mockLedState = 0;
        private readonly Stopwatch _mockUptime = Stopwatch.StartNew();
        private readonly Dictionary<byte, short> _mockServoPos = new() { { 1, 2048 }, { 2, 2048 } };

        public Action<Frame>? OnTx { get; set; }
        public Action<Frame>? OnRx { get; set; }
        public Action<Frame>? OnAsync { get; set; }

        public TueUsbClient(string? port = null, int baudrate = 1000000, double timeout = 0.5, bool mock = false)
        {
            Port = port ?? "";
            BaudRate = baudrate;
            TimeoutSeconds = timeout;
            Mock = mock;
        }

        public void SetCustomTransport(ISerialPortTransport transport)
        {
            lock (_lock)
            {
                _transport = transport;
            }
        }

        public static string[] ListPorts()
        {
            try
            {
                return SerialPort.GetPortNames();
            }
            catch
            {
                return new[] { "MOCK_PORT" };
            }
        }

        public void Connect(string? port = null)
        {
            if (port != null) Port = port;

            lock (_lock)
            {
                if (_isConnected) return;

                if (Mock || Port == "MOCK_PORT" || string.IsNullOrEmpty(Port))
                {
                    Mock = true;
                    _isConnected = true;
                    return;
                }

                if (_transport == null)
                {
                    _transport = new SystemSerialPortTransport(Port, BaudRate, (int)(TimeoutSeconds * 1000), (int)(TimeoutSeconds * 1000));
                }

                if (!_transport.IsOpen)
                {
                    _transport.Open();
                }

                _parser.Reset();
                _isConnected = true;
            }
        }

        public void Disconnect()
        {
            lock (_lock)
            {
                if (!_isConnected) return;
                try
                {
                    _transport?.Close();
                }
                catch { }
                _transport = null;
                _isConnected = false;
            }
        }

        public bool IsConnected => _isConnected;

        private byte NextSeq()
        {
            return _seq++;
        }

        public Frame Transact(Frame req, double? timeoutSec = null)
        {
            double timeout = timeoutSec ?? TimeoutSeconds;

            lock (_lock)
            {
                if (!_isConnected)
                    throw new InvalidOperationException("TueUsbClient is not connected");

                req.Seq = NextSeq();
                OnTx?.Invoke(req);

                if (Mock)
                {
                    var mockResponses = SimulateMockResponse(req);
                    foreach (var resp in mockResponses)
                    {
                        OnRx?.Invoke(resp);
                    }
                    if (mockResponses.Count > 0)
                    {
                        var last = mockResponses[0];
                        if (last.Type == FrameType.RespErr)
                        {
                            var err = last.Payload.Length > 0 ? (ErrorCode)last.Payload[0] : ErrorCode.ErrInternal;
                            throw new TueUsbException(err);
                        }
                        return last;
                    }
                    throw new TimeoutException("Mock response generation failed");
                }

                if (_transport == null || !_transport.IsOpen)
                    throw new InvalidOperationException("Serial transport is unavailable or closed");

                _transport.DiscardInBuffer();
                byte[] rawReq = req.ToBytes();
                _transport.Write(rawReq, 0, rawReq.Length);

                // Receive loop
                var sw = Stopwatch.StartNew();
                byte[] rxBuf = new byte[64];
                _parser.Reset();

                while (sw.Elapsed.TotalSeconds < timeout)
                {
                    int bytesAvailable = _transport.BytesToRead;
                    if (bytesAvailable > 0)
                    {
                        int readCount = _transport.Read(rxBuf, 0, Math.Min(bytesAvailable, rxBuf.Length));
                        for (int i = 0; i < readCount; i++)
                        {
                            var completedFrame = _parser.FeedByte(rxBuf[i]);
                            if (completedFrame != null)
                            {
                                OnRx?.Invoke(completedFrame);
                                if (completedFrame.Type == FrameType.Async)
                                {
                                    OnAsync?.Invoke(completedFrame);
                                    continue;
                                }

                                if (completedFrame.CanId == req.CanId && completedFrame.Seq == req.Seq)
                                {
                                    if (completedFrame.Type == FrameType.RespErr)
                                    {
                                        ErrorCode errCode = completedFrame.Payload.Length > 0
                                            ? (ErrorCode)completedFrame.Payload[0]
                                            : ErrorCode.ErrInternal;
                                        throw new TueUsbException(errCode);
                                    }
                                    return completedFrame;
                                }
                            }
                        }
                    }
                    else
                    {
                        Thread.Sleep(1);
                    }
                }

                throw new TimeoutException($"Transaction timed out after {timeout:F2}s for CanId 0x{req.CanId:X3}");
            }
        }

        private List<Frame> SimulateMockResponse(Frame req)
        {
            var res = new List<Frame>();
            uint id = req.CanId;
            byte seq = req.Seq;
            byte[] p = req.Payload;

            if (id == CanId.SysPing)
            {
                byte[] token = p.Length >= 4 ? p : new byte[4];
                res.Add(new Frame(FrameType.RespOk, id, token, seq));
            }
            else if (id == CanId.SysInfo)
            {
                byte[] payload = new byte[8];
                payload[0] = 1; payload[1] = 1; payload[2] = 1; payload[3] = 0;
                res.Add(new Frame(FrameType.RespOk, id, payload, seq));
            }
            else if (id == CanId.SysUidPart1)
            {
                byte[] payload = new byte[8];
                BinaryPrimitives.WriteUInt32LittleEndian(payload.AsSpan(0, 4), 0x11223344);
                BinaryPrimitives.WriteUInt32LittleEndian(payload.AsSpan(4, 4), 0x55667788);
                res.Add(new Frame(FrameType.RespOk, id, payload, seq));
            }
            else if (id == CanId.SysUidPart2)
            {
                byte[] payload = new byte[4];
                BinaryPrimitives.WriteUInt32LittleEndian(payload, 0x99AABBCC);
                res.Add(new Frame(FrameType.RespOk, id, payload, seq));
            }
            else if (id == CanId.SysUptime)
            {
                byte[] payload = new byte[4];
                BinaryPrimitives.WriteUInt32LittleEndian(payload, (uint)_mockUptime.ElapsedMilliseconds);
                res.Add(new Frame(FrameType.RespOk, id, payload, seq));
            }
            else if (id == CanId.SysLedCtrl)
            {
                byte led = p.Length > 0 ? p[0] : (byte)0;
                byte state = p.Length > 1 ? p[1] : (byte)0;
                if (state == 2) _mockLedState = (byte)(_mockLedState == 1 ? 0 : 1);
                else if (state <= 1) _mockLedState = state;
                res.Add(new Frame(FrameType.RespOk, id, new byte[] { led, _mockLedState }, seq));
            }
            else if (id == CanId.SysReset || id == CanId.SysEcho)
            {
                res.Add(new Frame(FrameType.RespOk, id, p, seq));
            }
            else if (id == CanId.HapticPing)
            {
                byte token = p.Length > 0 ? p[0] : (byte)0;
                byte[] payload = new byte[] { token, 1, 1, 0, 1, 0, 0, 0 };
                res.Add(new Frame(FrameType.RespOk, id, payload, seq));
            }
            else if (id == CanId.HapticDiag)
            {
                byte[] payload = new byte[] { 0x0B, 0, 0, 0, 0, 0, 0, 0 };
                res.Add(new Frame(FrameType.RespOk, id, payload, seq));
            }
            else if (id == CanId.StepGetInfo)
            {
                byte[] payload = new byte[8];
                BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(0, 2), 80);
                BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(2, 2), 400);
                BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(4, 2), 50);
                BinaryPrimitives.WriteUInt16LittleEndian(payload.AsSpan(6, 2), 4000);
                res.Add(new Frame(FrameType.RespOk, id, payload, seq));
            }
            else if (id == CanId.StepGetStatus)
            {
                res.Add(new Frame(FrameType.RespOk, id, new byte[] { 0, (byte)(_mockStepperEnabled ? 1 : 0), 0, 0 }, seq));
            }
            else if (id == CanId.StepEnable)
            {
                if (p.Length > 0 && p[0] != 0xFF) _mockStepperEnabled = (p[0] != 0);
                res.Add(new Frame(FrameType.RespOk, id, new byte[] { (byte)(_mockStepperEnabled ? 1 : 0) }, seq));
            }
            else if (id == CanId.StepGetPosXY || id == CanId.StepGetTgtXY)
            {
                byte[] payload = new byte[8];
                BinaryPrimitives.WriteInt32LittleEndian(payload.AsSpan(0, 4), _mockPosXUm);
                BinaryPrimitives.WriteInt32LittleEndian(payload.AsSpan(4, 4), _mockPosYUm);
                res.Add(new Frame(FrameType.RespOk, id, payload, seq));
            }
            else if (id == CanId.StepGetPosZ || id == CanId.StepGetTgtZ)
            {
                byte[] payload = new byte[4];
                BinaryPrimitives.WriteInt32LittleEndian(payload, _mockPosZUm);
                res.Add(new Frame(FrameType.RespOk, id, payload, seq));
            }
            else if (id == CanId.StepSetposXY || id == CanId.StepMoveXY)
            {
                if (p.Length >= 8)
                {
                    _mockPosXUm = BinaryPrimitives.ReadInt32LittleEndian(p.AsSpan(0, 4));
                    _mockPosYUm = BinaryPrimitives.ReadInt32LittleEndian(p.AsSpan(4, 4));
                }
                res.Add(new Frame(FrameType.RespOk, id, p, seq));
            }
            else if (id == CanId.StepSetposZ || id == CanId.StepMoveZ)
            {
                if (p.Length >= 4)
                {
                    _mockPosZUm = BinaryPrimitives.ReadInt32LittleEndian(p.AsSpan(0, 4));
                }
                res.Add(new Frame(FrameType.RespOk, id, p, seq));
            }
            else if (id == CanId.StepMoveCoords)
            {
                if (p.Length >= 8)
                {
                    _mockPosXUm = BinaryPrimitives.ReadInt32LittleEndian(p.AsSpan(0, 4));
                    _mockPosYUm = BinaryPrimitives.ReadInt32LittleEndian(p.AsSpan(4, 4));
                }
                res.Add(new Frame(FrameType.RespOk, id, new byte[] { 1, 0 }, seq));
            }
            else if (id == CanId.StepMoveExec)
            {
                if (p.Length >= 4)
                {
                    _mockPosZUm = BinaryPrimitives.ReadInt32LittleEndian(p.AsSpan(0, 4));
                }
                byte[] payload = new byte[8];
                BinaryPrimitives.WriteInt32LittleEndian(payload.AsSpan(0, 4), _mockPosXUm);
                BinaryPrimitives.WriteInt32LittleEndian(payload.AsSpan(4, 4), _mockPosYUm);
                res.Add(new Frame(FrameType.RespOk, id, payload, seq));
            }
            else if (id == CanId.StepJogXY || id == CanId.StepJogZ || id == CanId.StepStop || id == CanId.StepAbort)
            {
                res.Add(new Frame(FrameType.RespOk, id, new byte[] { 1 }, seq));
            }
            else if (id == CanId.ServoPing)
            {
                byte sid = p.Length > 0 ? p[0] : (byte)1;
                res.Add(new Frame(FrameType.RespOk, id, new byte[] { sid, 0 }, seq));
            }
            else if (id == CanId.ServoSetPos)
            {
                byte sid = p.Length > 0 ? p[0] : (byte)1;
                short pos = p.Length >= 4 ? BinaryPrimitives.ReadInt16LittleEndian(p.AsSpan(2, 2)) : (short)2048;
                _mockServoPos[sid] = pos;
                res.Add(new Frame(FrameType.RespOk, id, p, seq));
            }
            else if (id == CanId.ServoGetPos)
            {
                byte sid = p.Length > 0 ? p[0] : (byte)1;
                short pos = _mockServoPos.TryGetValue(sid, out var val) ? val : (short)2048;
                byte[] payload = new byte[3];
                payload[0] = sid;
                BinaryPrimitives.WriteInt16LittleEndian(payload.AsSpan(1, 2), pos);
                res.Add(new Frame(FrameType.RespOk, id, payload, seq));
            }
            else if (id == CanId.ServoTorque)
            {
                res.Add(new Frame(FrameType.RespOk, id, p, seq));
            }
            else if (id == CanId.ServoFbDynamics)
            {
                byte sid = p.Length > 0 ? p[0] : (byte)1;
                short pos = _mockServoPos.TryGetValue(sid, out var val) ? val : (short)2048;
                byte[] payload = new byte[8];
                payload[0] = sid;
                BinaryPrimitives.WriteInt16LittleEndian(payload.AsSpan(1, 2), pos);
                BinaryPrimitives.WriteInt16LittleEndian(payload.AsSpan(3, 2), 0);
                BinaryPrimitives.WriteInt16LittleEndian(payload.AsSpan(5, 2), 50);
                payload[7] = 0;
                res.Add(new Frame(FrameType.RespOk, id, payload, seq));
            }
            else if (id == CanId.ServoFbHealth)
            {
                byte sid = p.Length > 0 ? p[0] : (byte)1;
                byte[] payload = new byte[8];
                payload[0] = sid;
                payload[1] = 120; // 12.0V
                payload[2] = 35;  // 35C
                payload[3] = 0;   // Status OK
                BinaryPrimitives.WriteInt16LittleEndian(payload.AsSpan(4, 2), 150); // 150mA
                res.Add(new Frame(FrameType.RespOk, id, payload, seq));
            }
            else
            {
                res.Add(new Frame(FrameType.RespOk, id, p, seq));
            }

            return res;
        }

        // --- High Level API ---
        public uint Ping(uint token = 0)
        {
            var resp = Transact(new Frame(FrameType.Req, CanId.SysPing, Encoders.SysPing(token)));
            return Decoders.SysPing(resp.Payload);
        }

        public SysInfo GetInfo()
        {
            var resp = Transact(new Frame(FrameType.Req, CanId.SysInfo));
            return Decoders.SysInfo(resp.Payload);
        }

        public SysUid GetUid()
        {
            var resp1 = Transact(new Frame(FrameType.Req, CanId.SysUidPart1));
            var (uid0, uid1) = Decoders.SysUidPart1(resp1.Payload);
            var resp2 = Transact(new Frame(FrameType.Req, CanId.SysUidPart2));
            var uid2 = Decoders.SysUidPart2(resp2.Payload);
            return new SysUid(uid0, uid1, uid2);
        }

        public uint GetUptime()
        {
            var resp = Transact(new Frame(FrameType.Req, CanId.SysUptime));
            return Decoders.SysUptime(resp.Payload);
        }

        public StepperConfig StepGetConfig()
        {
            var resp = Transact(new Frame(FrameType.Req, CanId.StepGetInfo));
            return Decoders.StepGetInfo(resp.Payload);
        }

        public StepperStatus StepGetStatus()
        {
            var resp = Transact(new Frame(FrameType.Req, CanId.StepGetStatus));
            return Decoders.StepGetStatus(resp.Payload);
        }

        public bool StepEnable(bool enable)
        {
            var resp = Transact(new Frame(FrameType.Req, CanId.StepEnable, Encoders.StepEnable(enable)));
            return Decoders.StepEnable(resp.Payload);
        }

        public (double x, double y) StepGetPosXY()
        {
            var resp = Transact(new Frame(FrameType.Req, CanId.StepGetPosXY));
            return Decoders.StepPosXY(resp.Payload);
        }

        public double StepGetPosZ()
        {
            var resp = Transact(new Frame(FrameType.Req, CanId.StepGetPosZ));
            return Decoders.StepPosZ(resp.Payload);
        }

        public void StepJogXY(double vxMmS, double vyMmS, double accelMmS2 = 300.0)
        {
            Transact(new Frame(FrameType.Req, CanId.StepJogXY,
                Encoders.StepJogXY((short)vxMmS, (short)vyMmS, (ushort)accelMmS2)));
        }

        public void StepJogZ(double vzMmS, double accelMmS2 = 300.0)
        {
            Transact(new Frame(FrameType.Req, CanId.StepJogZ,
                Encoders.StepJogZ((short)vzMmS, (ushort)accelMmS2)));
        }

        public void StepStop()
        {
            Transact(new Frame(FrameType.Req, CanId.StepStop));
        }

        public void StepAbort()
        {
            Transact(new Frame(FrameType.Req, CanId.StepAbort));
        }

        public bool ServoPing(byte sid, double timeout = 0.5)
        {
            try
            {
                var resp = Transact(new Frame(FrameType.Req, CanId.ServoPing, Encoders.ServoPing(sid)), timeout);
                return resp.Type == FrameType.RespOk;
            }
            catch
            {
                return false;
            }
        }

        public void ServoSetPos(byte sid, int pos, int speed = 0, int accel = 0)
        {
            Transact(new Frame(FrameType.Req, CanId.ServoSetPos,
                Encoders.ServoSetPos(sid, (short)pos, (short)speed, (byte)accel)));
        }

        public void ServoTorqueEnable(byte sid, bool enable)
        {
            Transact(new Frame(FrameType.Req, CanId.ServoTorque,
                Encoders.ServoTorque(sid, enable ? ServoTorque.On : ServoTorque.Off)));
        }

        public ServoFeedback ServoGetFeedback(byte sid)
        {
            var respDyn = Transact(new Frame(FrameType.Req, CanId.ServoFbDynamics, Encoders.ServoGetFb(sid)));
            var dyn = Decoders.ServoFbDynamics(respDyn.Payload);

            var respHlth = Transact(new Frame(FrameType.Req, CanId.ServoFbHealth, Encoders.ServoGetFb(sid)));
            var hlth = Decoders.ServoFbHealth(respHlth.Payload);

            return new ServoFeedback(dyn, hlth);
        }

        public HapticPingInfo HapticPing(byte token = 0)
        {
            var resp = Transact(new Frame(FrameType.Req, CanId.HapticPing, Encoders.HapticPing(token)));
            return Decoders.HapticPing(resp.Payload);
        }

        public HapticUartDiag HapticUartDiag()
        {
            var resp = Transact(new Frame(FrameType.Req, CanId.HapticDiag));
            return Decoders.HapticUartDiag(resp.Payload);
        }

        public void Dispose()
        {
            Disconnect();
        }
    }
}
