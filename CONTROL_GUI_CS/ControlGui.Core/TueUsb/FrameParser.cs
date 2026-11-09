using System;
using System.Buffers.Binary;
using System.IO;

namespace ControlGui.Core.TueUsb
{
    public enum ParserState
    {
        WaitSof1 = 0,
        WaitSof2 = 1,
        Header = 2,
        Payload = 3,
        Crc = 4,
        Eof1 = 5,
        Eof2 = 6
    }

    /// <summary>
    /// Streaming byte-by-byte frame parser for TUE USB Binary Protocol.
    /// Resilient to arbitrary fragmentation, stream noise, and concatenated frames.
    /// </summary>
    public class FrameParser
    {
        private ParserState _state = ParserState.WaitSof1;
        private readonly byte[] _headerBuf = new byte[FramingConstants.HeaderSize];
        private int _headerBytesRead;

        private readonly byte[] _payloadBuf = new byte[FramingConstants.MaxDlc];
        private int _payloadBytesRead;

        private FrameType _frameType;
        private uint _canId;
        private byte _dlc;
        private byte _seq;
        private byte _crc8;

        public void Reset()
        {
            _state = ParserState.WaitSof1;
            _headerBytesRead = 0;
            _payloadBytesRead = 0;
            _canId = 0;
            _dlc = 0;
            _seq = 0;
            _crc8 = 0;
        }

        public Frame? FeedByte(byte b)
        {
            switch (_state)
            {
                case ParserState.WaitSof1:
                    if (b == FramingConstants.SOF1)
                    {
                        _state = ParserState.WaitSof2;
                    }
                    return null;

                case ParserState.WaitSof2:
                    if (b == FramingConstants.SOF2)
                    {
                        _headerBytesRead = 0;
                        _payloadBytesRead = 0;
                        _state = ParserState.Header;
                    }
                    else if (b != FramingConstants.SOF1)
                    {
                        _state = ParserState.WaitSof1;
                    }
                    // If b == SOF1, remain in WaitSof2 looking for SOF2
                    return null;

                case ParserState.Header:
                    _headerBuf[_headerBytesRead++] = b;
                    if (_headerBytesRead >= FramingConstants.HeaderSize)
                    {
                        _frameType = (FrameType)_headerBuf[0];
                        _canId = BinaryPrimitives.ReadUInt32LittleEndian(_headerBuf.AsSpan(1, 4));
                        _dlc = _headerBuf[5];
                        _seq = _headerBuf[6];

                        if (_dlc > FramingConstants.MaxDlc)
                        {
                            Reset();
                            return null;
                        }

                        if (_dlc == 0)
                        {
                            _state = ParserState.Crc;
                        }
                        else
                        {
                            _payloadBytesRead = 0;
                            _state = ParserState.Payload;
                        }
                    }
                    return null;

                case ParserState.Payload:
                    _payloadBuf[_payloadBytesRead++] = b;
                    if (_payloadBytesRead >= _dlc)
                    {
                        _state = ParserState.Crc;
                    }
                    return null;

                case ParserState.Crc:
                    // Compute expected CRC over Header + Payload
                    byte[] crcBuffer = new byte[FramingConstants.HeaderSize + _dlc];
                    Array.Copy(_headerBuf, 0, crcBuffer, 0, FramingConstants.HeaderSize);
                    if (_dlc > 0)
                    {
                        Array.Copy(_payloadBuf, 0, crcBuffer, FramingConstants.HeaderSize, _dlc);
                    }

                    byte expectedCrc = Crc8.Compute(crcBuffer);
                    if (b != expectedCrc)
                    {
                        // CRC mismatch, resynchronize
                        Reset();
                        return null;
                    }

                    _crc8 = b;
                    _state = ParserState.Eof1;
                    return null;

                case ParserState.Eof1:
                    if (b == FramingConstants.EOF1)
                    {
                        _state = ParserState.Eof2;
                    }
                    else if (b == FramingConstants.SOF1)
                    {
                        _state = ParserState.WaitSof2;
                    }
                    else
                    {
                        Reset();
                    }
                    return null;

                case ParserState.Eof2:
                    if (b == FramingConstants.EOF2)
                    {
                        byte[] payloadCopy = new byte[_dlc];
                        if (_dlc > 0)
                        {
                            Array.Copy(_payloadBuf, 0, payloadCopy, 0, _dlc);
                        }

                        var completedFrame = new Frame(_frameType, _canId, payloadCopy, _seq, _crc8);
                        Reset();
                        return completedFrame;
                    }
                    else
                    {
                        Reset();
                    }
                    return null;

                default:
                    Reset();
                    return null;
            }
        }
    }
}
