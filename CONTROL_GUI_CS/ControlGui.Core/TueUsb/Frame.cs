using System;
using System.Buffers.Binary;

namespace ControlGui.Core.TueUsb
{
    public class Frame
    {
        public FrameType Type { get; set; }
        public uint CanId { get; set; }
        public byte[] Payload { get; set; }
        public byte Seq { get; set; }
        public byte? Crc { get; set; }

        public int Dlc => Payload?.Length ?? 0;

        public Frame(FrameType type, uint canId, byte[]? payload = null, byte seq = 0, byte? crc = null)
        {
            Type = type;
            CanId = canId;
            Payload = payload ?? Array.Empty<byte>();
            if (Payload.Length > FramingConstants.MaxDlc)
            {
                throw new ArgumentException($"Payload length {Payload.Length} exceeds maximum DLC {FramingConstants.MaxDlc}");
            }
            Seq = seq;
            Crc = crc ?? ComputeCrc();
        }

        public byte ComputeCrc()
        {
            // CRC computed over Header (7B) + Payload (DLC)
            byte[] buf = new byte[FramingConstants.HeaderSize + Dlc];
            buf[0] = (byte)Type;
            BinaryPrimitives.WriteUInt32LittleEndian(buf.AsSpan(1, 4), CanId);
            buf[5] = (byte)Dlc;
            buf[6] = Seq;
            if (Dlc > 0 && Payload != null)
            {
                Array.Copy(Payload, 0, buf, FramingConstants.HeaderSize, Dlc);
            }
            return Crc8.Compute(buf);
        }

        public bool IsCrcValid()
        {
            return Crc.HasValue && Crc.Value == ComputeCrc();
        }

        public byte[] ToBytes()
        {
            byte crcVal = Crc ?? ComputeCrc();
            int totalLen = FramingConstants.FrameOverhead + Dlc;
            byte[] result = new byte[totalLen];

            result[0] = FramingConstants.SOF1;
            result[1] = FramingConstants.SOF2;

            result[2] = (byte)Type;
            BinaryPrimitives.WriteUInt32LittleEndian(result.AsSpan(3, 4), CanId);
            result[7] = (byte)Dlc;
            result[8] = Seq;

            if (Dlc > 0 && Payload != null)
            {
                Array.Copy(Payload, 0, result, 9, Dlc);
            }

            int crcIndex = 9 + Dlc;
            result[crcIndex] = crcVal;
            result[crcIndex + 1] = FramingConstants.EOF1;
            result[crcIndex + 2] = FramingConstants.EOF2;

            return result;
        }

        public static Frame FromBytes(ReadOnlySpan<byte> raw)
        {
            if (raw.Length < FramingConstants.MinFrameLen)
            {
                throw new ArgumentException($"Frame length {raw.Length} is shorter than minimum {FramingConstants.MinFrameLen}");
            }

            if (raw[0] != FramingConstants.SOF1 || raw[1] != FramingConstants.SOF2)
            {
                throw new FormatException($"Invalid SOF markers: 0x{raw[0]:X2} 0x{raw[1]:X2}");
            }

            FrameType type = (FrameType)raw[2];
            uint canId = BinaryPrimitives.ReadUInt32LittleEndian(raw.Slice(3, 4));
            byte dlc = raw[7];
            byte seq = raw[8];

            if (dlc > FramingConstants.MaxDlc)
            {
                throw new FormatException($"DLC {dlc} exceeds maximum {FramingConstants.MaxDlc}");
            }

            int expectedLen = FramingConstants.FrameOverhead + dlc;
            if (raw.Length != expectedLen)
            {
                throw new FormatException($"Byte length {raw.Length} does not match expected {expectedLen} for DLC={dlc}");
            }

            byte[] payload = dlc > 0 ? raw.Slice(9, dlc).ToArray() : Array.Empty<byte>();
            int crcPos = 9 + dlc;
            byte crc = raw[crcPos];
            byte eof1 = raw[crcPos + 1];
            byte eof2 = raw[crcPos + 2];

            if (eof1 != FramingConstants.EOF1 || eof2 != FramingConstants.EOF2)
            {
                throw new FormatException($"Invalid EOF markers: 0x{eof1:X2} 0x{eof2:X2}");
            }

            // Verify CRC
            byte calculatedCrc = Crc8.Compute(raw.Slice(2, FramingConstants.HeaderSize + dlc));
            if (crc != calculatedCrc)
            {
                throw new FormatException($"CRC verification failed (computed: 0x{calculatedCrc:X2}, received: 0x{crc:X2})");
            }

            return new Frame(type, canId, payload, seq, crc);
        }

        public override string ToString()
        {
            return $"Frame(Type={Type}, CanId=0x{CanId:X3}, DLC={Dlc}, Seq={Seq}, Payload=[{BitConverter.ToString(Payload)}])";
        }
    }
}
