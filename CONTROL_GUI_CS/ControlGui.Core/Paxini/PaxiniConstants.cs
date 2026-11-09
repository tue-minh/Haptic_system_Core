using System;

namespace ControlGui.Core.Paxini
{
    public static class PaxiniConstants
    {
        public const int DefaultBaudRate = 921600;
        public static readonly byte[] ReqHead = new byte[] { 0x55, 0xAA };
        public static readonly byte[] RespHead = new byte[] { 0xAA, 0x55 };
        public static readonly byte[] PushHead = new byte[] { 0xAA, 0x56 };

        public const byte FuncRead = 0x03;
        public const byte FuncWrite = 0x10;

        public const ushort RegVersion = 0x0000;
        public const ushort RegCombination = 0x0010;
        public const ushort RegDataType = 0x0016;
        public const ushort RegAutoPush = 0x0017;
        public const ushort RegPoints = 0x0030;

        public const double ForceLsbN = 0.1; // 1 LSB = 0.1 N
        public const int TaxelCount = 52;

        public static readonly byte[] ProbeAddresses = new byte[] { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };

        public static double TaxelThresholdN { get; set; } = 0.1;
        public static double ForceThresholdN { get; set; } = 0.2;
    }
}
