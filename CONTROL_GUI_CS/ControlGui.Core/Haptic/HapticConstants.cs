using System;

namespace ControlGui.Core.Haptic
{
    public struct ValveLayoutItem
    {
        public string Name { get; }
        public string Label { get; }
        public double X { get; }
        public double Y { get; }

        public ValveLayoutItem(string name, string label, double x, double y)
        {
            Name = name;
            Label = label;
            X = x;
            Y = y;
        }
    }

    public static class HapticConstants
    {
        public const uint CanId = 0x00000300;
        public const byte FrameType = 0x01; // Req
        public const byte Dlc = 8;
        public const int FrameLen = 20;

        public const int BaudRate = 1000000;
        public const double RateHz = 100.0;
        public const double PeriodSec = 0.010;

        public const double ForceLsbN = 0.1;
        public const double MaxForceN = 25.5;

        public const int NTaxels = 52;

        // 3-zone spatial partition along finger, Tip -> Base
        public static readonly (int Start, int End) ZoneTip = (0, 16);
        public static readonly (int Start, int End) ZoneMid = (16, 36);
        public static readonly (int Start, int End) ZoneBase = (36, 52);

        // Status flags bitmask
        public const byte FlagLeftContact = 0x01;
        public const byte FlagRightContact = 0x02;
        public const byte FlagSlip = 0x04;
        public const byte FlagTare = 0x08;

        public static readonly ValveLayoutItem[] ValveLayout = new ValveLayoutItem[]
        {
            new("A", "Tip (+Y)", 0.0, 0.0),
            new("B", "Mid (Fz)", 0.0, 11.0),
            new("C", "Base (-Y)", 0.0, 22.0),
            new("D", "Left flank (+X)", -7.0, 11.0),
            new("E", "Right flank (-X)", 7.0, 11.0),
        };
    }
}
