using System;

namespace ControlGui.Core.Motor
{
    public static class MotorConstants
    {
        public const double VelMin = 1.0;
        public const double VelMax = 100.0;

        public const double AccelMin = 50.0;
        public const double AccelMax = 1000.0;

        public const int GripperOpen = 55;
        public const int GripperClose = 420;

        public const int GripperPosMin = 55;
        public const int GripperPosMax = 420;

        public const int GripperSpeedMin = 100;
        public const int GripperSpeedMax = 3000;

        public const int GripperAccelMin = 10;
        public const int GripperAccelMax = 200;

        public const byte DefaultServoId = 1;

        public static double Clamp(double val, double min, double max)
        {
            return val < min ? min : (val > max ? max : val);
        }
    }
}
