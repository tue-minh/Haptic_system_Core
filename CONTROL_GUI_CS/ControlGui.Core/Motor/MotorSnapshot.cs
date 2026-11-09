using System;

namespace ControlGui.Core.Motor
{
    public record MotorSnapshot
    {
        public bool Connected { get; init; } = false;
        public bool Connecting { get; init; } = false;
        public bool Streaming { get; init; } = false;
        public string Error { get; init; } = "";
        public string Board { get; init; } = "";
        public string Firmware { get; init; } = "";

        public bool Enabled { get; init; } = false;
        public bool Busy { get; init; } = false;
        public double X { get; init; } = 0.0;
        public double Y { get; init; } = 0.0;
        public double Z { get; init; } = 0.0;
        public ushort StepsPerMmXy { get; init; } = 0;
        public ushort StepsPerMmZ { get; init; } = 0;

        public int ServoPos { get; init; } = 0;
        public double ServoVoltV { get; init; } = 0.0;
        public double ServoLoadPct { get; init; } = 0.0;
        public int ServoTempC { get; init; } = 0;
        public bool ServoMoving { get; init; } = false;
        public byte ServoId { get; init; } = MotorConstants.DefaultServoId;
        public bool ServoOnline { get; init; } = false;
        public bool ServoScanning { get; init; } = false;
        public string ServoStatusMsg { get; init; } = "Offline (not synced)";

        public string? Jogging { get; init; } = null;
        public string LastWarning { get; init; } = "";
        public double PollHz { get; init; } = 0.0;
        public long Errors { get; init; } = 0;

        public string HapticPing { get; init; } = "";
        public bool HapticPingOk { get; init; } = false;
        public bool HapticPingPending { get; init; } = false;
    }
}
