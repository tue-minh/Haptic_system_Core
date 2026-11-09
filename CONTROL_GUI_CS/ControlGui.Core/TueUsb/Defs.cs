using System;

namespace ControlGui.Core.TueUsb
{
    public static class FramingConstants
    {
        public const byte SOF1 = 0xAA;
        public const byte SOF2 = 0x55;
        public const byte EOF1 = 0x55;
        public const byte EOF2 = 0xAA;

        public const int HeaderSize = 7;      // TYPE(1) + ID(4) + DLC(1) + SEQ(1)
        public const int CrcSize = 1;         // SAE J1850 CRC-8
        public const int FrameOverhead = 12;  // SOF(2) + HEADER(7) + CRC(1) + EOF(2)
        public const int MaxDlc = 8;
        public const int MinFrameLen = FrameOverhead;          // 12 bytes
        public const int MaxFrameLen = FrameOverhead + MaxDlc; // 20 bytes
    }

    public enum FrameType : byte
    {
        Req = 0x01,        // Host -> MCU Command / Request
        RespOk = 0x02,     // MCU -> Host Acknowledged Success (with optional data)
        RespErr = 0x03,    // MCU -> Host Execution Error (payload[0] = ErrorCode)
        Async = 0x04       // MCU -> Host Unsolicited Event / Telemetry
    }

    public enum ErrorCode : byte
    {
        Ok = 0x00,               // Success
        ErrCrc = 0x01,          // Frame CRC-8 check failed
        ErrBadFormat = 0x02,    // Truncated frame, framing mismatch, or DLC > 8
        ErrUnknownCanId = 0x03, // CAN ID is unmapped or unsupported
        ErrBadArg = 0x04,       // Payload parameter out of range or malformed
        ErrBusTimeout = 0x05,   // Downstream bus transaction timed out (e.g. STS3215 UART)
        ErrBusFault = 0x06,     // Hardware framing / DMA error
        ErrBusy = 0x07,         // Subsystem busy with prior transaction
        ErrServoFault = 0x08,   // Servo reported hardware alarm bitmask
        ErrQueueFull = 0x09,    // Stepper motion planner queue saturated (16 segments max)
        ErrTimeout = 0x0A,      // Stepper wait deadline expired
        ErrNotSupported = 0x0B, // Command or direction not implemented
        ErrInternal = 0x0C      // MCU driver internal fault
    }

    public static class ErrorCodeExtensions
    {
        public static string Describe(this ErrorCode code) => code switch
        {
            ErrorCode.Ok => "Operation successful",
            ErrorCode.ErrCrc => "Frame CRC-8 integrity verification failed",
            ErrorCode.ErrBadFormat => "Malformed frame envelope or invalid DLC",
            ErrorCode.ErrUnknownCanId => "Unknown or unmapped CAN ID",
            ErrorCode.ErrBadArg => "Payload argument value out of range",
            ErrorCode.ErrBusTimeout => "Downstream serial bus timeout",
            ErrorCode.ErrBusFault => "Hardware communication fault / framing error",
            ErrorCode.ErrBusy => "Subsystem is currently busy",
            ErrorCode.ErrServoFault => "Bus servo reported hardware alarm condition",
            ErrorCode.ErrQueueFull => "Motion planner queue full (16 segments max)",
            ErrorCode.ErrTimeout => "Motion execution wait timeout expired",
            ErrorCode.ErrNotSupported => "Command or feature not supported",
            ErrorCode.ErrInternal => "Internal firmware driver error",
            _ => $"Unknown error code (0x{(byte)code:X2})"
        };
    }

    public static class CanId
    {
        // System Management (0x100 - 0x107)
        public const uint SysPing = 0x100;
        public const uint SysInfo = 0x101;
        public const uint SysUidPart1 = 0x102;
        public const uint SysUidPart2 = 0x103;
        public const uint SysUptime = 0x104;
        public const uint SysLedCtrl = 0x105;
        public const uint SysReset = 0x106;
        public const uint SysEcho = 0x107;

        // Stepper Motor Kinematics (CoreXY + Z) (0x200 - 0x21A)
        public const uint StepGetInfo = 0x200;
        public const uint StepGetStatus = 0x201;
        public const uint StepEnable = 0x202;
        public const uint StepGetPosXY = 0x203;
        public const uint StepGetPosZ = 0x204;
        public const uint StepGetTgtXY = 0x205;
        public const uint StepGetTgtZ = 0x206;
        public const uint StepSetposXY = 0x207;
        public const uint StepSetposZ = 0x208;
        public const uint StepMoveCoords = 0x210;
        public const uint StepMoveExec = 0x211;
        public const uint StepMoveXY = 0x212;
        public const uint StepMoveZ = 0x213;
        public const uint StepMoveRelXY = 0x214;
        public const uint StepSetSpeed = 0x215;
        public const uint StepJogXY = 0x216;
        public const uint StepJogZ = 0x217;
        public const uint StepStop = 0x218;
        public const uint StepAbort = 0x219;
        public const uint StepWaitIdle = 0x21A;

        // Raw Stepper Motor Channels (0x280 - 0x281)
        public const uint StepRawGetSteps = 0x280;
        public const uint StepRawMove = 0x281;

        // STS3215 Serial Bus Servos (0x400 - 0x415)
        public const uint ServoPing = 0x400;
        public const uint ServoSetPos = 0x401;
        public const uint ServoGetPos = 0x402;
        public const uint ServoSetSpeed = 0x403;
        public const uint ServoGetSpeed = 0x404;
        public const uint ServoTorque = 0x405;
        public const uint ServoTlim = 0x406;
        public const uint ServoMode = 0x407;
        public const uint ServoPwm = 0x408;
        public const uint ServoGetFb = 0x410;
        public const uint ServoFbDynamics = 0x410;
        public const uint ServoFbHealth = 0x411;
        public const uint ServoMemRead = 0x412;
        public const uint ServoMemWrite = 0x413;
        public const uint ServoSetId = 0x414;
        public const uint ServoReset = 0x415;

        // User Custom Range (STM32 <-> UART slave bridge)
        public const uint HapticTelemetry = 0x300;
        public const uint HapticPing = 0x301;
        public const uint HapticDiag = 0x302;

        public static bool IsReserved(uint canId)
        {
            return (canId >= 0x300 && canId <= 0x3FF) ||
                   (canId >= 0x3000 && canId <= 0x3FFF) ||
                   (canId >= 0x30000000 && canId <= 0x3FFFFFFF);
        }
    }

    public enum LedState : byte
    {
        Off = 0,
        On = 1,
        Toggle = 2,
        Query = 0xFF
    }

    public enum ServoMode : byte
    {
        Position = 0,
        Speed = 1,
        Pwm = 2,
        Step = 3,
        Query = 0xFF
    }

    public enum ServoTorque : byte
    {
        Off = 0,
        On = 1,
        Calibrate = 128,
        Query = 0xFF
    }

    [Flags]
    public enum ServoAlarmBit : byte
    {
        None = 0x00,
        Voltage = 0x01,     // Input voltage out of safe bounds
        Sensor = 0x02,      // Magnetic encoder error
        Temperature = 0x04, // Overheating alarm
        Current = 0x08,     // Overcurrent alarm
        Angle = 0x10,       // Target position beyond software limits
        Overload = 0x20     // Mechanical overload stall
    }
}
