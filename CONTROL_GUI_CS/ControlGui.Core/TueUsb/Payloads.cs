using System;
using System.Buffers.Binary;
using System.Collections.Generic;

namespace ControlGui.Core.TueUsb
{
    public static class Units
    {
        public static int MmToUm(double mm) => (int)Math.Round(mm * 1000.0);
        public static double UmToMm(int um) => um / 1000.0;
    }

    public record SysInfo(byte HwId, byte HwRev, byte FwMajor, byte FwMinor, uint BuildFlags)
    {
        public string VersionString => $"v{FwMajor}.{FwMinor}";
        public override string ToString() => $"SysInfo(HW ID={HwId}, Rev={HwRev}, Firmware={VersionString}, Flags=0x{BuildFlags:X8})";
    }

    public record SysUid(uint Uid0, uint Uid1, uint Uid2)
    {
        public string HexString => $"{Uid0:X8}-{Uid1:X8}-{Uid2:X8}";
        public override string ToString() => $"UID: {HexString}";
    }

    public record StepperConfig(ushort XyStepsPerMm, ushort ZStepsPerMm, ushort MinRateStepsS, ushort MaxRateStepsS);

    public record StepperStatus(bool IsBusy, bool IsEnabled, byte QueueDepth);

    public record PositionXyz(double XMm, double YMm, double ZMm)
    {
        public int XUm => Units.MmToUm(XMm);
        public int YUm => Units.MmToUm(YMm);
        public int ZUm => Units.MmToUm(ZMm);
    }

    public record ServoFeedbackDynamics(byte ServoId, short Position, short Speed, short LoadRaw, bool IsMoving)
    {
        public double AngleDeg => (Position * 360.0) / 4096.0;
        public double LoadPercent => LoadRaw / 10.0;
    }

    public record ServoFeedbackHealth(byte ServoId, double VoltageV, sbyte TemperatureC, byte StatusMask, short CurrentMa)
    {
        public List<string> Alarms
        {
            get
            {
                var list = new List<string>();
                foreach (ServoAlarmBit bit in Enum.GetValues(typeof(ServoAlarmBit)))
                {
                    if (bit != ServoAlarmBit.None && ((ServoAlarmBit)StatusMask).HasFlag(bit))
                    {
                        list.Add(bit.ToString());
                    }
                }
                return list;
            }
        }
    }

    public record ServoFeedback(ServoFeedbackDynamics Dynamics, ServoFeedbackHealth Health);

    public record HapticPingInfo(byte Token, byte Protocol, byte FwMajor, byte FwMinor, byte State, byte FrameCounter, byte Errors, byte UptimeS)
    {
        public string VersionString => $"v{FwMajor}.{FwMinor}";
        public string StateName => State switch
        {
            0 => "IDLE",
            1 => "RUNNING",
            2 => "ERROR",
            _ => $"UNKNOWN({State})"
        };

        public override string ToString() =>
            $"ESP32 {VersionString} {StateName}, uptime {UptimeS}s, frame {FrameCounter}, errors {Errors}";
    }

    public record HapticUartDiag(byte Flags, byte RxErrors, ushort RxBytes, byte LastPingRxBytes, byte TxFail, byte Timeouts, byte Replies)
    {
        public bool Initialized => (Flags & 0x01) != 0;
        public bool RxArmed => (Flags & 0x02) != 0;
        public bool TxBusy => (Flags & 0x04) != 0;
        public bool LastTxOk => (Flags & 0x08) != 0;
        public bool LastPingSawBytes => (Flags & 0x10) != 0;

        public override string ToString() =>
            $"STM32 UART init={(Initialized ? 1 : 0)} rx_armed={(RxArmed ? 1 : 0)} tx_ok={(LastTxOk ? 1 : 0)} " +
            $"rx={RxBytes}B err={RxErrors} last_ping_rx={LastPingRxBytes}B tx_fail={TxFail} timeouts={Timeouts} replies={Replies}";
    }

    public record HapticMappingConfig(
        double VMinV,
        double VMaxV,
        double FThreshN,
        double FRatedN,
        bool SavedInEeprom)
    {
        public static HapticMappingConfig Default => new(
            Haptic.HapticConstants.VMin,
            Haptic.HapticConstants.VMax,
            Haptic.HapticConstants.ForceThresh,
            Haptic.HapticConstants.ForceRated,
            false);

        public double SlopeVPerN
        {
            get
            {
                double denom = FRatedN - FThreshN;
                return denom > 0.01 ? (VMaxV - VMinV) / denom : 0.0;
            }
        }

        public double ComputeVoltage(double forceN) =>
            Haptic.HapticConstants.ComputeValveVoltage(forceN, VMinV, VMaxV, FThreshN, FRatedN);

        public string FormatEquationSummary()
        {
            double k = SlopeVPerN;
            double b = VMinV - k * FThreshN;
            string sign = b >= 0 ? "+" : "-";
            return
                $"k = ({VMaxV:F2} - {VMinV:F2}) / ({FRatedN:F2} - {FThreshN:F2}) = {k:F5} V/N\n" +
                $"• F <= {FThreshN:F2} N           =>  V(F) = 0.000 V   (DAC = 0)\n" +
                $"• {FThreshN:F2} < F < {FRatedN:F2} N  =>  V(F) = {VMinV:F2} + {k:F5}*(F - {FThreshN:F2}) = {k:F5}*F {sign} {Math.Abs(b):F4} V\n" +
                $"• F >= {FRatedN:F2} N          =>  V(F) = {VMaxV:F3} V   (DAC = {Haptic.HapticConstants.VoltageToDac(VMaxV)})\n" +
                $"• DAC_12bit(F) = floor(V(F) / 3.30 * 4095)   [min open DAC = {Haptic.HapticConstants.VoltageToDac(VMinV)}]";
        }

        public override string ToString() =>
            $"Vmin={VMinV:F2}V, Vmax={VMaxV:F2}V, Fth={FThreshN:F2}N, Frated={FRatedN:F2}N" +
            (SavedInEeprom ? " [EEPROM saved]" : " [RAM only]");
    }

    public record SpiAdcFeedback(ushort[] Channels, byte Seq, byte Status, ushort RxOkCnt, ushort CrcErrCnt)
    {
        public bool AdcValid => (Status & 0x01) != 0;
        public bool AdcError => (Status & 0x02) != 0;

        public double[] Millivolts(int vrefMv = 3300)
        {
            var mv = new double[Channels.Length];
            for (int i = 0; i < Channels.Length; i++)
            {
                mv[i] = Channels[i] * (double)vrefMv / 4095.0;
            }
            return mv;
        }

        public override string ToString()
        {
            string ch = string.Join(", ", Channels);
            return $"SpiAdc[{ch}] seq={Seq} status=0x{Status:X2} ok={RxOkCnt} crc_err={CrcErrCnt}";
        }
    }

    public record SpiAdcCtrlStatus(bool Enabled, byte BusId, byte CsIdx, byte PeriodMs, uint TotalFrames)
    {
        public override string ToString() =>
            $"SpiAdcCtrl(enabled={(Enabled ? 1 : 0)}, bus=SPI{BusId}, cs={CsIdx}, period={PeriodMs}ms, frames={TotalFrames})";
    }

    public static class Encoders
    {
        public static byte[] SysPing(uint token = 0)
        {
            byte[] p = new byte[4];
            BinaryPrimitives.WriteUInt32LittleEndian(p, token);
            return p;
        }

        public static byte[] SysLed(byte ledIdx = 0, LedState state = LedState.Query)
        {
            return new byte[] { ledIdx, (byte)state };
        }

        public static byte[] SysReset(uint magic = 0xDEADBEEF)
        {
            byte[] p = new byte[4];
            BinaryPrimitives.WriteUInt32LittleEndian(p, magic);
            return p;
        }

        public static byte[] SysEcho(byte[] data)
        {
            if (data.Length > FramingConstants.MaxDlc)
                throw new ArgumentException($"SYS_ECHO payload cannot exceed {FramingConstants.MaxDlc} bytes");
            return (byte[])data.Clone();
        }

        public static byte[] StepEnable(bool enable)
        {
            return new byte[] { (byte)(enable ? 1 : 0) };
        }

        public static byte[] StepSetposXY(double xMm, double yMm)
        {
            byte[] p = new byte[8];
            BinaryPrimitives.WriteInt32LittleEndian(p.AsSpan(0, 4), Units.MmToUm(xMm));
            BinaryPrimitives.WriteInt32LittleEndian(p.AsSpan(4, 4), Units.MmToUm(yMm));
            return p;
        }

        public static byte[] StepSetposZ(double zMm)
        {
            byte[] p = new byte[4];
            BinaryPrimitives.WriteInt32LittleEndian(p, Units.MmToUm(zMm));
            return p;
        }

        public static byte[] StepMoveCoords(double xMm, double yMm)
        {
            byte[] p = new byte[8];
            BinaryPrimitives.WriteInt32LittleEndian(p.AsSpan(0, 4), Units.MmToUm(xMm));
            BinaryPrimitives.WriteInt32LittleEndian(p.AsSpan(4, 4), Units.MmToUm(yMm));
            return p;
        }

        public static byte[] StepMoveExec(double zMm, ushort feedMmS, ushort accelMmS2)
        {
            byte[] p = new byte[8];
            BinaryPrimitives.WriteInt32LittleEndian(p.AsSpan(0, 4), Units.MmToUm(zMm));
            BinaryPrimitives.WriteUInt16LittleEndian(p.AsSpan(4, 2), feedMmS);
            BinaryPrimitives.WriteUInt16LittleEndian(p.AsSpan(6, 2), accelMmS2);
            return p;
        }

        public static byte[] StepMoveXY(double xMm, double yMm)
        {
            byte[] p = new byte[8];
            BinaryPrimitives.WriteInt32LittleEndian(p.AsSpan(0, 4), Units.MmToUm(xMm));
            BinaryPrimitives.WriteInt32LittleEndian(p.AsSpan(4, 4), Units.MmToUm(yMm));
            return p;
        }

        public static byte[] StepMoveZ(double zMm, ushort feedMmS, ushort accelMmS2)
        {
            byte[] p = new byte[8];
            BinaryPrimitives.WriteInt32LittleEndian(p.AsSpan(0, 4), Units.MmToUm(zMm));
            BinaryPrimitives.WriteUInt16LittleEndian(p.AsSpan(4, 2), feedMmS);
            BinaryPrimitives.WriteUInt16LittleEndian(p.AsSpan(6, 2), accelMmS2);
            return p;
        }

        public static byte[] StepMoveRelXY(double dxMm, double dyMm)
        {
            byte[] p = new byte[8];
            BinaryPrimitives.WriteInt32LittleEndian(p.AsSpan(0, 4), Units.MmToUm(dxMm));
            BinaryPrimitives.WriteInt32LittleEndian(p.AsSpan(4, 4), Units.MmToUm(dyMm));
            return p;
        }

        public static byte[] StepSetSpeed(ushort defaultFeedMmS, ushort defaultAccelMmS2)
        {
            byte[] p = new byte[4];
            BinaryPrimitives.WriteUInt16LittleEndian(p.AsSpan(0, 2), defaultFeedMmS);
            BinaryPrimitives.WriteUInt16LittleEndian(p.AsSpan(2, 2), defaultAccelMmS2);
            return p;
        }

        public static byte[] StepJogXY(short vxMmS, short vyMmS, ushort accelMmS2)
        {
            byte[] p = new byte[6];
            BinaryPrimitives.WriteInt16LittleEndian(p.AsSpan(0, 2), vxMmS);
            BinaryPrimitives.WriteInt16LittleEndian(p.AsSpan(2, 2), vyMmS);
            BinaryPrimitives.WriteUInt16LittleEndian(p.AsSpan(4, 2), accelMmS2);
            return p;
        }

        public static byte[] StepJogZ(short vzMmS, ushort accelMmS2)
        {
            byte[] p = new byte[4];
            BinaryPrimitives.WriteInt16LittleEndian(p.AsSpan(0, 2), vzMmS);
            BinaryPrimitives.WriteUInt16LittleEndian(p.AsSpan(2, 2), accelMmS2);
            return p;
        }

        public static byte[] StepWaitIdle(uint timeoutMs = 0)
        {
            byte[] p = new byte[4];
            BinaryPrimitives.WriteUInt32LittleEndian(p, timeoutMs);
            return p;
        }

        public static byte[] StepRawGetSteps(byte motorId)
        {
            return new byte[] { motorId };
        }

        public static byte[] StepRawMove(byte motorId, int steps, ushort feedMmS = 0)
        {
            byte[] p = new byte[8];
            p[0] = motorId;
            p[1] = 0;
            BinaryPrimitives.WriteInt32LittleEndian(p.AsSpan(2, 4), steps);
            BinaryPrimitives.WriteUInt16LittleEndian(p.AsSpan(6, 2), feedMmS);
            return p;
        }

        public static byte[] ServoPing(byte servoId)
        {
            return new byte[] { servoId };
        }

        public static byte[] ServoSetPos(byte servoId, short pos, short speed = 0, byte accel = 0)
        {
            byte[] p = new byte[6];
            p[0] = servoId;
            p[1] = accel;
            BinaryPrimitives.WriteInt16LittleEndian(p.AsSpan(2, 2), pos);
            BinaryPrimitives.WriteInt16LittleEndian(p.AsSpan(4, 2), speed);
            return p;
        }

        public static byte[] ServoGetPos(byte servoId)
        {
            return new byte[] { servoId };
        }

        public static byte[] ServoSetSpeed(byte servoId, short speed, byte accel = 0)
        {
            byte[] p = new byte[4];
            p[0] = servoId;
            p[1] = accel;
            BinaryPrimitives.WriteInt16LittleEndian(p.AsSpan(2, 2), speed);
            return p;
        }

        public static byte[] ServoGetSpeed(byte servoId)
        {
            return new byte[] { servoId };
        }

        public static byte[] ServoTorque(byte servoId, ServoTorque state)
        {
            return new byte[] { servoId, (byte)state };
        }

        public static byte[] ServoTlim(byte servoId, ushort limit0p1Pct)
        {
            byte[] p = new byte[3];
            p[0] = servoId;
            BinaryPrimitives.WriteUInt16LittleEndian(p.AsSpan(1, 2), limit0p1Pct);
            return p;
        }

        public static byte[] ServoMode(byte servoId, ServoMode mode)
        {
            return new byte[] { servoId, (byte)mode };
        }

        public static byte[] ServoPwm(byte servoId, short pwm)
        {
            byte[] p = new byte[3];
            p[0] = servoId;
            BinaryPrimitives.WriteInt16LittleEndian(p.AsSpan(1, 2), pwm);
            return p;
        }

        public static byte[] ServoGetFb(byte servoId)
        {
            return new byte[] { servoId };
        }

        public static byte[] ServoMemRead(byte servoId, byte regAddr, byte length)
        {
            return new byte[] { servoId, regAddr, length };
        }

        public static byte[] ServoMemWrite(byte servoId, byte regAddr, byte[] data)
        {
            if (data.Length > 6)
                throw new ArgumentException("SERVO_MEM_WRITE max write length is 6 bytes");
            byte[] p = new byte[2 + data.Length];
            p[0] = servoId;
            p[1] = regAddr;
            Array.Copy(data, 0, p, 2, data.Length);
            return p;
        }

        public static byte[] ServoSetId(byte currentId, byte newId)
        {
            return new byte[] { currentId, newId };
        }

        public static byte[] ServoReset(byte servoId)
        {
            return new byte[] { servoId };
        }

        public static byte[] HapticPing(byte token = 0)
        {
            byte[] p = new byte[8];
            p[0] = token;
            return p;
        }

        public static byte[] HapticSetCfg(double vMinV, double vMaxV, double fThreshN, double fRatedN, bool saveEeprom)
        {
            ushort vminMv = (ushort)Math.Clamp((int)Math.Round(vMinV * 1000.0), 0, 3300);
            ushort vmaxMv = (ushort)Math.Clamp((int)Math.Round(vMaxV * 1000.0), vminMv, 3300);
            ushort fthCn  = (ushort)Math.Clamp((int)Math.Round(fThreshN * 100.0), 0, 2500);
            ushort fratCn = (ushort)Math.Clamp((int)Math.Round(fRatedN * 100.0), fthCn + 10, 2550);
            ushort packed0 = (ushort)((vminMv & 0x7FFF) | (saveEeprom ? 0x8000 : 0));

            byte[] p = new byte[8];
            BinaryPrimitives.WriteUInt16LittleEndian(p.AsSpan(0, 2), packed0);
            BinaryPrimitives.WriteUInt16LittleEndian(p.AsSpan(2, 2), vmaxMv);
            BinaryPrimitives.WriteUInt16LittleEndian(p.AsSpan(4, 2), fthCn);
            BinaryPrimitives.WriteUInt16LittleEndian(p.AsSpan(6, 2), fratCn);
            return p;
        }

        public static byte[] HapticGetCfg() => new byte[8];

        public static byte[] SensorAdcReq(int? csIdx = null)
        {
            return csIdx.HasValue ? new byte[] { (byte)(csIdx.Value & 0xFF) } : Array.Empty<byte>();
        }

        public static byte[] SensorSpiCtrl(bool enable, byte busId = 1, byte csIdx = 0, byte periodMs = 10)
        {
            return new byte[] { (byte)(enable ? 1 : 0), busId, csIdx, periodMs };
        }
    }

    public static class Decoders
    {
        public static uint SysPing(byte[] payload)
        {
            return payload.Length >= 4 ? BinaryPrimitives.ReadUInt32LittleEndian(payload) : 0;
        }

        public static SysInfo SysInfo(byte[] payload)
        {
            if (payload.Length < 8) throw new ArgumentException("SYS_INFO payload must be 8 bytes");
            byte hwId = payload[0];
            byte hwRev = payload[1];
            byte maj = payload[2];
            byte min = payload[3];
            uint flags = BinaryPrimitives.ReadUInt32LittleEndian(payload.AsSpan(4, 4));
            return new SysInfo(hwId, hwRev, maj, min, flags);
        }

        public static (uint part1, uint part2) SysUidPart1(byte[] payload)
        {
            if (payload.Length < 8) throw new ArgumentException("SYS_UID_PART1 payload must be 8 bytes");
            uint p1 = BinaryPrimitives.ReadUInt32LittleEndian(payload.AsSpan(0, 4));
            uint p2 = BinaryPrimitives.ReadUInt32LittleEndian(payload.AsSpan(4, 4));
            return (p1, p2);
        }

        public static uint SysUidPart2(byte[] payload)
        {
            if (payload.Length < 4) throw new ArgumentException("SYS_UID_PART2 payload must be 4 bytes");
            return BinaryPrimitives.ReadUInt32LittleEndian(payload.AsSpan(0, 4));
        }

        public static uint SysUptime(byte[] payload)
        {
            return payload.Length >= 4 ? BinaryPrimitives.ReadUInt32LittleEndian(payload) : 0;
        }

        public static (byte ledIdx, byte state) SysLed(byte[] payload)
        {
            return payload.Length >= 2 ? (payload[0], payload[1]) : ((byte)0, (byte)0);
        }

        public static StepperConfig StepGetInfo(byte[] payload)
        {
            if (payload.Length < 8) throw new ArgumentException("STEP_GET_INFO payload must be 8 bytes");
            ushort xy = BinaryPrimitives.ReadUInt16LittleEndian(payload.AsSpan(0, 2));
            ushort z = BinaryPrimitives.ReadUInt16LittleEndian(payload.AsSpan(2, 2));
            ushort minRate = BinaryPrimitives.ReadUInt16LittleEndian(payload.AsSpan(4, 2));
            ushort maxRate = BinaryPrimitives.ReadUInt16LittleEndian(payload.AsSpan(6, 2));
            return new StepperConfig(xy, z, minRate, maxRate);
        }

        public static StepperStatus StepGetStatus(byte[] payload)
        {
            if (payload.Length < 3) throw new ArgumentException("STEP_GET_STATUS payload must be at least 3 bytes");
            return new StepperStatus(payload[0] != 0, payload[1] != 0, payload[2]);
        }

        public static bool StepEnable(byte[] payload)
        {
            return payload.Length >= 1 && payload[0] != 0;
        }

        public static (double xMm, double yMm) StepPosXY(byte[] payload)
        {
            if (payload.Length < 8) throw new ArgumentException("STEP_POS_XY payload must be 8 bytes");
            int xUm = BinaryPrimitives.ReadInt32LittleEndian(payload.AsSpan(0, 4));
            int yUm = BinaryPrimitives.ReadInt32LittleEndian(payload.AsSpan(4, 4));
            return (Units.UmToMm(xUm), Units.UmToMm(yUm));
        }

        public static double StepPosZ(byte[] payload)
        {
            if (payload.Length < 4) throw new ArgumentException("STEP_POS_Z payload must be 4 bytes");
            int zUm = BinaryPrimitives.ReadInt32LittleEndian(payload.AsSpan(0, 4));
            return Units.UmToMm(zUm);
        }

        public static ServoFeedbackDynamics ServoFbDynamics(byte[] payload)
        {
            if (payload.Length < 8) throw new ArgumentException("SERVO_FB_DYNAMICS payload must be 8 bytes");
            byte id = payload[0];
            short pos = BinaryPrimitives.ReadInt16LittleEndian(payload.AsSpan(1, 2));
            short spd = BinaryPrimitives.ReadInt16LittleEndian(payload.AsSpan(3, 2));
            short load = BinaryPrimitives.ReadInt16LittleEndian(payload.AsSpan(5, 2));
            bool moving = payload[7] != 0;
            return new ServoFeedbackDynamics(id, pos, spd, load, moving);
        }

        public static ServoFeedbackHealth ServoFbHealth(byte[] payload)
        {
            if (payload.Length < 8) throw new ArgumentException("SERVO_FB_HEALTH payload must be 8 bytes");
            byte id = payload[0];
            double volt = payload[1] / 10.0;
            sbyte temp = (sbyte)payload[2];
            byte status = payload[3];
            short curMa = BinaryPrimitives.ReadInt16LittleEndian(payload.AsSpan(4, 2));
            return new ServoFeedbackHealth(id, volt, temp, status, curMa);
        }

        public static HapticPingInfo HapticPing(byte[] payload)
        {
            if (payload.Length < 8) throw new ArgumentException("HAPTIC_PING payload must be 8 bytes");
            return new HapticPingInfo(
                Token: payload[0],
                Protocol: payload[1],
                FwMajor: payload[2],
                FwMinor: payload[3],
                State: payload[4],
                FrameCounter: payload[5],
                Errors: payload[6],
                UptimeS: payload[7]
            );
        }

        public static HapticUartDiag HapticUartDiag(byte[] payload)
        {
            if (payload.Length < 8) throw new ArgumentException("HAPTIC_DIAG payload must be 8 bytes");
            byte flags = payload[0];
            byte errs = payload[1];
            ushort rxBytes = (ushort)(payload[2] | (payload[3] << 8));
            byte lastRx = payload[4];
            byte txFail = payload[5];
            byte timeouts = payload[6];
            byte replies = payload[7];
            return new HapticUartDiag(flags, errs, rxBytes, lastRx, txFail, timeouts, replies);
        }

        public static HapticMappingConfig HapticMapping(byte[] payload)
        {
            if (payload.Length < 8) throw new ArgumentException("HAPTIC_CFG payload must be 8 bytes");
            ushort raw0 = BinaryPrimitives.ReadUInt16LittleEndian(payload.AsSpan(0, 2));
            bool saved  = (raw0 & 0x8000) != 0;
            ushort vminMv = (ushort)(raw0 & 0x7FFF);
            ushort vmaxMv = BinaryPrimitives.ReadUInt16LittleEndian(payload.AsSpan(2, 2));
            ushort fthCn  = BinaryPrimitives.ReadUInt16LittleEndian(payload.AsSpan(4, 2));
            ushort fratCn = BinaryPrimitives.ReadUInt16LittleEndian(payload.AsSpan(6, 2));
            return new HapticMappingConfig(
                VMinV: vminMv / 1000.0,
                VMaxV: vmaxMv / 1000.0,
                FThreshN: fthCn / 100.0,
                FRatedN: fratCn / 100.0,
                SavedInEeprom: saved);
        }

        public static (ushort ch0, ushort ch1, ushort ch2, ushort ch3) SensorAdcCh0_3(byte[] payload)
        {
            if (payload.Length < 8) throw new ArgumentException("SENSOR_ADC_CH0_3 payload must be 8 bytes");
            return (
                BinaryPrimitives.ReadUInt16LittleEndian(payload.AsSpan(0, 2)),
                BinaryPrimitives.ReadUInt16LittleEndian(payload.AsSpan(2, 2)),
                BinaryPrimitives.ReadUInt16LittleEndian(payload.AsSpan(4, 2)),
                BinaryPrimitives.ReadUInt16LittleEndian(payload.AsSpan(6, 2))
            );
        }

        public static (ushort ch4, byte seq, byte status, ushort rxOk, ushort crcErr) SensorAdcCh4Meta(byte[] payload)
        {
            if (payload.Length < 8) throw new ArgumentException("SENSOR_ADC_CH4_META payload must be 8 bytes");
            return (
                BinaryPrimitives.ReadUInt16LittleEndian(payload.AsSpan(0, 2)),
                payload[2],
                payload[3],
                BinaryPrimitives.ReadUInt16LittleEndian(payload.AsSpan(4, 2)),
                BinaryPrimitives.ReadUInt16LittleEndian(payload.AsSpan(6, 2))
            );
        }

        public static (ushort[] channels, byte status) SensorAdcAll(byte[] payload)
        {
            if (payload.Length < 8) throw new ArgumentException("SENSOR_ADC_ALL payload must be 8 bytes");
            ulong raw = BinaryPrimitives.ReadUInt64LittleEndian(payload.AsSpan(0, 8));
            var channels = new ushort[5];
            for (int i = 0; i < 5; i++)
            {
                channels[i] = (ushort)((raw >> (12 * i)) & 0xFFF);
            }
            byte status = (byte)((raw >> 60) & 0x0F);
            return (channels, status);
        }

        public static SpiAdcCtrlStatus SensorSpiCtrl(byte[] payload)
        {
            if (payload.Length < 8) throw new ArgumentException("SENSOR_SPI_CTRL payload must be 8 bytes");
            return new SpiAdcCtrlStatus(
                Enabled: payload[0] != 0,
                BusId: payload[1],
                CsIdx: payload[2],
                PeriodMs: payload[3],
                TotalFrames: BinaryPrimitives.ReadUInt32LittleEndian(payload.AsSpan(4, 4))
            );
        }
    }
}
