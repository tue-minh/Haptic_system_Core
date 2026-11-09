using System;

namespace ControlGui.Core.Haptic
{
    public class HapticPayloadSynthesizer
    {
        public double ContactThresh { get; set; } = 0.20;
        public double ForceThresh { get; set; } = 0.20;
        public double SlipMu { get; set; } = 0.50;
        public double StaleSeconds { get; set; } = 0.10;

        private byte _counter = 0;

        public static byte ClampU8(double val)
        {
            if (double.IsNaN(val) || double.IsInfinity(val) || val <= 0.0) return 0;
            long q = (long)Math.Round(val * 10.0);
            return (byte)Math.Clamp(q, 0, 255);
        }

        public byte[] Synthesize(
            Paxini.SensorReading? left,
            Paxini.SensorReading? right,
            bool slipAlert = false,
            bool tareActive = false,
            double? nowSec = null)
        {
            double now = nowSec ?? (DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0);

            bool freshL = left != null && (now - left.T <= StaleSeconds || left.T == 0);
            bool freshR = right != null && (now - right.T <= StaleSeconds || right.T == 0);

            double fzL = freshL && left != null ? left.Fz : 0.0;
            double fxL = freshL && left != null ? left.Fx : 0.0;
            double fyL = freshL && left != null ? left.Fy : 0.0;

            double fzR = freshR && right != null ? right.Fz : 0.0;
            double fxR = freshR && right != null ? right.Fx : 0.0;
            double fyR = freshR && right != null ? right.Fy : 0.0;

            double fzAvg;
            double fxNet;
            double fyNet;

            if (freshL && freshR)
            {
                fzAvg = (fzL + fzR) / 2.0;
                fxNet = (fxL - fxR) / 2.0;
                fyNet = (fyL + fyR) / 2.0;
            }
            else if (freshL)
            {
                fzAvg = fzL;
                fxNet = fxL;
                fyNet = fyL;
            }
            else if (freshR)
            {
                fzAvg = fzR;
                fxNet = -fxR;
                fyNet = fyR;
            }
            else
            {
                fzAvg = 0.0;
                fxNet = 0.0;
                fyNet = 0.0;
            }

            // Directional mapping:
            // Move in +Y -> Valve A blows; Move in -Y -> Valve C blows
            double fA = fyNet > 0 ? fyNet : 0.0;
            double fC = fyNet < 0 ? Math.Abs(fyNet) : 0.0;

            // Move in +X -> Valve E blows; Move in -X -> Valve D blows
            double fD = fxNet < 0 ? Math.Abs(fxNet) : 0.0;
            double fE = fxNet > 0 ? fxNet : 0.0;

            double fB = fzAvg;

            byte byteA = ClampU8(fA);
            byte byteB = ClampU8(fB);
            byte byteC = ClampU8(fC);
            byte byteD = ClampU8(fD);
            byte byteE = ClampU8(fE);
            byte byteSpare = 0;

            byte flags = 0;
            if (freshL && fzL > ForceThresh) flags |= HapticConstants.FlagLeftContact;
            if (freshR && fzR > ForceThresh) flags |= HapticConstants.FlagRightContact;

            bool isSlipL = freshL && left != null && left.Fz > ForceThresh && (left.FShear / left.Fz > SlipMu);
            bool isSlipR = freshR && right != null && right.Fz > ForceThresh && (right.FShear / right.Fz > SlipMu);

            if (slipAlert || isSlipL || isSlipR) flags |= HapticConstants.FlagSlip;
            if (tareActive) flags |= HapticConstants.FlagTare;

            byte[] payload = new byte[8]
            {
                byteA,
                byteB,
                byteC,
                byteD,
                byteE,
                byteSpare,
                flags,
                _counter
            };

            _counter++;
            return payload;
        }
    }
}
