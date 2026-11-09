using System;
using ControlGui.Core.TueUsb;

namespace ControlGui.Core.Haptic
{
    public class TueBinaryEncoder
    {
        private byte _seq = 0;

        public byte Seq => _seq;

        public byte[] Encode(byte[] payload)
        {
            if (payload == null || payload.Length != HapticConstants.Dlc)
            {
                throw new ArgumentException($"Payload must be exactly {HapticConstants.Dlc} bytes, got {payload?.Length ?? 0}");
            }

            byte seq = _seq++;
            var frame = new Frame(FrameType.Req, HapticConstants.CanId, payload, seq);
            return frame.ToBytes();
        }
    }
}
