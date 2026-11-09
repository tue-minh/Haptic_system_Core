using System;

namespace ControlGui.Core.Paxini
{
    public record SensorReading(
        double T,
        double Epoch,
        double Fx,
        double Fy,
        double Fz,
        double FMag,
        double FShear,
        double Mu,
        double CopX,
        double CopY,
        double ShearDir,
        double Tilt,
        double PeakTaxel,
        double PeakFz,
        double ContactTaxels,
        float[,]? Taxels // [52, 3] representing [Fx, Fy, Fz]
    )
    {
        public static SensorReading Empty => new(
            0, 0, 0, 0, 0, 0, 0, double.NaN,
            double.NaN, double.NaN, double.NaN, double.NaN,
            double.NaN, 0, 0, null);
    }
}
