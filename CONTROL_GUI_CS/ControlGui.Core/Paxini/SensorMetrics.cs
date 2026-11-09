using System;

namespace ControlGui.Core.Paxini
{
    public static class SensorMetrics
    {
        public static SensorReading Compute(
            double t,
            double epoch,
            double fx,
            double fy,
            double fz,
            float[,] taxels,
            Point3D[] coordinates,
            double taxelThresholdN = 0.1,
            double forceThresholdN = 0.2)
        {
            double fShear = Math.Sqrt(fx * fx + fy * fy);
            double fMag = Math.Sqrt(fShear * fShear + fz * fz);

            int n = taxels.GetLength(0);
            double wSum = 0.0;
            double copSumX = 0.0;
            double copSumY = 0.0;
            int contactN = 0;

            int peakIdx = 0;
            double peakFz = double.NegativeInfinity;

            for (int i = 0; i < n; i++)
            {
                double fz_i = taxels[i, 2];
                if (fz_i > peakFz)
                {
                    peakFz = fz_i;
                    peakIdx = i;
                }

                if (fz_i >= taxelThresholdN)
                {
                    contactN++;
                    wSum += fz_i;
                    copSumX += fz_i * coordinates[i].X;
                    copSumY += fz_i * coordinates[i].Y;
                }
            }

            double copX = (wSum > 0.0 && contactN > 0) ? (copSumX / wSum) : double.NaN;
            double copY = (wSum > 0.0 && contactN > 0) ? (copSumY / wSum) : double.NaN;

            double peakTaxelNum = (peakFz >= taxelThresholdN) ? (peakIdx + 1) : double.NaN;
            if (peakFz < 0) peakFz = 0;

            double shearDir;
            double tilt;
            double mu;

            if (fMag >= forceThresholdN)
            {
                shearDir = Math.Atan2(fy, fx) * (180.0 / Math.PI);
                tilt = Math.Atan2(fShear, fz) * (180.0 / Math.PI);
                mu = (fz >= forceThresholdN) ? (fShear / fz) : double.NaN;
            }
            else
            {
                shearDir = double.NaN;
                tilt = double.NaN;
                mu = double.NaN;
            }

            return new SensorReading(
                T: t,
                Epoch: epoch,
                Fx: fx,
                Fy: fy,
                Fz: fz,
                FMag: fMag,
                FShear: fShear,
                Mu: mu,
                CopX: copX,
                CopY: copY,
                ShearDir: shearDir,
                Tilt: tilt,
                PeakTaxel: peakTaxelNum,
                PeakFz: peakFz,
                ContactTaxels: contactN,
                Taxels: taxels
            );
        }
    }
}
