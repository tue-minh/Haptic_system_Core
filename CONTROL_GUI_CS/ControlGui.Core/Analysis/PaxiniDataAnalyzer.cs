using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;

namespace ControlGui.Core.Analysis
{
    public class SensorSessionData
    {
        public string FilePath { get; set; } = "";
        public string Tag { get; set; } = "Unknown";
        public List<double> TimeRel { get; set; } = new();
        public List<double> Fx { get; set; } = new();
        public List<double> Fy { get; set; } = new();
        public List<double> Fz { get; set; } = new();
        public List<double> FMag { get; set; } = new();
        public List<double> FShear { get; set; } = new();
        public List<double> Mu { get; set; } = new();
        public List<double> CopX { get; set; } = new();
        public List<double> CopY { get; set; } = new();
        public List<double> ContactTaxels { get; set; } = new();
        public List<float[,]> TaxelFrames { get; set; } = new();
    }

    public static class PaxiniDataAnalyzer
    {
        public static SensorSessionData LoadPerSensorCsv(string path)
        {
            var data = new SensorSessionData
            {
                FilePath = path,
                Tag = path.Contains("_L_") ? "Left" : (path.Contains("_R_") ? "Right" : "Unknown")
            };

            var lines = File.ReadAllLines(path);
            int headerIdx = -1;
            for (int i = 0; i < lines.Length; i++)
            {
                string line = lines[i].Trim();
                if (line.StartsWith("#") || string.IsNullOrEmpty(line)) continue;
                headerIdx = i;
                break;
            }

            if (headerIdx < 0 || headerIdx >= lines.Length - 1) return data;

            var header = lines[headerIdx].Split(',').Select(s => s.Trim()).ToList();
            int idxT = header.IndexOf("t_rel_s");
            int idxFx = header.IndexOf("Fx_N");
            int idxFy = header.IndexOf("Fy_N");
            int idxFz = header.IndexOf("Fz_N");
            int idxFMag = header.IndexOf("F_mag_N");
            int idxFShear = header.IndexOf("F_shear_N");
            int idxMu = header.IndexOf("mu_Ft_over_Fz");
            int idxCopX = header.IndexOf("CoP_x_mm");
            int idxCopY = header.IndexOf("CoP_y_mm");
            int idxContact = header.IndexOf("contact_taxels");

            var inv = CultureInfo.InvariantCulture;

            for (int i = headerIdx + 1; i < lines.Length; i++)
            {
                string line = lines[i].Trim();
                if (string.IsNullOrEmpty(line)) continue;
                var parts = line.Split(',');
                if (parts.Length <= idxFz) continue;

                double ParseVal(int idx) => (idx >= 0 && idx < parts.Length && double.TryParse(parts[idx], NumberStyles.Float, inv, out var v)) ? v : double.NaN;

                data.TimeRel.Add(ParseVal(idxT));
                data.Fx.Add(ParseVal(idxFx));
                data.Fy.Add(ParseVal(idxFy));
                data.Fz.Add(ParseVal(idxFz));
                data.FMag.Add(ParseVal(idxFMag));
                data.FShear.Add(ParseVal(idxFShear));
                data.Mu.Add(ParseVal(idxMu));
                data.CopX.Add(ParseVal(idxCopX));
                data.CopY.Add(ParseVal(idxCopY));
                data.ContactTaxels.Add(ParseVal(idxContact));
            }

            return data;
        }

        public static string GenerateDiagnosticReport(SensorSessionData left, SensorSessionData right)
        {
            var sb = new StringBuilder();
            sb.AppendLine("================================================================================");
            sb.AppendLine("           PaXini Tactile Acquisition Session Diagnostic Summary                ");
            sb.AppendLine("================================================================================");

            double maxFzL = left.Fz.Count > 0 ? left.Fz.Where(v => !double.IsNaN(v)).DefaultIfEmpty(0).Max() : 0;
            double maxFzR = right.Fz.Count > 0 ? right.Fz.Where(v => !double.IsNaN(v)).DefaultIfEmpty(0).Max() : 0;

            double maxMagL = left.FMag.Count > 0 ? left.FMag.Where(v => !double.IsNaN(v)).DefaultIfEmpty(0).Max() : 0;
            double maxMagR = right.FMag.Count > 0 ? right.FMag.Where(v => !double.IsNaN(v)).DefaultIfEmpty(0).Max() : 0;

            double maxMuL = left.Mu.Count > 0 ? left.Mu.Where(v => !double.IsNaN(v)).DefaultIfEmpty(0).Max() : 0;
            double maxMuR = right.Mu.Count > 0 ? right.Mu.Where(v => !double.IsNaN(v)).DefaultIfEmpty(0).Max() : 0;

            sb.AppendLine($"[Left Sensor]");
            sb.AppendLine($"  File: {Path.GetFileName(left.FilePath)}");
            sb.AppendLine($"  Frames: {left.TimeRel.Count}");
            sb.AppendLine($"  Peak Normal Force (Fz): {maxFzL:F2} N");
            sb.AppendLine($"  Peak Total Force (|F|): {maxMagL:F2} N");
            sb.AppendLine($"  Max Friction Utilization (mu): {maxMuL:F3}");

            sb.AppendLine();
            sb.AppendLine($"[Right Sensor]");
            sb.AppendLine($"  File: {Path.GetFileName(right.FilePath)}");
            sb.AppendLine($"  Frames: {right.TimeRel.Count}");
            sb.AppendLine($"  Peak Normal Force (Fz): {maxFzR:F2} N");
            sb.AppendLine($"  Peak Total Force (|F|): {maxMagR:F2} N");
            sb.AppendLine($"  Max Friction Utilization (mu): {maxMuR:F3}");

            sb.AppendLine();
            sb.AppendLine("[Grasp Symmetry & Balance]");
            double totalPeak = maxFzL + maxFzR;
            if (totalPeak > 0)
            {
                double asymmetry = Math.Abs(maxFzL - maxFzR) / totalPeak * 100.0;
                sb.AppendLine($"  Grip Peak Asymmetry: {asymmetry:F1}%");
                sb.AppendLine(asymmetry < 15.0 ? "  Balance Status: EXCELLENT / SYMMETRIC" : "  Balance Status: ASYMMETRIC LOAD DISTRIBUTION");
            }

            sb.AppendLine("================================================================================");
            return sb.ToString();
        }
    }
}
