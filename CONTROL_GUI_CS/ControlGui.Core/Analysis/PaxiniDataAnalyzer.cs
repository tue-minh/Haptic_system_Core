using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using Microsoft.Data.Sqlite;

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

    public class RecordedSessionMeta
    {
        public string TableName { get; set; } = "";
        public string CreatedAt { get; set; } = "";
        public double SampleRateHz { get; set; }
        public long RowCount { get; set; }
        public double DurationSeconds { get; set; }
        public string Meta { get; set; } = "";
    }

    public static class PaxiniDataAnalyzer
    {
        public static List<RecordedSessionMeta> GetSessionTables(string dbPath)
        {
            var sessions = new List<RecordedSessionMeta>();
            if (!File.Exists(dbPath)) return sessions;

            using var conn = new SqliteConnection($"Data Source={dbPath};Mode=ReadOnly;");
            conn.Open();

            // Check if _recording_sessions exists
            using var cmdCheck = conn.CreateCommand();
            cmdCheck.CommandText = "SELECT COUNT(*) FROM sqlite_master WHERE type='table' AND name='_recording_sessions';";
            long hasCatalog = (long)(cmdCheck.ExecuteScalar() ?? 0);

            if (hasCatalog > 0)
            {
                using var cmd = conn.CreateCommand();
                cmd.CommandText = "SELECT table_name, created_at, sample_rate_hz, row_count, duration_s, meta FROM _recording_sessions ORDER BY id DESC;";
                using var reader = cmd.ExecuteReader();
                while (reader.Read())
                {
                    sessions.Add(new RecordedSessionMeta
                    {
                        TableName = reader.GetString(0),
                        CreatedAt = reader.IsDBNull(1) ? "" : reader.GetString(1),
                        SampleRateHz = reader.IsDBNull(2) ? 0.0 : reader.GetDouble(2),
                        RowCount = reader.IsDBNull(3) ? 0 : reader.GetInt64(3),
                        DurationSeconds = reader.IsDBNull(4) ? 0.0 : reader.GetDouble(4),
                        Meta = reader.IsDBNull(5) ? "" : reader.GetString(5)
                    });
                }
            }
            else
            {
                // Fallback: list all tables starting with rec_
                using var cmd = conn.CreateCommand();
                cmd.CommandText = "SELECT name FROM sqlite_master WHERE type='table' AND name LIKE 'rec_%' ORDER BY name DESC;";
                using var reader = cmd.ExecuteReader();
                while (reader.Read())
                {
                    sessions.Add(new RecordedSessionMeta
                    {
                        TableName = reader.GetString(0)
                    });
                }
            }

            return sessions;
        }

        public static (SensorSessionData left, SensorSessionData right) LoadSessionFromSqlite(string dbPath, string tableName)
        {
            var left = new SensorSessionData { FilePath = $"{dbPath} [{tableName}]", Tag = "Left" };
            var right = new SensorSessionData { FilePath = $"{dbPath} [{tableName}]", Tag = "Right" };

            if (!File.Exists(dbPath)) return (left, right);

            using var conn = new SqliteConnection($"Data Source={dbPath};Mode=ReadOnly;");
            conn.Open();

            using var cmd = conn.CreateCommand();
            cmd.CommandText = $@"
                SELECT 
                    t_rel_s,
                    L_Fx_N, L_Fy_N, L_Fz_N, L_F_mag_N, L_F_shear_N, L_mu_Ft_over_Fz, L_CoP_x_mm, L_CoP_y_mm, L_contact_taxels,
                    R_Fx_N, R_Fy_N, R_Fz_N, R_F_mag_N, R_F_shear_N, R_mu_Ft_over_Fz, R_CoP_x_mm, R_CoP_y_mm, R_contact_taxels
                FROM [{tableName}]
                ORDER BY row_id ASC;
            ";

            using var reader = cmd.ExecuteReader();
            while (reader.Read())
            {
                double t = reader.IsDBNull(0) ? double.NaN : reader.GetDouble(0);

                // Left
                left.TimeRel.Add(t);
                left.Fx.Add(reader.IsDBNull(1) ? double.NaN : reader.GetDouble(1));
                left.Fy.Add(reader.IsDBNull(2) ? double.NaN : reader.GetDouble(2));
                left.Fz.Add(reader.IsDBNull(3) ? double.NaN : reader.GetDouble(3));
                left.FMag.Add(reader.IsDBNull(4) ? double.NaN : reader.GetDouble(4));
                left.FShear.Add(reader.IsDBNull(5) ? double.NaN : reader.GetDouble(5));
                left.Mu.Add(reader.IsDBNull(6) ? double.NaN : reader.GetDouble(6));
                left.CopX.Add(reader.IsDBNull(7) ? double.NaN : reader.GetDouble(7));
                left.CopY.Add(reader.IsDBNull(8) ? double.NaN : reader.GetDouble(8));
                left.ContactTaxels.Add(reader.IsDBNull(9) ? double.NaN : reader.GetDouble(9));

                // Right
                right.TimeRel.Add(t);
                right.Fx.Add(reader.IsDBNull(10) ? double.NaN : reader.GetDouble(10));
                right.Fy.Add(reader.IsDBNull(11) ? double.NaN : reader.GetDouble(11));
                right.Fz.Add(reader.IsDBNull(12) ? double.NaN : reader.GetDouble(12));
                right.FMag.Add(reader.IsDBNull(13) ? double.NaN : reader.GetDouble(13));
                right.FShear.Add(reader.IsDBNull(14) ? double.NaN : reader.GetDouble(14));
                right.Mu.Add(reader.IsDBNull(15) ? double.NaN : reader.GetDouble(15));
                right.CopX.Add(reader.IsDBNull(16) ? double.NaN : reader.GetDouble(16));
                right.CopY.Add(reader.IsDBNull(17) ? double.NaN : reader.GetDouble(17));
                right.ContactTaxels.Add(reader.IsDBNull(18) ? double.NaN : reader.GetDouble(18));
            }

            return (left, right);
        }

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
            sb.AppendLine($"  Source: {Path.GetFileName(left.FilePath)}");
            sb.AppendLine($"  Frames: {left.TimeRel.Count}");
            sb.AppendLine($"  Peak Normal Force (Fz): {maxFzL:F2} N");
            sb.AppendLine($"  Peak Total Force (|F|): {maxMagL:F2} N");
            sb.AppendLine($"  Max Friction Utilization (mu): {maxMuL:F3}");

            sb.AppendLine();
            sb.AppendLine($"[Right Sensor]");
            sb.AppendLine($"  Source: {Path.GetFileName(right.FilePath)}");
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
