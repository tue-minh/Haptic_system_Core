using System;
using System.Collections.Generic;

namespace ControlGui.Core.Recording
{
    public static class RecordingSchema
    {
        public static readonly string[] LeadColumns = new string[]
        {
            "t_rel_s",
            "epoch_s",
            "Stepper_X_mm",
            "Stepper_Y_mm",
            "Stepper_Z_mm",
            "Gripper_Pos_raw",
            "Gripper_Volt_V"
        };

        public static readonly string[] SensorMetrics = new string[]
        {
            "Fx_N",
            "Fy_N",
            "Fz_N",
            "F_mag_N",
            "F_shear_N",
            "mu_Ft_over_Fz",
            "CoP_x_mm",
            "CoP_y_mm",
            "shear_dir_deg",
            "tilt_deg",
            "peak_taxel",
            "peak_Fz_N",
            "contact_taxels"
        };

        public static List<string> GetSensorBlockColumns(string tag)
        {
            var cols = new List<string>();
            foreach (var m in SensorMetrics)
            {
                cols.Add($"{tag}_{m}");
            }
            for (int k = 1; k <= 52; k++)
            {
                cols.Add($"{tag}_t{k}_Fx");
                cols.Add($"{tag}_t{k}_Fy");
                cols.Add($"{tag}_t{k}_Fz");
            }
            return cols;
        }

        public static List<string> GetMasterColumns()
        {
            var cols = new List<string>(LeadColumns);
            cols.AddRange(GetSensorBlockColumns("L"));
            cols.AddRange(GetSensorBlockColumns("R"));
            return cols;
        }

        public static List<string> GetPerSensorColumns()
        {
            var cols = new List<string> { "t_rel_s", "epoch_s" };
            cols.AddRange(SensorMetrics);
            for (int k = 1; k <= 52; k++)
            {
                cols.Add($"t{k}_Fx");
                cols.Add($"t{k}_Fy");
                cols.Add($"t{k}_Fz");
            }
            return cols;
        }
    }
}
