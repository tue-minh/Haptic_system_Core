using System;
using System.Windows.Media;

namespace ControlGui.Wpf
{
    public static class Theme
    {
        public static readonly Color BgColor = Color.FromRgb(0xFA, 0xFA, 0xFA);
        public static readonly Color PanelColor = Color.FromRgb(0xFF, 0xFF, 0xFF);
        public static readonly Color FieldColor = Color.FromRgb(0xF5, 0xF5, 0xF5);
        public static readonly Color BorderColor = Color.FromRgb(0xE0, 0xE0, 0xE0);
        public static readonly Color BorderHoverColor = Color.FromRgb(0x9E, 0x9E, 0x9E);
        public static readonly Color TextColor = Color.FromRgb(0x21, 0x21, 0x21);
        public static readonly Color TextDimColor = Color.FromRgb(0x75, 0x75, 0x75);

        public static readonly Color PrimaryColor = Color.FromRgb(0x21, 0x96, 0xF3);
        public static readonly Color DangerColor = Color.FromRgb(0xD3, 0x2F, 0x2F);
        public static readonly Color SuccessColor = Color.FromRgb(0x38, 0x8E, 0x3C);
        public static readonly Color WarningColor = Color.FromRgb(0xF5, 0x7C, 0x00);

        public static readonly Color LedOffColor = Color.FromRgb(0xB0, 0xBE, 0xC5);
        public static readonly Color LedConnectingColor = Color.FromRgb(0xFD, 0xD8, 0x35);
        public static readonly Color LedReadyColor = Color.FromRgb(0x4C, 0xAF, 0x50);
        public static readonly Color LedErrorColor = Color.FromRgb(0xF4, 0x43, 0x36);
        public static readonly Color LedWarnColor = Color.FromRgb(0xFF, 0x98, 0x00);

        public static readonly SolidColorBrush BgBrush = new(BgColor);
        public static readonly SolidColorBrush PanelBrush = new(PanelColor);
        public static readonly SolidColorBrush FieldBrush = new(FieldColor);
        public static readonly SolidColorBrush BorderBrush = new(BorderColor);
        public static readonly SolidColorBrush BorderHoverBrush = new(BorderHoverColor);
        public static readonly SolidColorBrush TextBrush = new(TextColor);
        public static readonly SolidColorBrush TextDimBrush = new(TextDimColor);

        public static readonly SolidColorBrush PrimaryBrush = new(PrimaryColor);
        public static readonly SolidColorBrush DangerBrush = new(DangerColor);
        public static readonly SolidColorBrush SuccessBrush = new(SuccessColor);
        public static readonly SolidColorBrush WarningBrush = new(WarningColor);

        public static Color GetColormapColor(double t, string mapName = "Greys")
        {
            t = Math.Clamp(t, 0.0, 1.0);
            return mapName switch
            {
                "viridis" or "Viridis" => Viridis(t),
                "inferno" or "Inferno" => Inferno(t),
                "cividis" or "Cividis" => Cividis(t),
                _ => Greys(t)
            };
        }

        private static Color Greys(double t)
        {
            byte v = (byte)(235 - t * 200); // 0 = light grey, 1 = dark charcoal
            return Color.FromRgb(v, v, v);
        }

        private static Color Viridis(double t)
        {
            // Approximate Viridis polynomial ramp
            double r = Math.Clamp(0.267 + t * (0.004 + t * (2.82 - t * 2.09)), 0.0, 1.0);
            double g = Math.Clamp(0.004 + t * (1.40 + t * (-0.38 - t * 0.05)), 0.0, 1.0);
            double b = Math.Clamp(0.329 + t * (0.87 - t * (1.85 - t * 0.65)), 0.0, 1.0);
            return Color.FromRgb((byte)(r * 255), (byte)(g * 255), (byte)(b * 255));
        }

        private static Color Inferno(double t)
        {
            double r = Math.Clamp(t < 0.5 ? t * 2.0 * 0.8 : 0.8 + (t - 0.5) * 2.0 * 0.2, 0.0, 1.0);
            double g = Math.Clamp(t < 0.3 ? 0.0 : (t < 0.7 ? (t - 0.3) / 0.4 * 0.6 : 0.6 + (t - 0.7) / 0.3 * 0.4), 0.0, 1.0);
            double b = Math.Clamp(t < 0.5 ? (1.0 - t * 2.0) * 0.5 : (t - 0.5) * 0.6, 0.0, 1.0);
            return Color.FromRgb((byte)(r * 255), (byte)(g * 255), (byte)(b * 255));
        }

        private static Color Cividis(double t)
        {
            double r = Math.Clamp(0.0 + t * 0.9, 0.0, 1.0);
            double g = Math.Clamp(0.13 + t * 0.75, 0.0, 1.0);
            double b = Math.Clamp(0.33 + (1.0 - t) * 0.3 + t * 0.2, 0.0, 1.0);
            return Color.FromRgb((byte)(r * 255), (byte)(g * 255), (byte)(b * 255));
        }
    }
}
