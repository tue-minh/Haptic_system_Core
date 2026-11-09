using System;
using System.Windows;
using System.Windows.Controls;
using ControlGui.Core.Paxini;
using ControlGui.Wpf.Controls;

namespace ControlGui.Wpf.Panels
{
    public partial class SensorMonitorPanel : UserControl
    {
        public event Action? TareLeftRequested;
        public event Action? TareRightRequested;
        public event Action? TareBothRequested;
        public event Action? ResetTareRequested;

        public SensorMonitorPanel()
        {
            InitializeComponent();
        }

        private void OnDirectionChanged(object sender, RoutedEventArgs e)
        {
            if (HistoryPlot == null) return;

            if (RbFz.IsChecked == true) HistoryPlot.DirectionFilter = "Fz";
            else if (RbFx.IsChecked == true) HistoryPlot.DirectionFilter = "Fx";
            else if (RbFy.IsChecked == true) HistoryPlot.DirectionFilter = "Fy";
            else if (RbAll.IsChecked == true) HistoryPlot.DirectionFilter = "All";
        }

        private void OnOverlayChanged(object sender, RoutedEventArgs e)
        {
            if (HistoryPlot == null) return;
            HistoryPlot.ShowMag = ChkMag.IsChecked == true;
            HistoryPlot.ShowShear = ChkShear.IsChecked == true;
        }

        private void OnTareLClick(object sender, RoutedEventArgs e) => TareLeftRequested?.Invoke();
        private void OnTareRClick(object sender, RoutedEventArgs e) => TareRightRequested?.Invoke();
        private void OnTareBothClick(object sender, RoutedEventArgs e) => TareBothRequested?.Invoke();
        private void OnResetTareClick(object sender, RoutedEventArgs e) => ResetTareRequested?.Invoke();

        public void UpdateTelemetry(SensorReading? left, SensorReading? right)
        {
            // Push to plot
            double now = left?.T ?? right?.T ?? 0.0;
            PlotDataPoint? ptL = left != null ? new PlotDataPoint { T = left.T, Fx = left.Fx, Fy = left.Fy, Fz = left.Fz, FMag = left.FMag, FShear = left.FShear } : null;
            PlotDataPoint? ptR = right != null ? new PlotDataPoint { T = right.T, Fx = right.Fx, Fy = right.Fy, Fz = right.Fz, FMag = right.FMag, FShear = right.FShear } : null;

            HistoryPlot.PushPoints(ptL, ptR, now);

            // Numerical readouts Left
            if (left != null)
            {
                TxtLFz.Text = $"{left.Fz:F2}";
                TxtLFShear.Text = $"{left.FShear:F2}";
                TxtLFMag.Text = $"{left.FMag:F2}";
                TxtLMu.Text = double.IsNaN(left.Mu) ? "nan" : $"{left.Mu:F3}";
                TxtLTilt.Text = double.IsNaN(left.Tilt) ? "nan" : $"{left.Tilt:F1}";
                TxtLDir.Text = double.IsNaN(left.ShearDir) ? "nan" : $"{left.ShearDir:+0.0;-0.0;0.0}";
                TxtLContact.Text = $"{left.ContactTaxels:F0}";
                TxtLPeakIdx.Text = double.IsNaN(left.PeakTaxel) ? "nan" : $"#{left.PeakTaxel:F0}";
                TxtLPeakFz.Text = $"{left.PeakFz:F2}";
            }

            // Numerical readouts Right
            if (right != null)
            {
                TxtRFz.Text = $"{right.Fz:F2}";
                TxtRFShear.Text = $"{right.FShear:F2}";
                TxtRFMag.Text = $"{right.FMag:F2}";
                TxtRMu.Text = double.IsNaN(right.Mu) ? "nan" : $"{right.Mu:F3}";
                TxtRTilt.Text = double.IsNaN(right.Tilt) ? "nan" : $"{right.Tilt:F1}";
                TxtRDir.Text = double.IsNaN(right.ShearDir) ? "nan" : $"{right.ShearDir:+0.0;-0.0;0.0}";
                TxtRContact.Text = $"{right.ContactTaxels:F0}";
                TxtRPeakIdx.Text = double.IsNaN(right.PeakTaxel) ? "nan" : $"#{right.PeakTaxel:F0}";
                TxtRPeakFz.Text = $"{right.PeakFz:F2}";
            }
        }
    }
}
