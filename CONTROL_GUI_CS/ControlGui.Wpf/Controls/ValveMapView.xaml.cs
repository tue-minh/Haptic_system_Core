using System;
using System.Collections.Generic;
using System.Globalization;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using ControlGui.Core.Haptic;

namespace ControlGui.Wpf.Controls
{
    public class ValveDisplayItem
    {
        public string Name { get; set; } = "";
        public string Label { get; set; } = "";
        public double Force { get; set; }
        public string ForceFormatted => $"{Force:F1} N";
        public double Voltage { get; set; }
        public string VoltageFormatted => $"{Voltage:F2} V";
        public byte Raw { get; set; }
        public double Weight { get; set; }
        public string WeightFormatted => $"{Weight * 100:F0}%";
    }

    public partial class ValveMapView : UserControl
    {
        private double _fullScaleN = 25.5;
        private string _colormap = "Greys";
        private HapticSnapshot? _snapshot;

        public ValveMapView()
        {
            InitializeComponent();
        }

        private void OnFullScaleChanged(object sender, TextChangedEventArgs e)
        {
            if (double.TryParse(TxtFullScale?.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out var v) && v > 0)
            {
                _fullScaleN = v;
                Redraw();
            }
        }

        private void OnColormapChanged(object sender, SelectionChangedEventArgs e)
        {
            if (CmbColormap?.SelectedItem is ComboBoxItem item)
            {
                _colormap = item.Content.ToString() ?? "Greys";
                Redraw();
            }
        }

        private void OnCanvasSizeChanged(object sender, SizeChangedEventArgs e)
        {
            Redraw();
        }

        public void Update(HapticSnapshot? snap, bool isStreaming)
        {
            _snapshot = snap;

            if (LblStatus != null)
            {
                if (isStreaming)
                {
                    LblStatus.Text = $"Streaming ({snap?.RateHz:F0} Hz)";
                    LblStatus.Foreground = Theme.SuccessBrush;
                }
                else
                {
                    LblStatus.Text = "Preview (idle)";
                    LblStatus.Foreground = Theme.TextDimBrush;
                }
            }

            if (TxtFrameHex != null && snap != null)
            {
                TxtFrameHex.Text = $"Wire: {snap.FrameHex}";
                TxtFlags.Text = $"Flags: 0x{snap.Flags:X2} [{(snap.FlagNames.Count > 0 ? string.Join(", ", snap.FlagNames) : "none")}]";
            }

            var items = new List<ValveDisplayItem>();
            if (snap?.Valves != null)
            {
                foreach (var v in snap.Valves)
                {
                    items.Add(new ValveDisplayItem
                    {
                        Name = v.Name,
                        Label = v.Label,
                        Force = v.Force,
                        Voltage = v.Voltage,
                        Raw = v.Raw,
                        Weight = v.Weight
                    });
                }
            }
            if (GridValves != null)
            {
                GridValves.ItemsSource = items;
            }

            Redraw();
        }

        private void Redraw()
        {
            if (CanvasValves == null || CanvasValves.ActualWidth <= 10 || CanvasValves.ActualHeight <= 10) return;

            CanvasValves.Children.Clear();

            double w = CanvasValves.ActualWidth;
            double h = CanvasValves.ActualHeight;

            // Valve layout range: X -10..+10, Y -2..26
            double minX = -12.0, maxX = 12.0;
            double minY = -4.0, maxY = 26.0;

            double scaleX = w / (maxX - minX);
            double scaleY = h / (maxY - minY);
            double scale = Math.Min(scaleX, scaleY) * 0.85;

            double cx = w / 2.0;
            double cy = h / 2.0;
            double midY = (minY + maxY) / 2.0;

            Point ToScreen(double mx, double my)
            {
                double sx = cx + mx * scale;
                double sy = cy - (my - midY) * scale;
                return new Point(sx, sy);
            }

            // Outline of finger thimble
            var fingerPath = new PathGeometry();
            var figure = new PathFigure { StartPoint = ToScreen(-8, 24) };
            figure.Segments.Add(new LineSegment(ToScreen(-8, 5), true));
            figure.Segments.Add(new ArcSegment(ToScreen(8, 5), new Size(8 * scale, 8 * scale), 0, false, SweepDirection.Counterclockwise, true));
            figure.Segments.Add(new LineSegment(ToScreen(8, 24), true));
            fingerPath.Figures.Add(figure);

            var fingerShape = new Path
            {
                Data = fingerPath,
                Stroke = new SolidColorBrush(Color.FromArgb(50, 0, 0, 0)),
                StrokeThickness = 1.5,
                Fill = new SolidColorBrush(Color.FromArgb(10, 0, 0, 0))
            };
            CanvasValves.Children.Add(fingerShape);

            if (_snapshot?.Valves != null)
            {
                foreach (var v in _snapshot.Valves)
                {
                    var pt = ToScreen(v.X, v.Y);
                    double norm = Math.Clamp(v.Force / _fullScaleN, 0.0, 1.0);
                    var c = Theme.GetColormapColor(norm, _colormap);

                    double radius = 10.0 + norm * 14.0;

                    var circle = new Ellipse
                    {
                        Width = radius * 2,
                        Height = radius * 2,
                        Fill = new SolidColorBrush(c),
                        Stroke = new SolidColorBrush(Color.FromRgb(0x75, 0x75, 0x75)),
                        StrokeThickness = 1.2,
                        ToolTip = $"Valve {v.Name}: {v.Label}\nForce: {v.Force:F1} N\nVoltage: {v.Voltage:F2} V\nByte: {v.Raw}"
                    };
                    Canvas.SetLeft(circle, pt.X - radius);
                    Canvas.SetTop(circle, pt.Y - radius);
                    CanvasValves.Children.Add(circle);

                    var label = new TextBlock
                    {
                        Text = v.Name,
                        FontWeight = FontWeights.Bold,
                        FontSize = 12,
                        Foreground = Brushes.Black,
                        HorizontalAlignment = HorizontalAlignment.Center
                    };
                    Canvas.SetLeft(label, pt.X - 5);
                    Canvas.SetTop(label, pt.Y - 8);
                    CanvasValves.Children.Add(label);
                }
            }
        }
    }
}
