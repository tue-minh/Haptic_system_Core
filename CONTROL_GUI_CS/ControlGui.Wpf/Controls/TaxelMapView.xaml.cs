using System;
using System.Globalization;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using ControlGui.Core.Paxini;

namespace ControlGui.Wpf.Controls
{
    public partial class TaxelMapView : UserControl
    {
        private double _fullScaleN = 15.0;
        private double _arrowScale = 6.0;
        private string _colormap = "Greys";

        private SensorReading? _lastLeft;
        private SensorReading? _lastRight;

        public TaxelMapView()
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

        private void OnArrowScaleChanged(object sender, TextChangedEventArgs e)
        {
            if (double.TryParse(TxtArrowScale?.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out var v) && v > 0)
            {
                _arrowScale = v;
                Redraw();
            }
        }

        private void OnCanvasSizeChanged(object sender, SizeChangedEventArgs e)
        {
            Redraw();
        }

        public void Update(SensorReading? left, SensorReading? right)
        {
            _lastLeft = left;
            _lastRight = right;
            Redraw();
        }

        private void Redraw()
        {
            DrawFinger(CanvasLeft, _lastLeft);
            DrawFinger(CanvasRight, _lastRight);
        }

        private void DrawFinger(Canvas canvas, SensorReading? reading)
        {
            if (canvas == null || canvas.ActualWidth <= 10 || canvas.ActualHeight <= 10) return;

            canvas.Children.Clear();

            double w = canvas.ActualWidth;
            double h = canvas.ActualHeight;

            // Geometry range: X from -8 to +8 mm, Y from -3 to +22 mm
            double minX = -10.0, maxX = 10.0;
            double minY = -3.0, maxY = 22.0;

            double scaleX = w / (maxX - minX);
            double scaleY = h / (maxY - minY);
            double scale = Math.Min(scaleX, scaleY) * 0.90;

            double cx = w / 2.0;
            double cy = h / 2.0;
            double midY = (minY + maxY) / 2.0;

            Point ToScreen(double mx, double my)
            {
                double sx = cx + mx * scale;
                double sy = cy - (my - midY) * scale;
                return new Point(sx, sy);
            }

            // Draw grid lines
            var gridPen = new SolidColorBrush(Color.FromArgb(40, 0, 0, 0));
            var axisPen = new SolidColorBrush(Color.FromArgb(90, 0, 0, 0));

            var horizAxis = new Line { X1 = 0, Y1 = ToScreen(0, 0).Y, X2 = w, Y2 = ToScreen(0, 0).Y, Stroke = axisPen, StrokeThickness = 1 };
            var vertAxis = new Line { X1 = cx, Y1 = 0, X2 = cx, Y2 = h, Stroke = axisPen, StrokeThickness = 1 };
            canvas.Children.Add(horizAxis);
            canvas.Children.Add(vertAxis);

            var taxels = reading?.Taxels;
            var coords = S2015EliteGeometry.Coordinates;

            double taxelCircleRadius = 7.0;

            for (int i = 0; i < coords.Length; i++)
            {
                double fz = (taxels != null && taxels.GetLength(0) == coords.Length) ? taxels[i, 2] : 0.0;
                double norm = Math.Clamp(fz / _fullScaleN, 0.0, 1.0);
                var c = Theme.GetColormapColor(norm, _colormap);

                var pt = ToScreen(coords[i].X, coords[i].Y);

                var el = new Ellipse
                {
                    Width = taxelCircleRadius * 2,
                    Height = taxelCircleRadius * 2,
                    Fill = new SolidColorBrush(c),
                    Stroke = new SolidColorBrush(Color.FromRgb(0xBD, 0xBD, 0xBD)),
                    StrokeThickness = 0.8,
                    ToolTip = $"Taxel #{i + 1}\nForce: {fz:F2} N\nX: {coords[i].X:F2} mm\nY: {coords[i].Y:F2} mm"
                };

                Canvas.SetLeft(el, pt.X - taxelCircleRadius);
                Canvas.SetTop(el, pt.Y - taxelCircleRadius);
                canvas.Children.Add(el);
            }

            // Draw Center of Pressure marker (+)
            if (reading != null && !double.IsNaN(reading.CopX) && !double.IsNaN(reading.CopY))
            {
                var copPt = ToScreen(reading.CopX, reading.CopY);
                double s = 8.0;

                var l1 = new Line { X1 = copPt.X - s, Y1 = copPt.Y, X2 = copPt.X + s, Y2 = copPt.Y, Stroke = Brushes.Red, StrokeThickness = 2.5 };
                var l2 = new Line { X1 = copPt.X, Y1 = copPt.Y - s, X2 = copPt.X, Y2 = copPt.Y + s, Stroke = Brushes.Red, StrokeThickness = 2.5 };
                canvas.Children.Add(l1);
                canvas.Children.Add(l2);

                // Draw tangential shear arrow
                double arrowDx = reading.Fx * _arrowScale;
                double arrowDy = -reading.Fy * _arrowScale; // Y inverted on screen

                var arrowLine = new Line
                {
                    X1 = copPt.X,
                    Y1 = copPt.Y,
                    X2 = copPt.X + arrowDx,
                    Y2 = copPt.Y + arrowDy,
                    Stroke = Brushes.DarkBlue,
                    StrokeThickness = 2.2
                };
                canvas.Children.Add(arrowLine);

                var tip = new Ellipse
                {
                    Width = 6,
                    Height = 6,
                    Fill = Brushes.DarkBlue
                };
                Canvas.SetLeft(tip, copPt.X + arrowDx - 3);
                Canvas.SetTop(tip, copPt.Y + arrowDy - 3);
                canvas.Children.Add(tip);
            }
        }
    }
}
