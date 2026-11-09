using System;
using System.Globalization;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;
using ControlGui.Core.Paxini;

namespace ControlGui.Wpf.Controls
{
    public partial class Tactile3DView : UserControl
    {
        private double _fullScaleN = 15.0;
        private string _colormap = "Greys";

        // View angles in degrees
        private double _azimuthDeg = 35.0;
        private double _elevationDeg = 25.0;

        private Point _lastMousePos;
        private bool _isDragging = false;

        private SensorReading? _lastLeft;
        private SensorReading? _lastRight;

        public Tactile3DView()
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

        private void OnCanvasMouseDown(object sender, MouseButtonEventArgs e)
        {
            if (e.ChangedButton == MouseButton.Left)
            {
                _isDragging = true;
                _lastMousePos = e.GetPosition(this);
                ((UIElement)sender).CaptureMouse();
            }
        }

        private void OnCanvasMouseMove(object sender, MouseEventArgs e)
        {
            if (_isDragging)
            {
                var cur = e.GetPosition(this);
                double dx = cur.X - _lastMousePos.X;
                double dy = cur.Y - _lastMousePos.Y;
                _lastMousePos = cur;

                _azimuthDeg = (_azimuthDeg + dx * 0.5) % 360.0;
                _elevationDeg = Math.Clamp(_elevationDeg - dy * 0.5, -85.0, 85.0);

                Redraw();
            }
        }

        private void OnCanvasMouseUp(object sender, MouseButtonEventArgs e)
        {
            if (_isDragging)
            {
                _isDragging = false;
                ((UIElement)sender).ReleaseMouseCapture();
            }
        }

        public void Update(SensorReading? left, SensorReading? right)
        {
            _lastLeft = left;
            _lastRight = right;
            Redraw();
        }

        private void Redraw()
        {
            DrawFinger3D(CanvasLeft, _lastLeft, "Left");
            DrawFinger3D(CanvasRight, _lastRight, "Right");
        }

        private void DrawFinger3D(Canvas canvas, SensorReading? reading, string fingerTag)
        {
            if (canvas == null || canvas.ActualWidth <= 10 || canvas.ActualHeight <= 10) return;

            canvas.Children.Clear();

            double w = canvas.ActualWidth;
            double h = canvas.ActualHeight;

            double azRad = _azimuthDeg * Math.PI / 180.0;
            double elRad = _elevationDeg * Math.PI / 180.0;

            double cosAz = Math.Cos(azRad);
            double sinAz = Math.Sin(azRad);
            double cosEl = Math.Cos(elRad);
            double sinEl = Math.Sin(elRad);

            // Center of sensor geometry is approx X=0, Y=10, Z=4
            double ox = 0.0, oy = 9.5, oz = 4.0;
            double scale = Math.Min(w, h) / 36.0;

            Point Project(double x, double y, double z)
            {
                double dx = x - ox;
                double dy = y - oy;
                double dz = z - oz;

                // Azimuth rotation around Z
                double rx = dx * cosAz - dy * sinAz;
                double ry = dx * sinAz + dy * cosAz;
                double rz = dz;

                // Elevation rotation around X
                double px = rx;
                double py = ry * sinEl + rz * cosEl;

                return new Point(w / 2.0 + px * scale, h / 2.0 - py * scale);
            }

            // Draw coordinate axes
            var origin2D = Project(0, 0, 0);
            var xEnd = Project(5, 0, 0);
            var yEnd = Project(0, 5, 0);
            var zEnd = Project(0, 0, 5);

            canvas.Children.Add(new Line { X1 = origin2D.X, Y1 = origin2D.Y, X2 = xEnd.X, Y2 = xEnd.Y, Stroke = Brushes.LightCoral, StrokeThickness = 1.2 });
            canvas.Children.Add(new Line { X1 = origin2D.X, Y1 = origin2D.Y, X2 = yEnd.X, Y2 = yEnd.Y, Stroke = Brushes.LightGreen, StrokeThickness = 1.2 });
            canvas.Children.Add(new Line { X1 = origin2D.X, Y1 = origin2D.Y, X2 = zEnd.X, Y2 = zEnd.Y, Stroke = Brushes.LightBlue, StrokeThickness = 1.2 });

            var coords = S2015EliteGeometry.Coordinates;
            var taxels = reading?.Taxels;

            // Draw 52 taxel spheres
            for (int i = 0; i < coords.Length; i++)
            {
                double fz = (taxels != null && taxels.GetLength(0) == coords.Length) ? taxels[i, 2] : 0.0;
                double norm = Math.Clamp(fz / _fullScaleN, 0.0, 1.0);
                var c = Theme.GetColormapColor(norm, _colormap);

                double radius = 3.5 + norm * 5.0;
                var pt = Project(coords[i].X, coords[i].Y, coords[i].Z);

                var el = new Ellipse
                {
                    Width = radius * 2,
                    Height = radius * 2,
                    Fill = new SolidColorBrush(c),
                    Stroke = new SolidColorBrush(Color.FromRgb(0x9E, 0x9E, 0x9E)),
                    StrokeThickness = 0.6,
                    ToolTip = $"Taxel #{i + 1}\nFz: {fz:F2} N"
                };
                Canvas.SetLeft(el, pt.X - radius);
                Canvas.SetTop(el, pt.Y - radius);
                canvas.Children.Add(el);
            }

            // 3D Resultant Force Vector Quiver
            if (reading != null && reading.FMag > 0.1)
            {
                double copX = double.IsNaN(reading.CopX) ? 0.0 : reading.CopX;
                double copY = double.IsNaN(reading.CopY) ? 9.5 : reading.CopY;
                double copZ = 4.5;

                var cop2D = Project(copX, copY, copZ);

                // Force vector direction
                double arrowScale = 2.5;
                double endX = copX + reading.Fx * arrowScale;
                double endY = copY + reading.Fy * arrowScale;
                double endZ = copZ + reading.Fz * arrowScale;

                var arrowEnd2D = Project(endX, endY, endZ);

                var quiver = new Line
                {
                    X1 = cop2D.X,
                    Y1 = cop2D.Y,
                    X2 = arrowEnd2D.X,
                    Y2 = arrowEnd2D.Y,
                    Stroke = Brushes.DarkOrange,
                    StrokeThickness = 2.8
                };
                canvas.Children.Add(quiver);

                var copMarker = new Ellipse
                {
                    Width = 8,
                    Height = 8,
                    Fill = Brushes.Red,
                    ToolTip = $"Center of Pressure\nFx: {reading.Fx:F2} N\nFy: {reading.Fy:F2} N\nFz: {reading.Fz:F2} N"
                };
                Canvas.SetLeft(copMarker, cop2D.X - 4);
                Canvas.SetTop(copMarker, cop2D.Y - 4);
                canvas.Children.Add(copMarker);
            }
        }
    }
}
