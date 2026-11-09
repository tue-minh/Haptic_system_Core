using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;

namespace ControlGui.Wpf.Controls
{
    public class PlotDataPoint
    {
        public double T { get; set; }
        public double Fx { get; set; }
        public double Fy { get; set; }
        public double Fz { get; set; }
        public double FMag { get; set; }
        public double FShear { get; set; }
    }

    public partial class ForceHistoryPlot : UserControl
    {
        private readonly List<PlotDataPoint> _leftHistory = new();
        private readonly List<PlotDataPoint> _rightHistory = new();

        public double TimeWindowSeconds { get; set; } = 10.0;
        public string DirectionFilter { get; set; } = "Fz"; // Fz, Fx, Fy, All
        public bool ShowMag { get; set; } = false;
        public bool ShowShear { get; set; } = false;

        public ForceHistoryPlot()
        {
            InitializeComponent();
        }

        private void OnCanvasSizeChanged(object sender, SizeChangedEventArgs e)
        {
            Redraw();
        }

        public void PushPoints(PlotDataPoint? left, PlotDataPoint? right, double currentT)
        {
            double cutoff = currentT - TimeWindowSeconds;

            if (left != null)
            {
                _leftHistory.Add(left);
                _leftHistory.RemoveAll(p => p.T < cutoff);
            }

            if (right != null)
            {
                _rightHistory.Add(right);
                _rightHistory.RemoveAll(p => p.T < cutoff);
            }

            Redraw();
        }

        public void Clear()
        {
            _leftHistory.Clear();
            _rightHistory.Clear();
            Redraw();
        }

        private void Redraw()
        {
            if (PlotCanvas == null || PlotCanvas.ActualWidth <= 10 || PlotCanvas.ActualHeight <= 10) return;

            PlotCanvas.Children.Clear();
            double w = PlotCanvas.ActualWidth;
            double h = PlotCanvas.ActualHeight;

            // Background grid
            var gridPen = new SolidColorBrush(Color.FromArgb(30, 0, 0, 0));
            for (double y = 0; y <= h; y += h / 5.0)
            {
                PlotCanvas.Children.Add(new Line { X1 = 0, Y1 = y, X2 = w, Y2 = y, Stroke = gridPen, StrokeThickness = 1 });
            }
            for (double x = 0; x <= w; x += w / 8.0)
            {
                PlotCanvas.Children.Add(new Line { X1 = x, Y1 = 0, X2 = x, Y2 = h, Stroke = gridPen, StrokeThickness = 1 });
            }

            // Find current time range
            double maxT = 0;
            if (_leftHistory.Count > 0) maxT = Math.Max(maxT, _leftHistory[^1].T);
            if (_rightHistory.Count > 0) maxT = Math.Max(maxT, _rightHistory[^1].T);
            double minT = maxT - TimeWindowSeconds;

            // Find force range (min -2N, max at least 10N)
            double maxF = 10.0;
            double minF = -2.0;

            void CheckMinMax(List<PlotDataPoint> list)
            {
                foreach (var p in list)
                {
                    maxF = Math.Max(maxF, Math.Max(p.Fz, Math.Max(p.Fx, p.Fy)));
                    minF = Math.Min(minF, Math.Min(p.Fx, p.Fy));
                    if (ShowMag) maxF = Math.Max(maxF, p.FMag);
                    if (ShowShear) maxF = Math.Max(maxF, p.FShear);
                }
            }
            CheckMinMax(_leftHistory);
            CheckMinMax(_rightHistory);
            maxF = Math.Ceiling(maxF * 1.15);

            Point MapPoint(double t, double f)
            {
                double sx = w * ((t - minT) / TimeWindowSeconds);
                double sy = h - (h * ((f - minF) / (maxF - minF)));
                return new Point(sx, sy);
            }

            // Zero line
            double yZero = MapPoint(0, 0).Y;
            PlotCanvas.Children.Add(new Line
            {
                X1 = 0, Y1 = yZero, X2 = w, Y2 = yZero,
                Stroke = new SolidColorBrush(Color.FromArgb(90, 0, 0, 0)),
                StrokeThickness = 1.2
            });

            // Draw Left sensor lines (Dark Grey / Black)
            DrawCurve(_leftHistory, MapPoint, Color.FromRgb(0x21, 0x21, 0x21), isLeft: true);

            // Draw Right sensor lines (Light Grey / Dim)
            DrawCurve(_rightHistory, MapPoint, Color.FromRgb(0x9E, 0x9E, 0x9E), isLeft: false);
        }

        private void DrawCurve(List<PlotDataPoint> history, Func<double, double, Point> mapPoint, Color baseColor, bool isLeft)
        {
            if (history.Count < 2) return;

            var brush = new SolidColorBrush(baseColor);

            void DrawPoly(Func<PlotDataPoint, double> selector, DoubleCollection? dashArray)
            {
                var figure = new PathFigure { StartPoint = mapPoint(history[0].T, selector(history[0])) };
                for (int i = 1; i < history.Count; i++)
                {
                    figure.Segments.Add(new LineSegment(mapPoint(history[i].T, selector(history[i])), true));
                }

                var geom = new PathGeometry();
                geom.Figures.Add(figure);

                var path = new Path
                {
                    Data = geom,
                    Stroke = brush,
                    StrokeThickness = isLeft ? 1.8 : 1.4,
                    StrokeDashArray = dashArray
                };
                PlotCanvas.Children.Add(path);
            }

            if (DirectionFilter == "Fz" || DirectionFilter == "All")
            {
                // Solid line for Fz
                DrawPoly(p => p.Fz, null);
            }

            if (DirectionFilter == "Fx" || DirectionFilter == "All")
            {
                // Dashed line for Fx
                DrawPoly(p => p.Fx, new DoubleCollection { 4, 2 });
            }

            if (DirectionFilter == "Fy" || DirectionFilter == "All")
            {
                // Dotted line for Fy
                DrawPoly(p => p.Fy, new DoubleCollection { 1, 2 });
            }

            if (ShowMag)
            {
                DrawPoly(p => p.FMag, new DoubleCollection { 6, 3, 2, 3 });
            }

            if (ShowShear)
            {
                DrawPoly(p => p.FShear, new DoubleCollection { 2, 4 });
            }
        }
    }
}
