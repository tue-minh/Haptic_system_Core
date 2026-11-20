using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;
using ControlGui.Core.Haptic;
using ControlGui.Core.Motor;
using Microsoft.Win32;

namespace ControlGui.Wpf.Windows
{
    public partial class BandwidthExperimentWindow : Window
    {
        private readonly Func<MotorWorker?> _getMotor;
        private readonly Action<string> _log;
        private readonly DispatcherTimer _uiTimer;

        private BandwidthExperimentRunner? _runner;
        private MotorWorker? _ownMotor;
        private int _activeMeasureCh = 0;
        private bool _uiReady = false;
        private bool _updatingFreqs = false;

        public BandwidthExperimentWindow(Func<MotorWorker?> getMotor, Action<string>? log = null)
        {
            InitializeComponent();
            _getMotor = getMotor;
            _log = log ?? (_ => { });

            TxtDbPath.Text = System.IO.Path.Combine(
                "recordings",
                $"bandwidth_{DateTime.Now:yyyyMMdd_HHmmss}.db");

            RefreshComPorts();
            _uiReady = true;
            SyncUiModeAndPreview();

            _uiTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(33) };
            _uiTimer.Tick += OnUiTick;
            _uiTimer.Start();

            Closing += (_, _) =>
            {
                _uiTimer.Stop();
                _runner?.Stop();
                _runner?.Dispose();
                _runner = null;
                _ownMotor?.Shutdown();
                _ownMotor?.Dispose();
                _ownMotor = null;
            };
        }

        private MotorWorker? ResolveMotor()
        {
            var ext = _getMotor();
            if (ext != null && ext.Snapshot().Connected) return ext;
            return _ownMotor ?? ext;
        }

        private void RefreshComPorts()
        {
            string? prev = CboComPort.SelectedItem as string;
            CboComPort.Items.Clear();
            var ports = SerialPort.GetPortNames().OrderBy(p => p, StringComparer.OrdinalIgnoreCase).ToList();
            foreach (var p in ports) CboComPort.Items.Add(p);
            CboComPort.Items.Add("MOCK_PORT");

            if (!string.IsNullOrEmpty(prev) && CboComPort.Items.Contains(prev))
                CboComPort.SelectedItem = prev;
            else if (ports.Count > 0)
                CboComPort.SelectedIndex = 0;
            else
                CboComPort.SelectedIndex = CboComPort.Items.Count - 1;
        }

        private void OnRefreshCom(object sender, RoutedEventArgs e)
        {
            RefreshComPorts();
        }

        private void OnConnectComClick(object sender, RoutedEventArgs e)
        {
            if (_ownMotor != null)
            {
                _runner?.Stop();
                _ownMotor.Shutdown();
                _ownMotor.Dispose();
                _ownMotor = null;
                BtnConnectCom.Content = "Connect";
                TxtComStatus.Text = "Disconnected";
                TxtComStatus.Foreground = Brushes.Gray;
                return;
            }

            string? port = CboComPort.SelectedItem as string;
            if (string.IsNullOrWhiteSpace(port))
            {
                MessageBox.Show(this, "Select a COM port first.", "COM Port", MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }

            _ownMotor = new MotorWorker(port, 1_000_000, _log, pollHz: 15.0);
            _ownMotor.Start();
            BtnConnectCom.Content = "Disconnect";
            TxtComStatus.Text = $"Connecting to {port}...";
            TxtComStatus.Foreground = Brushes.DarkOrange;
        }

        private void OnBrowseDb(object sender, RoutedEventArgs e)
        {
            var dlg = new SaveFileDialog
            {
                Filter = "SQLite Database (*.db)|*.db|All files (*.*)|*.*",
                FileName = System.IO.Path.GetFileName(TxtDbPath.Text),
                InitialDirectory = System.IO.Path.GetDirectoryName(System.IO.Path.GetFullPath(TxtDbPath.Text)) ?? ""
            };
            if (dlg.ShowDialog(this) == true)
            {
                TxtDbPath.Text = dlg.FileName;
            }
        }

        private void OnStartClick(object sender, RoutedEventArgs e)
        {
            var motor = ResolveMotor();
            if (motor == null || !motor.Snapshot().Connected)
            {
                MessageBox.Show(this, "STM32 motor/USB link is not connected. Select a COM port above and click Connect.", "Not Connected",
                    MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }

            if (!TryParseConfig(out var cfg, out string err))
            {
                MessageBox.Show(this, err, "Invalid Parameters", MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }

            // If the current DB path already exists from an earlier run, create a fresh timestamped file
            if (File.Exists(System.IO.Path.GetFullPath(cfg.DbPath)))
            {
                string modeTag = cfg.SweepMode == BandwidthSweepMode.Chirp ? "chirp" : "stepped";
                string fresh = System.IO.Path.Combine(
                    System.IO.Path.GetDirectoryName(cfg.DbPath) ?? "recordings",
                    $"bandwidth_{modeTag}_{DateTime.Now:yyyyMMdd_HHmmss}.db");
                TxtDbPath.Text = fresh;
                cfg = cfg with { DbPath = fresh };
            }

            _runner?.Dispose();
            _activeMeasureCh = cfg.ChannelIndex >= 0 && cfg.ChannelIndex < 5 ? cfg.ChannelIndex : 0;

            _runner = new BandwidthExperimentRunner(motor, cfg, _log);
            _runner.Start();

            BtnStart.IsEnabled = false;
            BtnStop.IsEnabled = true;
        }

        private void OnStopClick(object sender, RoutedEventArgs e)
        {
            _runner?.Stop();
            BtnStop.IsEnabled = false;
        }

        private void OnParamChanged(object sender, SelectionChangedEventArgs e)
        {
            if (!_uiReady) return;
            SyncUiModeAndPreview();
        }

        private void OnParamTextChanged(object sender, TextChangedEventArgs e)
        {
            if (!_uiReady || _updatingFreqs) return;
            SyncUiModeAndPreview();
        }

        private static string GetSelectedTag(ComboBox cbo, string fallback)
        {
            return (cbo.SelectedItem as ComboBoxItem)?.Tag?.ToString() ?? fallback;
        }

        private void SyncUiModeAndPreview()
        {
            bool isChirp = GetSelectedTag(CboSweepMode, "stepped") == "chirp";
            bool isLogFreq = isChirp || GetSelectedTag(CboFreqMode, "log") == "log";
            bool isTotalTime = isChirp || GetSelectedTag(CboTimingMode, "total") == "total";

            CboFreqMode.IsEnabled = !isChirp;
            PanelLogFreq.IsEnabled = isLogFreq;
            TxtFreqs.IsReadOnly = isLogFreq;
            CboTimingMode.IsEnabled = !isChirp;
            TxtTotalSec.IsEnabled = isTotalTime;
            TxtChirpSettle.IsEnabled = isChirp;
            TxtCycles.IsEnabled = !isChirp;
            TxtSettle.IsEnabled = !isChirp;

            if (isLogFreq &&
                double.TryParse(TxtFMin.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double fMin) &&
                double.TryParse(TxtFMax.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double fMax) &&
                int.TryParse(TxtFPoints.Text, out int nPts) &&
                fMin > 0 && fMax > fMin && nPts >= 2)
            {
                bool desc = GetSelectedTag(CboFreqOrder, "asc") == "desc";
                var arr = BandwidthExperimentRunner.BuildLogSpacedFrequencies(fMin, fMax, nPts, desc);
                _updatingFreqs = true;
                TxtFreqs.Text = string.Join(", ", arr.Select(f => f.ToString("G3", CultureInfo.InvariantCulture)));
                _updatingFreqs = false;
            }

            if (TryParseConfig(out var cfg, out string err))
            {
                double estSec = BandwidthExperimentRunner.EstimateDurationSec(cfg);
                if (cfg.SweepMode == BandwidthSweepMode.Chirp)
                {
                    double f0 = cfg.FreqDescending ? cfg.FreqMaxHz : cfg.FreqMinHz;
                    double f1 = cfg.FreqDescending ? cfg.FreqMinHz : cfg.FreqMaxHz;
                    TxtSchedulePreview.Text =
                        $"Est: {estSec:F1} s continuous log-chirp ({f0:G3}→{f1:G3} Hz, {cfg.FreqPoints} Bode bins, {cfg.ChirpSettleSec:F1} s settle)";
                }
                else
                {
                    var sched = BandwidthExperimentRunner.BuildSteppedSchedule(cfg);
                    double minStep = sched.Min(s => s.SettleSec + s.MeasureSec);
                    double maxStep = sched.Max(s => s.SettleSec + s.MeasureSec);
                    TxtSchedulePreview.Text =
                        $"Est: {estSec:F1} s stepped-sine ({sched.Count} pts, step {minStep:F2}–{maxStep:F2} s)";
                }
                TxtSchedulePreview.Foreground = new SolidColorBrush(Color.FromRgb(21, 101, 192));
            }
            else
            {
                TxtSchedulePreview.Text = $"Invalid config: {err}";
                TxtSchedulePreview.Foreground = Brushes.Crimson;
            }
        }

        private bool TryParseConfig(out BandwidthExperimentConfig cfg, out string error)
        {
            cfg = new BandwidthExperimentConfig();
            error = "";

            int chIdx = Convert.ToInt32(((ComboBoxItem)CboChannel.SelectedItem).Tag);
            bool isChirp = GetSelectedTag(CboSweepMode, "stepped") == "chirp";
            bool isLogFreq = isChirp || GetSelectedTag(CboFreqMode, "log") == "log";
            bool isTotalTime = isChirp || GetSelectedTag(CboTimingMode, "total") == "total";
            bool desc = GetSelectedTag(CboFreqOrder, "asc") == "desc";

            if (!double.TryParse(TxtU0.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double u0) || u0 < 0 || u0 > 100)
            {
                error = "Bias u0 must be 0..100%.";
                return false;
            }
            if (!double.TryParse(TxtUa.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double ua) || ua <= 0 || ua > 50)
            {
                error = "Amplitude ua must be > 0 and <= 50%.";
                return false;
            }

            double fMin = 0.1, fMax = 10.0;
            int fPoints = 25;
            double[] freqs;
            if (isLogFreq)
            {
                if (!double.TryParse(TxtFMin.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out fMin) || fMin < 0.02)
                {
                    error = "f_min must be >= 0.02 Hz.";
                    return false;
                }
                if (!double.TryParse(TxtFMax.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out fMax) || fMax <= fMin)
                {
                    error = "f_max must be > f_min.";
                    return false;
                }
                if (!int.TryParse(TxtFPoints.Text, out fPoints) || fPoints < 2 || fPoints > 200)
                {
                    error = "Points must be 2..200.";
                    return false;
                }
                freqs = BandwidthExperimentRunner.BuildLogSpacedFrequencies(fMin, fMax, fPoints, desc);
            }
            else
            {
                freqs = TxtFreqs.Text
                    .Split(new[] { ',', ';', ' ' }, StringSplitOptions.RemoveEmptyEntries)
                    .Select(s => double.TryParse(s.Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out double v) ? v : -1)
                    .Where(v => v > 0)
                    .ToArray();
                if (freqs.Length == 0)
                {
                    error = "Provide at least one positive frequency in Hz.";
                    return false;
                }
                fMin = freqs.Min();
                fMax = freqs.Max();
                fPoints = freqs.Length;
            }

            if (!double.TryParse(TxtTotalSec.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double totalSec) || totalSec < 5)
            {
                error = "Total Duration must be >= 5 s.";
                return false;
            }
            if (!double.TryParse(TxtChirpSettle.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double chirpSettle) || chirpSettle < 0)
            {
                error = "Chirp Settle must be >= 0 s.";
                return false;
            }
            if (!int.TryParse(TxtCycles.Text, out int cycles) || cycles < 1)
            {
                error = "Cycles / Freq must be >= 1.";
                return false;
            }
            if (!int.TryParse(TxtSettle.Text, out int settle) || settle < 0)
            {
                error = "Settle Cycles must be >= 0.";
                return false;
            }
            if (!double.TryParse(TxtRateHz.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double rateHz) || rateHz < 10 || rateHz > 250)
            {
                error = "Sample Rate must be 10..250 Hz.";
                return false;
            }
            if (!byte.TryParse(TxtSpiPeriod.Text, out byte spiPeriod) || spiPeriod < 1)
            {
                error = "SPI Poll Period must be >= 1 ms.";
                return false;
            }
            if (!double.TryParse(TxtVref.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double vref) || vref <= 0)
            {
                error = "ADC Vref must be > 0 mV.";
                return false;
            }

            byte busId = Convert.ToByte(((ComboBoxItem)CboSpiBus.SelectedItem).Tag);
            byte csIdx = Convert.ToByte(((ComboBoxItem)CboSpiCs.SelectedItem).Tag);

            string dbPath = TxtDbPath.Text.Trim();
            if (string.IsNullOrEmpty(dbPath))
            {
                dbPath = System.IO.Path.Combine("recordings", $"bandwidth_{DateTime.Now:yyyyMMdd_HHmmss}.db");
                TxtDbPath.Text = dbPath;
            }

            cfg = new BandwidthExperimentConfig
            {
                ChannelIndex = chIdx,
                SweepMode = isChirp ? BandwidthSweepMode.Chirp : BandwidthSweepMode.SteppedSine,
                TimingMode = isTotalTime ? SteppedTimingMode.TotalDurationSec : SteppedTimingMode.FixedCycles,
                TotalDurationSec = totalSec,
                FreqMinHz = fMin,
                FreqMaxHz = fMax,
                FreqPoints = fPoints,
                FreqDescending = desc,
                ChirpSettleSec = chirpSettle,
                U0Percent = u0,
                UaPercent = ua,
                FrequenciesHz = freqs,
                CyclesPerFreq = cycles,
                SettleCycles = settle,
                SampleRateHz = rateHz,
                SpiBusId = busId,
                SpiCsIdx = csIdx,
                SpiPeriodMs = spiPeriod,
                VrefMv = vref,
                ZeroInactiveChannels = ChkZeroOthers.IsChecked == true,
                DbPath = dbPath
            };
            return true;
        }

        private void OnUiTick(object? sender, EventArgs e)
        {
            var ext = _getMotor();
            if (ext != null && ext.Snapshot().Connected)
            {
                TxtComStatus.Text = $"Connected via MainWindow ({ext.Port})";
                TxtComStatus.Foreground = Brushes.ForestGreen;
                BtnConnectCom.IsEnabled = false;
            }
            else
            {
                BtnConnectCom.IsEnabled = true;
                if (_ownMotor != null)
                {
                    var ms = _ownMotor.Snapshot();
                    if (ms.Connected)
                    {
                        TxtComStatus.Text = $"Connected ({_ownMotor.Port}, {ms.Firmware})";
                        TxtComStatus.Foreground = Brushes.ForestGreen;
                    }
                    else if (!string.IsNullOrEmpty(ms.Error))
                    {
                        TxtComStatus.Text = $"Error: {ms.Error}";
                        TxtComStatus.Foreground = Brushes.Crimson;
                    }
                    else
                    {
                        TxtComStatus.Text = $"Connecting to {_ownMotor.Port}...";
                        TxtComStatus.Foreground = Brushes.DarkOrange;
                    }
                }
                else
                {
                    TxtComStatus.Text = "Not connected — select COM port and click Connect";
                    TxtComStatus.Foreground = Brushes.Gray;
                }
            }

            if (_runner == null) return;

            var st = _runner.GetStatus();
            ProgSweep.Value = st.Progress01;
            TxtStatus.Text =
                $"State: {st.State} | Step: {st.CurrentStep}/{st.TotalSteps} ({st.CurrentFreqHz:F2} Hz)\n" +
                $"Samples: {st.SampleCount} | ReadErr: {st.ReadErrors} | Rate: {st.ActualRateHz:F1} Hz" +
                (string.IsNullOrEmpty(st.Error) ? "" : $"\nError: {st.Error}");

            TxtBandwidth.Text = st.BandwidthFcHz.HasValue
                ? $"Bandwidth (-3 dB): {st.BandwidthFcHz.Value:F2} Hz"
                : "Bandwidth (-3 dB): --- Hz";

            if (!st.Running)
            {
                BtnStart.IsEnabled = true;
                BtnStop.IsEnabled = false;
            }

            var samples = _runner.GetLiveSamples(500);
            DrawTimePlot(samples);

            var bode = _runner.GetBodePoints();
            GridBode.ItemsSource = bode;
            DrawBodePlot(bode, st.BandwidthFcHz);
        }

        private void OnCanvasSizeChanged(object sender, SizeChangedEventArgs e)
        {
            if (_runner == null) return;
            DrawTimePlot(_runner.GetLiveSamples(500));
            var st = _runner.GetStatus();
            DrawBodePlot(_runner.GetBodePoints(), st.BandwidthFcHz);
        }

        private void DrawTimePlot(List<BandwidthSample> samples)
        {
            CanvasTime.Children.Clear();
            double w = CanvasTime.ActualWidth;
            double h = CanvasTime.ActualHeight;
            if (w < 40 || h < 40 || samples.Count < 2) return;

            const double padL = 44, padR = 44, padT = 12, padB = 22;
            double pw = w - padL - padR;
            double ph = h - padT - padB;
            if (pw <= 10 || ph <= 10) return;

            double tMin = samples[0].TimeSec;
            double tMax = Math.Max(tMin + 0.1, samples[^1].TimeSec);
            double mvMin = 0.0;
            double mvMax = 3300.0;

            // Grid lines
            for (int i = 0; i <= 4; i++)
            {
                double y = padT + ph * i / 4.0;
                CanvasTime.Children.Add(new Line
                {
                    X1 = padL, Y1 = y, X2 = padL + pw, Y2 = y,
                    Stroke = new SolidColorBrush(Color.FromRgb(235, 235, 235)),
                    StrokeThickness = 1
                });
            }

            var cmdLine = new Polyline { Stroke = Brushes.RoyalBlue, StrokeThickness = 1.6 };
            var adcLine = new Polyline { Stroke = Brushes.Crimson, StrokeThickness = 1.6 };

            foreach (var s in samples)
            {
                double x = padL + (s.TimeSec - tMin) / (tMax - tMin) * pw;
                double yCmd = padT + (1.0 - Math.Clamp(s.UPct / 100.0, 0.0, 1.0)) * ph;
                double mv = _activeMeasureCh < s.Mv.Length ? s.Mv[_activeMeasureCh] : 0.0;
                double yAdc = padT + (1.0 - Math.Clamp((mv - mvMin) / (mvMax - mvMin), 0.0, 1.0)) * ph;
                cmdLine.Points.Add(new Point(x, yCmd));
                adcLine.Points.Add(new Point(x, yAdc));
            }

            CanvasTime.Children.Add(cmdLine);
            CanvasTime.Children.Add(adcLine);

            AddLabel(CanvasTime, "100%", 4, padT - 4, Brushes.RoyalBlue);
            AddLabel(CanvasTime, "0%", 4, padT + ph - 10, Brushes.RoyalBlue);
            AddLabel(CanvasTime, "3300 mV", padL + pw + 4, padT - 4, Brushes.Crimson);
            AddLabel(CanvasTime, "0 mV", padL + pw + 4, padT + ph - 10, Brushes.Crimson);
        }

        private void DrawBodePlot(List<BodePoint> points, double? fcHz)
        {
            CanvasBode.Children.Clear();
            double w = CanvasBode.ActualWidth;
            double h = CanvasBode.ActualHeight;
            if (w < 40 || h < 40) return;

            const double padL = 44, padR = 20, padT = 12, padB = 24;
            double pw = w - padL - padR;
            double ph = h - padT - padB;
            if (pw <= 10 || ph <= 10) return;

            double dbMin = -18.0, dbMax = 3.0;
            double fMinLog = Math.Log10(0.1);
            double fMaxLog = Math.Log10(12.0);

            if (points.Count > 0)
            {
                fMinLog = Math.Log10(Math.Max(0.05, points.Min(p => p.FreqHz) * 0.8));
                fMaxLog = Math.Log10(Math.Max(1.0, points.Max(p => p.FreqHz) * 1.2));
                dbMin = Math.Min(-12.0, Math.Floor(points.Min(p => p.MagDb) / 3.0) * 3.0 - 3.0);
                dbMax = Math.Max(3.0, Math.Ceiling(points.Max(p => p.MagDb) / 3.0) * 3.0);
            }

            // -3 dB dashed line
            double yMinus3 = padT + (1.0 - (-3.0 - dbMin) / (dbMax - dbMin)) * ph;
            CanvasBode.Children.Add(new Line
            {
                X1 = padL, Y1 = yMinus3, X2 = padL + pw, Y2 = yMinus3,
                Stroke = Brushes.DarkOrange,
                StrokeThickness = 1.4,
                StrokeDashArray = new DoubleCollection { 4, 3 }
            });
            AddLabel(CanvasBode, "-3 dB", 4, yMinus3 - 7, Brushes.DarkOrange);
            AddLabel(CanvasBode, $"{dbMax:F0} dB", 4, padT - 4, Brushes.Gray);
            AddLabel(CanvasBode, $"{dbMin:F0} dB", 4, padT + ph - 10, Brushes.Gray);

            if (points.Count == 0) return;

            // Log-spaced canonical grid ticks
            double[] tickFreqs = { 0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0 };
            foreach (double tf in tickFreqs)
            {
                double ltf = Math.Log10(tf);
                if (ltf < fMinLog || ltf > fMaxLog) continue;
                double gx = padL + (ltf - fMinLog) / (fMaxLog - fMinLog) * pw;
                CanvasBode.Children.Add(new Line
                {
                    X1 = gx, Y1 = padT, X2 = gx, Y2 = padT + ph,
                    Stroke = new SolidColorBrush(Color.FromRgb(238, 238, 238)),
                    StrokeThickness = 1
                });
                AddLabel(CanvasBode, $"{tf:G3}", gx - 8, padT + ph + 2, Brushes.DimGray);
            }

            var sorted = points.OrderBy(p => p.FreqHz).ToList();
            var line = new Polyline { Stroke = Brushes.Teal, StrokeThickness = 2.0 };
            double dotSize = sorted.Count > 16 ? 5.0 : 7.0;
            double dotHalf = dotSize * 0.5;
            foreach (var p in sorted)
            {
                double x = padL + (Math.Log10(p.FreqHz) - fMinLog) / (fMaxLog - fMinLog) * pw;
                double y = padT + (1.0 - Math.Clamp((p.MagDb - dbMin) / (dbMax - dbMin), 0.0, 1.0)) * ph;
                line.Points.Add(new Point(x, y));

                var dot = new Ellipse { Width = dotSize, Height = dotSize, Fill = Brushes.Teal };
                Canvas.SetLeft(dot, x - dotHalf);
                Canvas.SetTop(dot, y - dotHalf);
                CanvasBode.Children.Add(dot);
            }
            CanvasBode.Children.Add(line);

            if (fcHz.HasValue && fcHz.Value > 0)
            {
                double xFc = padL + (Math.Log10(fcHz.Value) - fMinLog) / (fMaxLog - fMinLog) * pw;
                CanvasBode.Children.Add(new Line
                {
                    X1 = xFc, Y1 = padT, X2 = xFc, Y2 = padT + ph,
                    Stroke = Brushes.Crimson,
                    StrokeThickness = 1.4,
                    StrokeDashArray = new DoubleCollection { 3, 3 }
                });
                AddLabel(CanvasBode, $"fc={fcHz.Value:F2}Hz", xFc + 4, padT + 4, Brushes.Crimson);
            }
        }

        private static void AddLabel(Canvas canvas, string text, double x, double y, Brush color)
        {
            var tb = new TextBlock
            {
                Text = text,
                FontSize = 10,
                FontFamily = new FontFamily("Consolas"),
                Foreground = color
            };
            Canvas.SetLeft(tb, x);
            Canvas.SetTop(tb, y);
            canvas.Children.Add(tb);
        }
    }
}
