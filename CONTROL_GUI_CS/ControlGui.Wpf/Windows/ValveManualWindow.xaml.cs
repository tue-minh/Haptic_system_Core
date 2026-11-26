using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO.Ports;
using System.Linq;
using System.Runtime.InteropServices;
using System.Threading;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;
using ControlGui.Core.Haptic;
using ControlGui.Core.Motor;

namespace ControlGui.Wpf.Windows
{
    public partial class ValveManualWindow : Window
    {
        [DllImport("winmm.dll", EntryPoint = "timeBeginPeriod")]
        private static extern uint TimeBeginPeriod(uint uMilliseconds);

        [DllImport("winmm.dll", EntryPoint = "timeEndPeriod")]
        private static extern uint TimeEndPeriod(uint uMilliseconds);

        private sealed class ValveRowUi
        {
            public required int Index;
            public required Slider Slider;
            public required TextBox TxtPct;
            public required TextBlock TxtMeta;
            public bool Updating;
        }

        private readonly record struct AdcPlotSample(double TimeSec, double[] Mv);

        private static readonly (string Name, string Desc)[] ValveDefs =
        {
            ("A", "Valve A (Ch 0 · Tip +Y)"),
            ("B", "Valve B (Ch 1 · Mid Fz)"),
            ("C", "Valve C (Ch 2 · Base -Y)"),
            ("D", "Valve D (Ch 3 · Left +X)"),
            ("E", "Valve E (Ch 4 · Right -X)")
        };

        private static readonly Brush[] AdcBrushes =
        {
            new SolidColorBrush(Color.FromRgb(0xD3, 0x2F, 0x2F)), // CH0
            new SolidColorBrush(Color.FromRgb(0xE6, 0x51, 0x00)), // CH1
            new SolidColorBrush(Color.FromRgb(0x2E, 0x7D, 0x32)), // CH2
            new SolidColorBrush(Color.FromRgb(0x15, 0x65, 0xC0)), // CH3
            new SolidColorBrush(Color.FromRgb(0x6A, 0x1B, 0x9A)), // CH4
        };

        private readonly ValveRowUi[] _valveRows = new ValveRowUi[5];
        private readonly double[] _pctTargets = new double[5];
        private readonly object _targetLock = new();
        private readonly object _statsLock = new();
        private readonly TueBinaryEncoder _encoder = new();

        private readonly List<AdcPlotSample> _adcHistory = new(450);
        private readonly Stopwatch _plotSw = Stopwatch.StartNew();
        private readonly DispatcherTimer _uiTimer;

        private MotorWorker? _motor;
        private Thread? _txThread;
        private CancellationTokenSource? _txCts;

        private long _txSent = 0;
        private long _txErrors = 0;
        private double _txRateHz = 0.0;
        private byte[] _lastWireFrame = Array.Empty<byte>();
        private uint _lastAdcSeqSeen = uint.MaxValue;

        public ValveManualWindow()
        {
            InitializeComponent();
            BuildValveControls();
            RefreshComPorts();

            PanelAdc.ApplyRequested += (enable, busId, csIdx, periodMs) =>
            {
                _motor?.ConfigureSpiAdc(enable, busId, csIdx, periodMs);
            };
            PanelAdc.ReadRequested += () =>
            {
                _motor?.ReadSpiAdc();
            };

            PanelMapping.ApplyRequested += (vMin, vMax, fThresh, fRated, saveEeprom) =>
            {
                _motor?.SetHapticMapping(vMin, vMax, fThresh, fRated, saveEeprom);
            };
            PanelMapping.ReadRequested += () =>
            {
                _motor?.ReadHapticMapping();
            };
            PanelMapping.MappingChanged += _ => RefreshValveMetas();

            _uiTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(33) };
            _uiTimer.Tick += OnUiTick;
            _uiTimer.Start();

            Closing += (_, _) =>
            {
                _uiTimer.Stop();
                StopStreamerAndVent();
                DisconnectMotor();
            };
        }

        private void BuildValveControls()
        {
            for (int i = 0; i < 5; i++)
            {
                int idx = i;
                var border = new Border
                {
                    Background = Brushes.White,
                    BorderBrush = new SolidColorBrush(Color.FromRgb(0xE0, 0xE0, 0xE0)),
                    BorderThickness = new Thickness(1),
                    CornerRadius = new CornerRadius(4),
                    Padding = new Thickness(8, 6, 8, 6),
                    Margin = new Thickness(0, 0, 0, 6)
                };

                var stack = new StackPanel();

                var headerGrid = new Grid();
                headerGrid.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(1, GridUnitType.Star) });
                headerGrid.ColumnDefinitions.Add(new ColumnDefinition { Width = GridLength.Auto });

                var lblName = new TextBlock
                {
                    Text = ValveDefs[idx].Desc,
                    FontWeight = FontWeights.SemiBold,
                    Foreground = AdcBrushes[idx],
                    VerticalAlignment = VerticalAlignment.Center
                };
                var lblMeta = new TextBlock
                {
                    Text = "Byte:   0 | F:  0.0 N | DAC: 0.00 V",
                    FontFamily = new FontFamily("Consolas"),
                    FontSize = 11,
                    Foreground = new SolidColorBrush(Color.FromRgb(0x42, 0x42, 0x42)),
                    VerticalAlignment = VerticalAlignment.Center
                };
                Grid.SetColumn(lblName, 0);
                Grid.SetColumn(lblMeta, 1);
                headerGrid.Children.Add(lblName);
                headerGrid.Children.Add(lblMeta);
                stack.Children.Add(headerGrid);

                var ctrlGrid = new Grid { Margin = new Thickness(0, 4, 0, 0) };
                ctrlGrid.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(1, GridUnitType.Star) });
                ctrlGrid.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(54) });
                ctrlGrid.ColumnDefinitions.Add(new ColumnDefinition { Width = GridLength.Auto });
                ctrlGrid.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(42) });

                var slider = new Slider
                {
                    Minimum = 0,
                    Maximum = 100,
                    Value = 0,
                    TickFrequency = 1,
                    IsSnapToTickEnabled = false,
                    VerticalAlignment = VerticalAlignment.Center,
                    Margin = new Thickness(0, 0, 8, 0)
                };
                var txtPct = new TextBox
                {
                    Text = "0.0",
                    VerticalContentAlignment = VerticalAlignment.Center,
                    HorizontalContentAlignment = HorizontalAlignment.Right,
                    FontFamily = new FontFamily("Consolas"),
                    Margin = new Thickness(0, 0, 4, 0)
                };
                var lblPctUnit = new TextBlock
                {
                    Text = "%",
                    VerticalAlignment = VerticalAlignment.Center,
                    Foreground = Brushes.Gray,
                    Margin = new Thickness(0, 0, 8, 0)
                };
                var btnZero = new Button
                {
                    Content = "0%",
                    Padding = new Thickness(6, 2, 6, 2),
                    ToolTip = $"Zero Valve {ValveDefs[idx].Name}"
                };

                Grid.SetColumn(slider, 0);
                Grid.SetColumn(txtPct, 1);
                Grid.SetColumn(lblPctUnit, 2);
                Grid.SetColumn(btnZero, 3);
                ctrlGrid.Children.Add(slider);
                ctrlGrid.Children.Add(txtPct);
                ctrlGrid.Children.Add(lblPctUnit);
                ctrlGrid.Children.Add(btnZero);
                stack.Children.Add(ctrlGrid);

                border.Child = stack;
                PanelValveSliders.Children.Add(border);

                var row = new ValveRowUi
                {
                    Index = idx,
                    Slider = slider,
                    TxtPct = txtPct,
                    TxtMeta = lblMeta
                };
                _valveRows[idx] = row;

                slider.ValueChanged += (_, _) =>
                {
                    if (row.Updating) return;
                    SetValvePercent(idx, slider.Value, fromSlider: true);
                };

                txtPct.TextChanged += (_, _) =>
                {
                    if (row.Updating) return;
                    if (double.TryParse(txtPct.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double v))
                    {
                        SetValvePercent(idx, Math.Clamp(v, 0.0, 100.0), fromSlider: false);
                    }
                };

                btnZero.Click += (_, _) => SetValvePercent(idx, 0.0, fromSlider: false, forceSyncBoth: true);
            }
        }

        private void SetValvePercent(int idx, double pct, bool fromSlider, bool forceSyncBoth = false)
        {
            pct = Math.Clamp(pct, 0.0, 100.0);
            lock (_targetLock)
            {
                _pctTargets[idx] = pct;
            }

            var row = _valveRows[idx];
            row.Updating = true;
            if (!fromSlider || forceSyncBoth)
            {
                row.Slider.Value = pct;
            }
            if (fromSlider || forceSyncBoth)
            {
                row.TxtPct.Text = pct.ToString("F1", CultureInfo.InvariantCulture);
            }
            row.Updating = false;

            UpdateRowMeta(row, pct);
        }

        private void RefreshValveMetas()
        {
            foreach (var row in _valveRows)
            {
                if (row == null) continue;
                UpdateRowMeta(row, row.Slider.Value);
            }
        }

        private void UpdateRowMeta(ValveRowUi row, double pct)
        {
            byte uByte = PctToWireByte(pct);
            double forceN = uByte * HapticConstants.ForceLsbN;
            double vDac = PanelMapping.CurrentConfig.ComputeVoltage(forceN);
            row.TxtMeta.Text = $"Byte: {uByte,3} | F: {forceN,4:F1} N | DAC: {vDac:F2} V";
        }

        private static byte PctToWireByte(double pct)
        {
            return (byte)Math.Clamp((int)Math.Round(pct * 255.0 / 100.0), 0, 255);
        }

        // ------------------------------------------------------------------
        // Connection & 100 Hz Streamer
        // ------------------------------------------------------------------

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

        private void OnRefreshPortsClick(object sender, RoutedEventArgs e)
        {
            RefreshComPorts();
        }

        private void OnConnectClick(object sender, RoutedEventArgs e)
        {
            if (_motor != null)
            {
                StopStreamerAndVent();
                DisconnectMotor();
                BtnConnect.Content = "Connect";
                CboComPort.IsEnabled = true;
                BtnPingEsp32.IsEnabled = false;
                TxtComStatus.Text = "Disconnected";
                TxtComStatus.Foreground = Brushes.Gray;
                PanelAdc.Clear();
                return;
            }

            string? port = CboComPort.SelectedItem as string;
            if (string.IsNullOrWhiteSpace(port))
            {
                MessageBox.Show(this, "Select a COM port first.", "COM Port", MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }

            _motor = new MotorWorker(port, 1_000_000, pollHz: 25.0);
            _motor.Start();
            BtnConnect.Content = "Disconnect";
            CboComPort.IsEnabled = false;
            TxtComStatus.Text = $"Connecting to {port}...";
            TxtComStatus.Foreground = Brushes.DarkOrange;
        }

        private void DisconnectMotor()
        {
            if (_motor != null)
            {
                _motor.Shutdown();
                _motor.Dispose();
                _motor = null;
            }
        }

        private void SyncStreamer(bool motorConnected)
        {
            bool want = motorConnected && _motor != null && ChkStream.IsChecked == true;
            if (want && _txThread == null)
            {
                _txCts = new CancellationTokenSource();
                _txThread = new Thread(() => RunStreamLoop(_txCts.Token))
                {
                    IsBackground = true,
                    Name = "ValveManualStreamer"
                };
                _txThread.Start();
            }
            else if (!want && _txThread != null)
            {
                StopStreamerAndVent();
            }
        }

        private void StopStreamerAndVent()
        {
            if (_txCts != null)
            {
                _txCts.Cancel();
                _txThread?.Join(500);
                _txCts.Dispose();
                _txCts = null;
                _txThread = null;
            }
            if (_motor != null && _motor.Snapshot().Connected)
            {
                byte[] zeroFrame = _encoder.Encode(new byte[8]);
                _motor.WriteFrame(zeroFrame);
                lock (_statsLock)
                {
                    _lastWireFrame = zeroFrame;
                    _txRateHz = 0.0;
                }
            }
        }

        private void RunStreamLoop(CancellationToken token)
        {
            bool timerRaised = false;
            if (RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
            {
                try { timerRaised = TimeBeginPeriod(1) == 0; } catch { }
            }

            try
            {
                var loopSw = Stopwatch.StartNew();
                var rateSw = Stopwatch.StartNew();
                long periodTicks = (long)(Stopwatch.Frequency * HapticConstants.PeriodSec);
                long nextTick = loopSw.ElapsedTicks;
                int rateCount = 0;
                byte counter = 0;

                while (!token.IsCancellationRequested)
                {
                    long cur = loopSw.ElapsedTicks;
                    if (cur < nextTick)
                    {
                        long remMs = (nextTick - cur) * 1000 / Stopwatch.Frequency;
                        if (remMs > 1) Thread.Sleep((int)(remMs - 1));
                        while (loopSw.ElapsedTicks < nextTick && !token.IsCancellationRequested)
                            Thread.SpinWait(10);
                    }
                    nextTick += periodTicks;

                    var motor = _motor;
                    if (motor == null) continue;

                    byte[] payload = new byte[8];
                    lock (_targetLock)
                    {
                        for (int i = 0; i < 5; i++)
                            payload[i] = PctToWireByte(_pctTargets[i]);
                    }
                    payload[5] = 0;
                    payload[6] = 0;
                    payload[7] = counter++;

                    byte[] wireFrame = _encoder.Encode(payload);
                    bool ok = false;
                    try
                    {
                        ok = motor.WriteFrame(wireFrame) == wireFrame.Length;
                    }
                    catch
                    {
                        ok = false;
                    }

                    rateCount++;
                    double? newHz = null;
                    if (rateSw.ElapsedMilliseconds >= 500)
                    {
                        newHz = rateCount / (rateSw.ElapsedMilliseconds / 1000.0);
                        rateCount = 0;
                        rateSw.Restart();
                    }

                    lock (_statsLock)
                    {
                        if (ok) _txSent++; else _txErrors++;
                        if (newHz.HasValue) _txRateHz = newHz.Value;
                        _lastWireFrame = wireFrame;
                    }
                }
            }
            finally
            {
                if (timerRaised)
                {
                    try { TimeEndPeriod(1); } catch { }
                }
            }
        }

        // ------------------------------------------------------------------
        // UI Actions
        // ------------------------------------------------------------------

        private void OnStreamToggled(object sender, RoutedEventArgs e)
        {
            bool connected = _motor?.Snapshot().Connected == true;
            SyncStreamer(connected);
        }

        private void OnVentAllClick(object sender, RoutedEventArgs e)
        {
            for (int i = 0; i < 5; i++)
                SetValvePercent(i, 0.0, fromSlider: false, forceSyncBoth: true);
        }

        private void OnSetAllPresetClick(object sender, RoutedEventArgs e)
        {
            if (sender is Button btn &&
                double.TryParse(btn.Tag?.ToString(), NumberStyles.Float, CultureInfo.InvariantCulture, out double pct))
            {
                for (int i = 0; i < 5; i++)
                    SetValvePercent(i, pct, fromSlider: false, forceSyncBoth: true);
            }
        }

        private void OnPingEsp32Click(object sender, RoutedEventArgs e)
        {
            _motor?.PingHaptic(0x5A);
        }

        private void OnPlotFilterChanged(object sender, RoutedEventArgs e)
        {
            DrawAdcPlot();
        }

        private void OnClearPlotClick(object sender, RoutedEventArgs e)
        {
            _adcHistory.Clear();
            DrawAdcPlot();
        }

        private void OnCanvasSizeChanged(object sender, SizeChangedEventArgs e)
        {
            DrawAdcPlot();
        }

        // ------------------------------------------------------------------
        // 30 FPS UI Refresh
        // ------------------------------------------------------------------

        private void OnUiTick(object? sender, EventArgs e)
        {
            var snap = _motor?.Snapshot();
            PanelAdc.Update(snap);
            PanelMapping.UpdateFromMotor(snap);

            bool connected = snap != null && snap.Connected;
            BtnPingEsp32.IsEnabled = connected;

            if (snap != null)
            {
                if (connected)
                {
                    TxtComStatus.Text = $"Connected ({_motor!.Port}, {snap.Firmware})";
                    TxtComStatus.Foreground = Brushes.ForestGreen;
                }
                else if (!string.IsNullOrEmpty(snap.Error))
                {
                    TxtComStatus.Text = $"Error: {snap.Error}";
                    TxtComStatus.Foreground = Brushes.Crimson;
                }

                if (snap.HapticPingPending)
                {
                    TxtPingResult.Text = "ESP32 ping: waiting...";
                    TxtPingResult.Foreground = Brushes.Gray;
                }
                else if (!string.IsNullOrEmpty(snap.HapticPing))
                {
                    TxtPingResult.Text = snap.HapticPing;
                    TxtPingResult.Foreground = snap.HapticPingOk ? Brushes.ForestGreen : Brushes.Crimson;
                }

                if (connected && snap.AdcAvailable && snap.AdcUpdated && snap.AdcChannels.Length >= 5)
                {
                    if (snap.AdcSeq != _lastAdcSeqSeen || _adcHistory.Count == 0)
                    {
                        _lastAdcSeqSeen = snap.AdcSeq;
                        double[] mv = new double[5];
                        for (int i = 0; i < 5; i++)
                            mv[i] = snap.AdcChannels[i] * 3300.0 / 4095.0;

                        _adcHistory.Add(new AdcPlotSample(_plotSw.Elapsed.TotalSeconds, mv));
                        if (_adcHistory.Count > 450)
                            _adcHistory.RemoveAt(0);

                        TxtAdcLatest.Text =
                            $"Latest mV: CH0={mv[0]:F1} | CH1={mv[1]:F1} | CH2={mv[2]:F1} | CH3={mv[3]:F1} | CH4={mv[4]:F1}";
                        DrawAdcPlot();
                    }
                }
            }

            SyncStreamer(connected);

            long sent, errs;
            double hz;
            byte[] wire;
            lock (_statsLock)
            {
                sent = _txSent;
                errs = _txErrors;
                hz = _txRateHz;
                wire = _lastWireFrame;
            }

            TxtTxStats.Text = _txThread != null
                ? $"TX: streaming ({sent} frames, {hz:F1} Hz, {errs} errors)"
                : $"TX: stopped ({sent} frames total, {errs} errors)";
            TxtWireFrame.Text = wire.Length > 0
                ? "Wire 0x300: " + BitConverter.ToString(wire).Replace("-", " ")
                : "Wire 0x300: ---";
        }

        private void DrawAdcPlot()
        {
            if (CanvasAdcPlot == null) return;
            CanvasAdcPlot.Children.Clear();

            double w = CanvasAdcPlot.ActualWidth;
            double h = CanvasAdcPlot.ActualHeight;
            if (w < 40 || h < 40 || _adcHistory.Count < 2) return;

            const double padL = 52, padR = 14, padT = 12, padB = 22;
            double pw = w - padL - padR;
            double ph = h - padT - padB;
            if (pw <= 10 || ph <= 10) return;

            bool[] show =
            {
                ChkPlot0.IsChecked == true,
                ChkPlot1.IsChecked == true,
                ChkPlot2.IsChecked == true,
                ChkPlot3.IsChecked == true,
                ChkPlot4.IsChecked == true,
            };

            double tMin = _adcHistory[0].TimeSec;
            double tMax = Math.Max(tMin + 0.1, _adcHistory[^1].TimeSec);

            double yMax = 500.0;
            foreach (var s in _adcHistory)
            {
                for (int c = 0; c < 5; c++)
                {
                    if (show[c]) yMax = Math.Max(yMax, s.Mv[c]);
                }
            }
            yMax = Math.Min(3300.0, Math.Ceiling(yMax / 250.0) * 250.0);

            for (int i = 0; i <= 4; i++)
            {
                double y = padT + ph * i / 4.0;
                CanvasAdcPlot.Children.Add(new Line
                {
                    X1 = padL,
                    Y1 = y,
                    X2 = padL + pw,
                    Y2 = y,
                    Stroke = new SolidColorBrush(Color.FromRgb(236, 236, 236)),
                    StrokeThickness = 1
                });
                double mv = yMax * (4 - i) / 4.0;
                AddPlotLabel($"{mv:F0} mV", 2, y - 7, Brushes.DimGray);
            }

            AddPlotLabel($"{tMin:F1} s", padL, padT + ph + 3, Brushes.DimGray);
            AddPlotLabel($"{tMax:F1} s", padL + pw - 42, padT + ph + 3, Brushes.DimGray);

            for (int c = 0; c < 5; c++)
            {
                if (!show[c]) continue;
                var poly = new Polyline { Stroke = AdcBrushes[c], StrokeThickness = 1.6 };
                foreach (var s in _adcHistory)
                {
                    double x = padL + (s.TimeSec - tMin) / (tMax - tMin) * pw;
                    double y = padT + (1.0 - Math.Clamp(s.Mv[c] / yMax, 0.0, 1.0)) * ph;
                    poly.Points.Add(new Point(x, y));
                }
                CanvasAdcPlot.Children.Add(poly);
            }
        }

        private void AddPlotLabel(string text, double x, double y, Brush color)
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
            CanvasAdcPlot.Children.Add(tb);
        }
    }
}
