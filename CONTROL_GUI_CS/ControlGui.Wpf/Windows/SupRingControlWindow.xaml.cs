using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Runtime.InteropServices;
using System.Threading;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;
using ControlGui.Core.Haptic;
using ControlGui.Core.Motor;
using ControlGui.Core.TueUsb;
using Microsoft.Win32;
using Path = System.IO.Path;

namespace ControlGui.Wpf.Windows
{
    public partial class SupRingControlWindow : Window
    {
        [DllImport("winmm.dll", EntryPoint = "timeBeginPeriod")]
        private static extern uint TimeBeginPeriod(uint uMilliseconds);

        [DllImport("winmm.dll", EntryPoint = "timeEndPeriod")]
        private static extern uint TimeEndPeriod(uint uMilliseconds);

        private readonly record struct AdcPlotSample(double TimeSec, double[] Mv);

        private static readonly (string Name, string Desc, double PadX, double PadY)[] ChamberDefs =
        {
            ("A", "Chamber A · -Fx (Distal Tip · Slide Down)",   0.0, -1.0),
            ("B", "Chamber B · Fz  (Medial Mid · Squeeze)",       0.0,  0.0),
            ("C", "Chamber C · +Fx (Proximal Base · Slide Up)",   0.0,  1.0),
            ("D", "Chamber D · -Fy (Left Wall · Pull)",          -1.0,  0.0),
            ("E", "Chamber E · +Fy (Right Wall · Push)",          1.0,  0.0),
        };

        private static readonly Brush[] AdcBrushes =
        {
            new SolidColorBrush(Color.FromRgb(0xD3, 0x2F, 0x2F)), // CH0 / A
            new SolidColorBrush(Color.FromRgb(0xE6, 0x51, 0x00)), // CH1 / B
            new SolidColorBrush(Color.FromRgb(0x2E, 0x7D, 0x32)), // CH2 / C
            new SolidColorBrush(Color.FromRgb(0x15, 0x65, 0xC0)), // CH3 / D
            new SolidColorBrush(Color.FromRgb(0x6A, 0x1B, 0x9A)), // CH4 / E
        };

        private static readonly Color[] ChamberColors =
        {
            Color.FromRgb(0xD3, 0x2F, 0x2F),
            Color.FromRgb(0xE6, 0x51, 0x00),
            Color.FromRgb(0x2E, 0x7D, 0x32),
            Color.FromRgb(0x15, 0x65, 0xC0),
            Color.FromRgb(0x6A, 0x1B, 0x9A),
        };

        private readonly TextBlock[] _valveReadouts = new TextBlock[5];
        private readonly object _stateLock = new();
        private readonly object _statsLock = new();
        private readonly TueBinaryEncoder _encoder = new();
        private readonly SupRingSqliteLogger _logger = new();

        private readonly List<AdcPlotSample> _adcHistory = new(450);
        private readonly Stopwatch _plotSw = Stopwatch.StartNew();
        private readonly DispatcherTimer _uiTimer;

        private SupRingState _state = new();
        private double _txRateHzTarget = HapticConstants.RateHz;
        private bool _uiReady = false;
        private bool _updatingUi = false;
        private bool _padDragging = false;

        private MotorWorker? _motor;
        private Thread? _txThread;
        private CancellationTokenSource? _txCts;

        private long _txSent = 0;
        private long _txErrors = 0;
        private double _txActualRateHz = 0.0;
        private byte[] _lastWireFrame = Array.Empty<byte>();
        private uint _lastAdcSeqSeen = uint.MaxValue;

        public SupRingControlWindow()
        {
            _uiReady = false;
            InitializeComponent();
            BuildChamberFeedbackRows();
            RefreshComPorts();
            TxtDbPath.Text = Path.Combine("recordings", $"sup_ring_{DateTime.Now:yyyyMMdd_HHmmss}.db");
            _uiReady = true;

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
            PanelMapping.MappingChanged += _ => UpdateStateDisplay();

            UpdateStateDisplay();

            _uiTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(33) };
            _uiTimer.Tick += OnUiTick;
            _uiTimer.Start();

            Closing += (_, _) =>
            {
                _uiTimer.Stop();
                _logger.Stop();
                StopStreamerAndVent();
                DisconnectMotor();
            };
        }

        private void BuildChamberFeedbackRows()
        {
            for (int i = 0; i < 5; i++)
            {
                var grid = new Grid { Margin = new Thickness(0, 1, 0, 1) };
                grid.ColumnDefinitions.Add(new ColumnDefinition { Width = new GridLength(1, GridUnitType.Star) });
                grid.ColumnDefinitions.Add(new ColumnDefinition { Width = GridLength.Auto });

                var lblName = new TextBlock
                {
                    Text = ChamberDefs[i].Desc,
                    Foreground = AdcBrushes[i],
                    FontWeight = FontWeights.SemiBold,
                    FontSize = 11.5,
                    VerticalAlignment = VerticalAlignment.Center
                };
                var lblVal = new TextBlock
                {
                    Text = "F:  0.0 N | Byte:   0 | DAC: 0.00 V",
                    FontFamily = new FontFamily("Consolas"),
                    FontSize = 11.5,
                    Foreground = new SolidColorBrush(Color.FromRgb(0x42, 0x42, 0x42)),
                    VerticalAlignment = VerticalAlignment.Center
                };

                Grid.SetColumn(lblName, 0);
                Grid.SetColumn(lblVal, 1);
                grid.Children.Add(lblName);
                grid.Children.Add(lblVal);

                _valveReadouts[i] = lblVal;
                PanelValveFeedback.Children.Add(grid);
            }
        }

        // ------------------------------------------------------------------
        // Target State Management & Sync (Pad <-> Sliders <-> TextBoxes)
        // ------------------------------------------------------------------

        private void ApplyForces(double fx, double fy, double fz, bool syncPolar = true, bool syncCartesian = true)
        {
            fx = Math.Clamp(fx, -HapticConstants.MaxForceN, HapticConstants.MaxForceN);
            fy = Math.Clamp(fy, -HapticConstants.MaxForceN, HapticConstants.MaxForceN);
            fz = Math.Clamp(fz, 0.0, HapticConstants.MaxForceN);

            lock (_stateLock)
            {
                _state = _state with { Fx = fx, Fy = fy, Fz = fz };
            }

            _updatingUi = true;
            if (syncCartesian)
            {
                SliderFz.Value = fz;
                TxtFz.Text = fz.ToString("F1", CultureInfo.InvariantCulture);
                SliderFx.Value = fx;
                TxtFx.Text = fx.ToString("F1", CultureInfo.InvariantCulture);
                SliderFy.Value = fy;
                TxtFy.Text = fy.ToString("F1", CultureInfo.InvariantCulture);
            }
            if (syncPolar)
            {
                double mag = Math.Clamp(SupRingMapper.ShearMagnitude(fx, fy), 0.0, HapticConstants.MaxForceN);
                double ang = SupRingMapper.ShearAngleDeg(fx, fy);
                SliderMag.Value = mag;
                TxtMag.Text = mag.ToString("F1", CultureInfo.InvariantCulture);
                SliderAngle.Value = ang;
                TxtAngle.Text = ang.ToString("F1", CultureInfo.InvariantCulture);
            }
            _updatingUi = false;

            UpdateStateDisplay();
        }

        private void UpdateStateDisplay()
        {
            SupRingState st;
            lock (_stateLock) st = _state;

            byte[] payload = SupRingMapper.BuildPayload(st, 0);
            var mapCfg = PanelMapping.CurrentConfig;
            for (int i = 0; i < 5; i++)
            {
                byte b = payload[i];
                double forceN = b * HapticConstants.ForceLsbN;
                double dacV = mapCfg.ComputeVoltage(forceN);
                _valveReadouts[i].Text = $"F: {forceN,4:F1} N | Byte: {b,3} | DAC: {dacV:F2} V";
            }

            if (ChkAutoFlags.IsChecked == true)
            {
                byte autoFlags = payload[6];
                _updatingUi = true;
                ChkFlagL.IsChecked = (autoFlags & HapticConstants.FlagLeftContact) != 0;
                ChkFlagR.IsChecked = (autoFlags & HapticConstants.FlagRightContact) != 0;
                ChkFlagSlip.IsChecked = (autoFlags & HapticConstants.FlagSlip) != 0;
                ChkFlagTare.IsChecked = (autoFlags & HapticConstants.FlagTare) != 0;
                _updatingUi = false;
            }

            DrawSupPad(st, payload, mapCfg);
        }

        private void OnCartesianChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (!_uiReady || _updatingUi) return;
            ApplyForces(SliderFx.Value, SliderFy.Value, SliderFz.Value, syncPolar: true, syncCartesian: true);
        }

        private void OnCartesianTextChanged(object sender, TextChangedEventArgs e)
        {
            if (!_uiReady || _updatingUi) return;
            if (double.TryParse(TxtFx.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double fx) &&
                double.TryParse(TxtFy.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double fy) &&
                double.TryParse(TxtFz.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double fz))
            {
                lock (_stateLock)
                {
                    fx = Math.Clamp(fx, -HapticConstants.MaxForceN, HapticConstants.MaxForceN);
                    fy = Math.Clamp(fy, -HapticConstants.MaxForceN, HapticConstants.MaxForceN);
                    fz = Math.Clamp(fz, 0.0, HapticConstants.MaxForceN);
                    _state = _state with { Fx = fx, Fy = fy, Fz = fz };
                }
                _updatingUi = true;
                SliderFx.Value = fx;
                SliderFy.Value = fy;
                SliderFz.Value = fz;
                double mag = Math.Clamp(SupRingMapper.ShearMagnitude(fx, fy), 0.0, HapticConstants.MaxForceN);
                double ang = SupRingMapper.ShearAngleDeg(fx, fy);
                SliderMag.Value = mag;
                TxtMag.Text = mag.ToString("F1", CultureInfo.InvariantCulture);
                SliderAngle.Value = ang;
                TxtAngle.Text = ang.ToString("F1", CultureInfo.InvariantCulture);
                _updatingUi = false;
                UpdateStateDisplay();
            }
        }

        private void OnPolarChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (!_uiReady || _updatingUi) return;
            var (fx, fy) = SupRingMapper.FromPolar(SliderMag.Value, SliderAngle.Value);
            double fz;
            lock (_stateLock) fz = _state.Fz;
            ApplyForces(fx, fy, fz, syncPolar: false, syncCartesian: true);
            _updatingUi = true;
            TxtMag.Text = SliderMag.Value.ToString("F1", CultureInfo.InvariantCulture);
            TxtAngle.Text = SliderAngle.Value.ToString("F1", CultureInfo.InvariantCulture);
            _updatingUi = false;
        }

        private void OnPolarTextChanged(object sender, TextChangedEventArgs e)
        {
            if (!_uiReady || _updatingUi) return;
            if (double.TryParse(TxtMag.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double mag) &&
                double.TryParse(TxtAngle.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double ang))
            {
                mag = Math.Clamp(mag, 0.0, HapticConstants.MaxForceN);
                ang = ((ang % 360.0) + 360.0) % 360.0;
                var (fx, fy) = SupRingMapper.FromPolar(mag, ang);
                double fz;
                lock (_stateLock) fz = _state.Fz;
                lock (_stateLock) _state = _state with { Fx = fx, Fy = fy, Fz = fz };
                _updatingUi = true;
                SliderMag.Value = mag;
                SliderAngle.Value = ang;
                SliderFx.Value = fx;
                TxtFx.Text = fx.ToString("F1", CultureInfo.InvariantCulture);
                SliderFy.Value = fy;
                TxtFy.Text = fy.ToString("F1", CultureInfo.InvariantCulture);
                _updatingUi = false;
                UpdateStateDisplay();
            }
        }

        // ------------------------------------------------------------------
        // Interactive 2D SuP / Ring Pad
        // ------------------------------------------------------------------

        private (double Cx, double Cy, double Radius) GetPadGeometry()
        {
            double w = CanvasSupPad.ActualWidth;
            double h = CanvasSupPad.ActualHeight;
            double cx = w / 2.0;
            double cy = h / 2.0;
            double radius = Math.Max(20.0, Math.Min(w, h) * 0.38);
            return (cx, cy, radius);
        }

        private void SteerFromPadPoint(Point pt)
        {
            var (cx, cy, radius) = GetPadGeometry();
            // Horizontal right = +Fy (Push · E), Horizontal left = -Fy (Pull · D)
            // Vertical up = -Fx (Slide Down · A), Vertical down = +Fx (Slide Up · C)
            double dx = (pt.X - cx) / radius;
            double dy = (pt.Y - cy) / radius;
            double norm = Math.Sqrt(dx * dx + dy * dy);
            if (norm > 1.0)
            {
                dx /= norm;
                dy /= norm;
            }
            double fy = dx * HapticConstants.MaxForceN;
            double fx = dy * HapticConstants.MaxForceN;

            double fz;
            lock (_stateLock) fz = _state.Fz;
            ApplyForces(fx, fy, fz);
        }

        private void OnPadSizeChanged(object sender, SizeChangedEventArgs e)
        {
            if (!_uiReady) return;
            UpdateStateDisplay();
        }

        private void OnPadMouseDown(object sender, MouseButtonEventArgs e)
        {
            if (e.ClickCount >= 2)
            {
                OnZeroShearClick(sender, e);
                return;
            }
            _padDragging = true;
            CanvasSupPad.CaptureMouse();
            SteerFromPadPoint(e.GetPosition(CanvasSupPad));
        }

        private void OnPadMouseMove(object sender, MouseEventArgs e)
        {
            if (!_padDragging) return;
            SteerFromPadPoint(e.GetPosition(CanvasSupPad));
        }

        private void OnPadMouseUp(object sender, MouseButtonEventArgs e)
        {
            if (!_padDragging) return;
            _padDragging = false;
            CanvasSupPad.ReleaseMouseCapture();
        }

        private void OnPadRightDown(object sender, MouseButtonEventArgs e)
        {
            OnZeroShearClick(sender, e);
        }

        private void DrawSupPad(SupRingState st, byte[] payload, HapticMappingConfig mapCfg)
        {
            if (CanvasSupPad == null || CanvasSupPad.ActualWidth < 40 || CanvasSupPad.ActualHeight < 40) return;
            CanvasSupPad.Children.Clear();

            var (cx, cy, r) = GetPadGeometry();

            // Outer boundary circle (max shear 25.5 N)
            var outer = new Ellipse
            {
                Width = r * 2,
                Height = r * 2,
                Stroke = new SolidColorBrush(Color.FromRgb(0xBD, 0xBD, 0xBD)),
                StrokeThickness = 1.2,
                Fill = new SolidColorBrush(Color.FromArgb(14, 0x90, 0xCA, 0xF9))
            };
            Canvas.SetLeft(outer, cx - r);
            Canvas.SetTop(outer, cy - r);
            CanvasSupPad.Children.Add(outer);

            // Half-scale ring (12.75 N)
            var inner = new Ellipse
            {
                Width = r,
                Height = r,
                Stroke = new SolidColorBrush(Color.FromRgb(0xE0, 0xE0, 0xE0)),
                StrokeThickness = 1.0,
                StrokeDashArray = new DoubleCollection { 3, 3 }
            };
            Canvas.SetLeft(inner, cx - r / 2.0);
            Canvas.SetTop(inner, cy - r / 2.0);
            CanvasSupPad.Children.Add(inner);

            // Crosshair axes
            var axisStroke = new SolidColorBrush(Color.FromRgb(0xE0, 0xE0, 0xE0));
            CanvasSupPad.Children.Add(new Line { X1 = cx - r, Y1 = cy, X2 = cx + r, Y2 = cy, Stroke = axisStroke, StrokeThickness = 1 });
            CanvasSupPad.Children.Add(new Line { X1 = cx, Y1 = cy - r, X2 = cx, Y2 = cy + r, Stroke = axisStroke, StrokeThickness = 1 });

            // 5 Chamber circles: A(top), B(center), C(bottom), D(left), E(right)
            double chamberOffset = r * 0.72;
            for (int i = 0; i < 5; i++)
            {
                double px = cx + ChamberDefs[i].PadX * chamberOffset;
                double py = cy + ChamberDefs[i].PadY * chamberOffset;
                double forceN = payload[i] * HapticConstants.ForceLsbN;
                double dacV = mapCfg.ComputeVoltage(forceN);
                double spanV = Math.Max(0.05, mapCfg.VMaxV - mapCfg.VMinV);
                double norm = dacV > 0 ? Math.Clamp((dacV - mapCfg.VMinV) / spanV * 0.8 + 0.2, 0.0, 1.0) : 0.0;

                double cr = 13.0 + norm * 8.0;
                Color baseCol = ChamberColors[i];
                byte alpha = (byte)(30 + norm * 195);

                var circle = new Ellipse
                {
                    Width = cr * 2,
                    Height = cr * 2,
                    Fill = new SolidColorBrush(Color.FromArgb(alpha, baseCol.R, baseCol.G, baseCol.B)),
                    Stroke = new SolidColorBrush(baseCol),
                    StrokeThickness = norm > 0 ? 2.0 : 1.0
                };
                Canvas.SetLeft(circle, px - cr);
                Canvas.SetTop(circle, py - cr);
                CanvasSupPad.Children.Add(circle);

                var lbl = new TextBlock
                {
                    Text = $"{ChamberDefs[i].Name}",
                    FontWeight = FontWeights.Bold,
                    FontSize = 11,
                    Foreground = norm > 0.55 ? Brushes.White : AdcBrushes[i]
                };
                Canvas.SetLeft(lbl, px - 4);
                Canvas.SetTop(lbl, py - 8);
                CanvasSupPad.Children.Add(lbl);
            }

            // Direction hints near margins
            AddPadHint("A (-Fx · Slide Down)", cx - 56, cy - r - 17, ChamberColors[0]);
            AddPadHint("C (+Fx · Slide Up)",   cx - 52, cy + r + 2,  ChamberColors[2]);
            AddPadHint("D (-Fy · Pull)",       cx - r - 76, cy - 8,  ChamberColors[3]);
            AddPadHint("E (+Fy · Push)",       cx + r + 8,  cy - 8,  ChamberColors[4]);

            // Drag handle vector (Fy -> +right, Fx -> +down)
            double hx = cx + Math.Clamp(st.Fy / HapticConstants.MaxForceN, -1.0, 1.0) * r;
            double hy = cy + Math.Clamp(st.Fx / HapticConstants.MaxForceN, -1.0, 1.0) * r;

            CanvasSupPad.Children.Add(new Line
            {
                X1 = cx,
                Y1 = cy,
                X2 = hx,
                Y2 = hy,
                Stroke = new SolidColorBrush(Color.FromRgb(0x21, 0x21, 0x21)),
                StrokeThickness = 2.2
            });

            var puck = new Ellipse
            {
                Width = 14,
                Height = 14,
                Fill = new SolidColorBrush(Color.FromRgb(0x19, 0x76, 0xD2)),
                Stroke = Brushes.White,
                StrokeThickness = 2.0
            };
            Canvas.SetLeft(puck, hx - 7);
            Canvas.SetTop(puck, hy - 7);
            CanvasSupPad.Children.Add(puck);
        }

        private void AddPadHint(string text, double x, double y, Color col)
        {
            var tb = new TextBlock
            {
                Text = text,
                FontSize = 10,
                FontWeight = FontWeights.SemiBold,
                Foreground = new SolidColorBrush(col)
            };
            Canvas.SetLeft(tb, Math.Max(2, x));
            Canvas.SetTop(tb, Math.Max(1, y));
            CanvasSupPad.Children.Add(tb);
        }

        // ------------------------------------------------------------------
        // Presets & Parameter Controls
        // ------------------------------------------------------------------

        private void OnPresetDirectionClick(object sender, RoutedEventArgs e)
        {
            if (sender is not Button btn) return;
            string tag = btn.Tag?.ToString() ?? "";

            double curFz, curMag;
            lock (_stateLock)
            {
                curFz = _state.Fz;
                curMag = SupRingMapper.ShearMagnitude(_state.Fx, _state.Fy);
            }
            double shearMag = curMag >= 0.5 ? curMag : 5.0;
            double fz = curFz >= 0.5 ? curFz : 4.0;

            switch (tag)
            {
                case "slide_down": ApplyForces(-shearMag, 0.0, fz); break;
                case "slide_up":   ApplyForces(+shearMag, 0.0, fz); break;
                case "pull":       ApplyForces(0.0, -shearMag, fz); break;
                case "push":       ApplyForces(0.0, +shearMag, fz); break;
                case "squeeze":    ApplyForces(0.0, 0.0, fz); break;
            }
        }

        private void OnZeroShearClick(object sender, RoutedEventArgs e)
        {
            double fz;
            lock (_stateLock) fz = _state.Fz;
            ApplyForces(0.0, 0.0, fz);
        }

        private void OnVentAllClick(object sender, RoutedEventArgs e)
        {
            ApplyForces(0.0, 0.0, 0.0);
        }

        private void OnStreamParamChanged(object sender, RoutedEventArgs e)
        {
            if (!_uiReady) return;
            bool connected = _motor?.Snapshot().Connected == true;
            SyncStreamer(connected);
        }

        private void OnTxRateTextChanged(object sender, TextChangedEventArgs e)
        {
            if (!_uiReady) return;
            if (double.TryParse(TxtTxRateHz.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double hz))
            {
                lock (_stateLock)
                {
                    _txRateHzTarget = Math.Clamp(hz, 10.0, 200.0);
                }
            }
        }

        private void OnFlagsChanged(object sender, RoutedEventArgs e)
        {
            if (!_uiReady || _updatingUi) return;
            bool auto = ChkAutoFlags.IsChecked == true;
            ChkFlagL.IsEnabled = !auto;
            ChkFlagR.IsEnabled = !auto;
            ChkFlagSlip.IsEnabled = !auto;
            ChkFlagTare.IsEnabled = !auto;

            byte manual = 0;
            if (ChkFlagL.IsChecked == true) manual |= HapticConstants.FlagLeftContact;
            if (ChkFlagR.IsChecked == true) manual |= HapticConstants.FlagRightContact;
            if (ChkFlagSlip.IsChecked == true) manual |= HapticConstants.FlagSlip;
            if (ChkFlagTare.IsChecked == true) manual |= HapticConstants.FlagTare;

            lock (_stateLock)
            {
                _state = _state with { AutoFlags = auto, ManualFlags = manual };
            }
            UpdateStateDisplay();
        }

        private void OnReservedTextChanged(object sender, TextChangedEventArgs e)
        {
            if (!_uiReady) return;
            string raw = TxtReserved.Text.Trim();
            bool parsed = false;
            byte val = 0;
            if (raw.StartsWith("0x", StringComparison.OrdinalIgnoreCase))
            {
                parsed = byte.TryParse(raw[2..], NumberStyles.HexNumber, CultureInfo.InvariantCulture, out val);
            }
            else
            {
                parsed = byte.TryParse(raw, NumberStyles.Integer, CultureInfo.InvariantCulture, out val);
            }
            if (parsed)
            {
                lock (_stateLock)
                {
                    _state = _state with { ReservedByte = val };
                }
            }
        }

        // ------------------------------------------------------------------
        // Connection & TX Streaming Thread
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
                _logger.Stop();
                BtnToggleLog.Content = "Start Log";
                BtnToggleLog.Background = new SolidColorBrush(Color.FromRgb(0x19, 0x76, 0xD2));
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

        private void OnPingEsp32Click(object sender, RoutedEventArgs e)
        {
            _motor?.PingHaptic(0x5A);
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
                    Name = "SupRingStreamer"
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
                    _txActualRateHz = 0.0;
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
                long nextTick = loopSw.ElapsedTicks;
                int rateCount = 0;
                byte counter = 0;

                while (!token.IsCancellationRequested)
                {
                    double hzTarget;
                    SupRingState st;
                    lock (_stateLock)
                    {
                        hzTarget = _txRateHzTarget;
                        st = _state;
                    }
                    long periodTicks = (long)(Stopwatch.Frequency / Math.Clamp(hzTarget, 10.0, 200.0));

                    long cur = loopSw.ElapsedTicks;
                    if (cur < nextTick)
                    {
                        long remMs = (nextTick - cur) * 1000 / Stopwatch.Frequency;
                        if (remMs > 1) Thread.Sleep((int)(remMs - 1));
                        while (loopSw.ElapsedTicks < nextTick && !token.IsCancellationRequested)
                            Thread.SpinWait(10);
                    }
                    nextTick += periodTicks;
                    if (loopSw.ElapsedTicks - nextTick > periodTicks * 4)
                        nextTick = loopSw.ElapsedTicks;

                    var motor = _motor;
                    if (motor == null) continue;

                    byte[] payload = SupRingMapper.BuildPayload(st, counter++);
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

                    if (_logger.IsRecording)
                    {
                        var snap = motor.Snapshot();
                        ushort[] adcRaw = snap.AdcChannels != null && snap.AdcChannels.Length >= 5
                            ? (ushort[])snap.AdcChannels.Clone()
                            : new ushort[5];
                        double[] adcMv = new double[5];
                        if (snap.AdcAvailable && snap.AdcUpdated && snap.AdcChannels != null && snap.AdcChannels.Length >= 5)
                        {
                            for (int i = 0; i < 5; i++)
                                adcMv[i] = snap.AdcChannels[i] * 3300.0 / 4095.0;
                        }
                        else
                        {
                            Array.Fill(adcMv, double.NaN);
                        }

                        _logger.Enqueue(new SupRingLogSample(
                            TimeSec: _logger.ElapsedSeconds,
                            Epoch: DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() / 1000.0,
                            Fx: st.Fx,
                            Fy: st.Fy,
                            Fz: st.Fz,
                            ShearMag: SupRingMapper.ShearMagnitude(st.Fx, st.Fy),
                            ShearAngleDeg: SupRingMapper.ShearAngleDeg(st.Fx, st.Fy),
                            Payload: payload,
                            AdcRaw: adcRaw,
                            AdcMv: adcMv));
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
                        if (newHz.HasValue) _txActualRateHz = newHz.Value;
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
        // SQLite Logging Controls
        // ------------------------------------------------------------------

        private void OnBrowseDbClick(object sender, RoutedEventArgs e)
        {
            var dlg = new SaveFileDialog
            {
                Filter = "SQLite Database (*.db)|*.db|All files (*.*)|*.*",
                FileName = Path.GetFileName(TxtDbPath.Text),
                InitialDirectory = Path.GetDirectoryName(Path.GetFullPath(TxtDbPath.Text)) ?? ""
            };
            if (dlg.ShowDialog(this) == true)
            {
                TxtDbPath.Text = dlg.FileName;
            }
        }

        private void OnToggleLogClick(object sender, RoutedEventArgs e)
        {
            if (_logger.IsRecording)
            {
                long finalRows = _logger.RowsLogged;
                string savedPath = _logger.DbPath;
                _logger.Stop();
                BtnToggleLog.Content = "Start Log";
                BtnToggleLog.Background = new SolidColorBrush(Color.FromRgb(0x19, 0x76, 0xD2));
                TxtDbPath.IsEnabled = true;
                BtnBrowseDb.IsEnabled = true;
                TxtLogStatus.Text = $"Saved {finalRows} rows -> {savedPath}";
                TxtLogStatus.Foreground = Brushes.ForestGreen;
                return;
            }

            if (_motor == null || !_motor.Snapshot().Connected)
            {
                MessageBox.Show(this, "Connect to STM32 first before starting SQLite logging.", "Not Connected",
                    MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }

            string dbPath = TxtDbPath.Text.Trim();
            if (string.IsNullOrEmpty(dbPath) || File.Exists(Path.GetFullPath(dbPath)))
            {
                string dir = string.IsNullOrEmpty(dbPath) ? "recordings" : (Path.GetDirectoryName(dbPath) ?? "recordings");
                dbPath = Path.Combine(dir, $"sup_ring_{DateTime.Now:yyyyMMdd_HHmmss}.db");
                TxtDbPath.Text = dbPath;
            }

            double hz;
            lock (_stateLock) hz = _txRateHzTarget;

            _logger.Start(dbPath, hz, 3300.0, PanelMapping.CurrentConfig);
            BtnToggleLog.Content = "Stop Log";
            BtnToggleLog.Background = new SolidColorBrush(Color.FromRgb(0xD3, 0x2F, 0x2F));
            TxtDbPath.IsEnabled = false;
            BtnBrowseDb.IsEnabled = false;
            TxtLogStatus.Foreground = Brushes.Crimson;
        }

        // ------------------------------------------------------------------
        // ADC Waveform Plot & 30 FPS UI Refresh
        // ------------------------------------------------------------------

        private void OnPlotFilterChanged(object sender, RoutedEventArgs e)
        {
            if (!_uiReady) return;
            DrawAdcPlot();
        }

        private void OnClearPlotClick(object sender, RoutedEventArgs e)
        {
            _adcHistory.Clear();
            DrawAdcPlot();
        }

        private void OnCanvasSizeChanged(object sender, SizeChangedEventArgs e)
        {
            if (!_uiReady) return;
            DrawAdcPlot();
        }

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

            if (_logger.IsRecording)
            {
                TxtLogStatus.Text = $"Recording: {_logger.RowsLogged} samples ({_logger.ElapsedSeconds:F1} s) -> {_logger.DbPath}";
            }

            long sent, errs;
            double hz;
            byte[] wire;
            lock (_statsLock)
            {
                sent = _txSent;
                errs = _txErrors;
                hz = _txActualRateHz;
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
