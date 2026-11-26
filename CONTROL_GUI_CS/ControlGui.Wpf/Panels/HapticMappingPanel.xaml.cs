using System;
using System.Globalization;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using ControlGui.Core.Haptic;
using ControlGui.Core.Motor;
using ControlGui.Core.TueUsb;

namespace ControlGui.Wpf.Panels
{
    public partial class HapticMappingPanel : UserControl
    {
        private bool _uiReady = false;
        private bool _updatingUi = false;
        private long _lastMappingSeq = -1;
        private bool _wasConnected = false;
        private HapticMappingConfig _currentConfig = HapticMappingConfig.Default;

        public HapticMappingPanel()
        {
            _uiReady = false;
            InitializeComponent();
            _uiReady = true;
            RefreshEquationAndNotify(notify: false);
        }

        public HapticMappingConfig CurrentConfig => _currentConfig;

        public event Action<double, double, double, double, bool>? ApplyRequested;
        public event Action? ReadRequested;
        public event Action<HapticMappingConfig>? MappingChanged;

        public void UpdateFromMotor(MotorSnapshot? snap)
        {
            bool connected = snap != null && snap.Connected;
            BtnRead.IsEnabled = connected && !snap!.HapticMappingPending;
            BtnApply.IsEnabled = connected && !snap!.HapticMappingPending;

            // Auto-query active ESP32 config once right after connecting
            if (connected && !_wasConnected)
            {
                _wasConnected = true;
                ReadRequested?.Invoke();
            }
            else if (!connected)
            {
                _wasConnected = false;
            }

            if (snap == null)
            {
                TxtStatus.Text = "ESP32 config: disconnected";
                TxtStatus.Foreground = Brushes.Gray;
                return;
            }

            if (snap.HapticMapping != null && snap.HapticMappingSeq != _lastMappingSeq)
            {
                _lastMappingSeq = snap.HapticMappingSeq;
                SetInputs(snap.HapticMapping);
            }

            if (!string.IsNullOrEmpty(snap.HapticMappingStatus))
            {
                TxtStatus.Text = snap.HapticMappingStatus;
                TxtStatus.Foreground = snap.HapticMappingPending
                    ? Brushes.DarkOrange
                    : (snap.HapticMappingOk ? Brushes.ForestGreen : Brushes.Crimson);
            }
        }

        private void SetInputs(HapticMappingConfig cfg)
        {
            _updatingUi = true;
            TxtVMin.Text = cfg.VMinV.ToString("F2", CultureInfo.InvariantCulture);
            TxtVMax.Text = cfg.VMaxV.ToString("F2", CultureInfo.InvariantCulture);
            TxtFThresh.Text = cfg.FThreshN.ToString("F2", CultureInfo.InvariantCulture);
            TxtFRated.Text = cfg.FRatedN.ToString("F2", CultureInfo.InvariantCulture);
            _updatingUi = false;
            _currentConfig = cfg;
            RefreshEquationAndNotify(notify: true);
        }

        private bool TryParseInputs(out double vMin, out double vMax, out double fThresh, out double fRated)
        {
            vMin = HapticConstants.VMin;
            vMax = HapticConstants.VMax;
            fThresh = HapticConstants.ForceThresh;
            fRated = HapticConstants.ForceRated;

            if (!double.TryParse(TxtVMin.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out vMin)) return false;
            if (!double.TryParse(TxtVMax.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out vMax)) return false;
            if (!double.TryParse(TxtFThresh.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out fThresh)) return false;
            if (!double.TryParse(TxtFRated.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out fRated)) return false;

            vMin = Math.Clamp(vMin, 0.0, HapticConstants.VHwCeiling);
            vMax = Math.Clamp(vMax, vMin, HapticConstants.VHwCeiling);
            fThresh = Math.Clamp(fThresh, 0.0, 25.0);
            fRated = Math.Clamp(fRated, fThresh + 0.1, 25.5);
            return true;
        }

        private void RefreshEquationAndNotify(bool notify)
        {
            if (TryParseInputs(out double vMin, out double vMax, out double fThresh, out double fRated))
            {
                _currentConfig = _currentConfig with
                {
                    VMinV = vMin,
                    VMaxV = vMax,
                    FThreshN = fThresh,
                    FRatedN = fRated
                };
            }

            TxtEquation.Text = _currentConfig.FormatEquationSummary();

            if (PanelHelp.Visibility == Visibility.Visible)
            {
                TxtHelpDetail.Text = BuildDetailedHelp(_currentConfig);
            }

            if (notify)
            {
                MappingChanged?.Invoke(_currentConfig);
            }
        }

        private static string BuildDetailedHelp(HapticMappingConfig cfg)
        {
            double k = cfg.SlopeVPerN;
            return
                "HÀM TRUYỀN LỰC -> ĐIỆN ÁP VAN TRÊN ESP32 (haptic_core.cpp):\n" +
                $"1) Vùng chết (F <= F_thresh = {cfg.FThreshN:F2} N): V = 0.000 V (xả áp hoàn toàn, lọc nhiễu không tải).\n" +
                $"2) Vùng mở tối thiểu (tại F > {cfg.FThreshN:F2} N): nhảy lên V_min = {cfg.VMinV:F2} V để thắng deadband cơ khí van ITV / màng SuP.\n" +
                $"3) Vùng tuyến tính ({cfg.FThreshN:F2} < F < {cfg.FRatedN:F2} N): V(F) = {cfg.VMinV:F2} + {k:F5} * (F - {cfg.FThreshN:F2}) V.\n" +
                $"4) Vùng bão hòa an toàn (F >= F_rated = {cfg.FRatedN:F2} N): kẹp cứng ở V_max = {cfg.VMaxV:F2} V (trần phần cứng 3.30 V).\n" +
                "5) DAC MCP4725 12-bit: DAC = floor(V / 3.30 * 4095). Chỉ ghi I2C khi |ΔV| >= 0.015 V.\n" +
                "• Save to EEPROM (NVS): tick để ghi vào Flash của ESP32 (giữ nguyên qua các lần bật tắt nguồn); bỏ tick chỉ áp dụng tạm trên RAM.\n" +
                "• Read ESP32 (0x305): đọc lại thông số đang chạy trên ESP32 và trạng thái đã lưu EEPROM hay chưa.";
        }

        private void OnParamTextChanged(object sender, TextChangedEventArgs e)
        {
            if (!_uiReady || _updatingUi) return;
            RefreshEquationAndNotify(notify: true);
        }

        private void OnReadClick(object sender, RoutedEventArgs e)
        {
            ReadRequested?.Invoke();
        }

        private void OnApplyClick(object sender, RoutedEventArgs e)
        {
            if (!TryParseInputs(out double vMin, out double vMax, out double fThresh, out double fRated))
            {
                MessageBox.Show(Window.GetWindow(this),
                    "Invalid mapping numbers. Ensure 0 <= V_min <= V_max <= 3.30 V and 0 <= F_thresh < F_rated <= 25.5 N.",
                    "Invalid Mapping", MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }
            bool saveEeprom = ChkSaveEeprom.IsChecked == true;
            ApplyRequested?.Invoke(vMin, vMax, fThresh, fRated, saveEeprom);
        }

        private void OnDefaultClick(object sender, RoutedEventArgs e)
        {
            SetInputs(HapticMappingConfig.Default);
        }

        private void OnHelpToggled(object sender, RoutedEventArgs e)
        {
            if (!_uiReady) return;
            bool show = BtnHelp.IsChecked == true;
            PanelHelp.Visibility = show ? Visibility.Visible : Visibility.Collapsed;
            if (show)
            {
                TxtHelpDetail.Text = BuildDetailedHelp(_currentConfig);
            }
        }
    }
}
