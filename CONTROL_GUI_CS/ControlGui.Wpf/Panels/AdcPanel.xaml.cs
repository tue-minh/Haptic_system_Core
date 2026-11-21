using System;
using System.Windows.Controls;
using ControlGui.Core.Motor;

namespace ControlGui.Wpf.Panels
{
    public partial class AdcPanel : UserControl
    {
        public event Action<bool, byte, byte, byte>? ApplyRequested;
        public event Action? ReadRequested;

        private readonly TextBlock[] _raw;
        private readonly TextBlock[] _mv;
        private readonly ProgressBar[] _bars;
        private bool _configSeeded;

        public AdcPanel()
        {
            InitializeComponent();

            _raw = new[] { TxtRaw0, TxtRaw1, TxtRaw2, TxtRaw3, TxtRaw4 };
            _mv = new[] { TxtMv0, TxtMv1, TxtMv2, TxtMv3, TxtMv4 };
            _bars = new[] { Bar0, Bar1, Bar2, Bar3, Bar4 };

            CmbCs.ItemsSource = new[] { "CS1", "CS2", "CS3" };
            CmbCs.SelectedIndex = 0;
            CmbPeriod.ItemsSource = new[] { "10 ms", "20 ms", "50 ms", "100 ms" };
            CmbPeriod.SelectedIndex = 0;

            SetAvailable(false);
        }

        public void Update(MotorSnapshot? snap)
        {
            if (snap == null || !snap.Connected || !snap.AdcAvailable)
            {
                SetAvailable(false, snap?.AdcLastError ?? "");
                return;
            }

            SetAvailable(true);

            var channels = snap.AdcChannels;
            for (int i = 0; i < _raw.Length; i++)
            {
                int raw = channels != null && i < channels.Length
                    ? Math.Clamp((int)channels[i], 0, 4095)
                    : 0;
                _raw[i].Text = $"{raw,4}";
                _mv[i].Text = $"{raw * 3300.0 / 4095.0,7:F1} mV";
                _bars[i].Value = raw;
            }

            string flags = snap.AdcValid ? "valid" : "no-valid";
            if (snap.AdcError)
            {
                flags += ", ADC-ERR";
            }

            TxtStatus.Text = $"seq {snap.AdcSeq,3} · status 0x{snap.AdcStatus:X2} [{flags}] · " +
                             $"ok {snap.AdcRxOk} · crc-err {snap.AdcCrcErr} · {snap.AdcHz,4:F1} Hz";
            TxtStatus.Foreground = snap.AdcError ? Theme.DangerBrush : Theme.TextDimBrush;

            if (!_configSeeded)
            {
                SeedConfig(snap);
                _configSeeded = true;
            }
        }

        public void Clear()
        {
            SetAvailable(false, "not connected");
        }

        private void SeedConfig(MotorSnapshot snap)
        {
            CmbCs.SelectedIndex = Math.Clamp((int)snap.AdcCsIdx, 0, 2);
            CmbPeriod.SelectedIndex = snap.AdcPeriodMs switch
            {
                20 => 1,
                50 => 2,
                100 => 3,
                _ => 0
            };
            ChkEnable.IsChecked = snap.AdcEnabled;
        }

        private void SetAvailable(bool available, string detail = "")
        {
            BtnApply.IsEnabled = available;
            BtnRead.IsEnabled = available;

            if (available)
            {
                TxtFooter.Text = "streaming from the STM32 SPI link";
                return;
            }

            _configSeeded = false;
            TxtFooter.Text = string.IsNullOrEmpty(detail) ? "not connected" : detail;
            TxtStatus.Text = "no data";
            TxtStatus.Foreground = Theme.TextDimBrush;
            for (int i = 0; i < _raw.Length; i++)
            {
                _raw[i].Text = "---";
                _mv[i].Text = "---";
                _bars[i].Value = 0;
            }
        }

        private void OnApplyClick(object sender, System.Windows.RoutedEventArgs e)
        {
            bool enable = ChkEnable.IsChecked == true;
            const byte bus = 1;   // ADC slave is on SPI1 only; SPI3 is PS2-only
            byte cs = (byte)Math.Clamp(CmbCs.SelectedIndex, 0, 2);
            byte[] periods = { 10, 20, 50, 100 };
            byte period = periods[Math.Clamp(CmbPeriod.SelectedIndex, 0, periods.Length - 1)];
            ApplyRequested?.Invoke(enable, bus, cs, period);
        }

        private void OnReadClick(object sender, System.Windows.RoutedEventArgs e)
        {
            ReadRequested?.Invoke();
        }
    }
}
