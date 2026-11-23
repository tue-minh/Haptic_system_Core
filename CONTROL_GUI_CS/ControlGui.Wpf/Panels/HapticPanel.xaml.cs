using System;
using System.Windows;
using System.Windows.Controls;
using ControlGui.Core.Haptic;

namespace ControlGui.Wpf.Panels
{
    public partial class HapticPanel : UserControl
    {
        public event Action<bool>? EnabledChanged;
        public event Action? PingRequested;

        public bool IsStreamingEnabled => ChkEnable?.IsChecked == true;

        public HapticPanel()
        {
            InitializeComponent();
        }

        private void OnEnableToggled(object sender, RoutedEventArgs e)
        {
            EnabledChanged?.Invoke(IsStreamingEnabled);
        }

        private void OnPingEsp32Click(object sender, RoutedEventArgs e)
        {
            PingRequested?.Invoke();
        }

        public void UpdateStatus(HapticSnapshot? snap, bool isStreaming, bool motorConnected)
        {
            BtnPingEsp32.IsEnabled = motorConnected;

            if (!IsStreamingEnabled)
            {
                TxtState.Text = "Off";
                return;
            }

            if (!motorConnected)
            {
                TxtState.Text = "Waiting for the STM32 link";
                return;
            }

            if (isStreaming)
            {
                TxtState.Text = $"Active (100 Hz, {snap?.Sent} frames sent, {snap?.Errors} errors)";
            }
            else
            {
                TxtState.Text = "Standby (ready)";
            }
        }

        public void SetPingResult(string text, bool ok, bool inProgress)
        {
            if (inProgress)
            {
                TxtPingResult.Text = "ESP32 ping: waiting for response...";
                TxtPingResult.Foreground = Theme.TextDimBrush;
            }
            else
            {
                TxtPingResult.Text = text;
                TxtPingResult.Foreground = ok ? Theme.SuccessBrush : Theme.DangerBrush;
            }
        }
    }
}
