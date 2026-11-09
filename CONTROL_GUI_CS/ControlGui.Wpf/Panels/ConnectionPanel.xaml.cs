using System;
using System.IO.Ports;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using ControlGui.Core.Paxini;

namespace ControlGui.Wpf.Panels
{
    public partial class ConnectionPanel : UserControl
    {
        private static readonly int[] BaudRates = new int[] { 1000000, 921600, 500000, 250000, 115200 };

        public event Action<string, string, int>? ConnectRequested;
        public event Action<string>? DisconnectRequested;
        public event Action? ConnectAllRequested;
        public event Action? DisconnectAllRequested;

        public ConnectionPanel()
        {
            InitializeComponent();
            PopulateBauds();
            RefreshPorts();
        }

        private void PopulateBauds()
        {
            foreach (var b in BaudRates)
            {
                CmbMotorBaud.Items.Add(b);
                CmbSensorLBaud.Items.Add(b);
                CmbSensorRBaud.Items.Add(b);
            }
            CmbMotorBaud.SelectedItem = 1000000;
            CmbSensorLBaud.SelectedItem = PaxiniConstants.DefaultBaudRate;
            CmbSensorRBaud.SelectedItem = PaxiniConstants.DefaultBaudRate;
        }

        public void RefreshPorts()
        {
            string[] ports;
            try
            {
                ports = SerialPort.GetPortNames();
            }
            catch
            {
                ports = Array.Empty<string>();
            }

            var all = ports.ToList();
            all.Add("MOCK_PORT"); // Always provide mock simulation option

            UpdateCombo(CmbMotorPort, all);
            UpdateCombo(CmbSensorLPort, all);
            UpdateCombo(CmbSensorRPort, all);
        }

        private static void UpdateCombo(ComboBox cmb, System.Collections.Generic.List<string> items)
        {
            string? sel = cmb.SelectedItem?.ToString();
            cmb.Items.Clear();
            foreach (var item in items) cmb.Items.Add(item);

            if (sel != null && cmb.Items.Contains(sel))
            {
                cmb.SelectedItem = sel;
            }
            else if (cmb.Items.Count > 0)
            {
                cmb.SelectedIndex = 0;
            }
        }

        public (string port, int baud) GetRoleConfig(string role)
        {
            return role switch
            {
                "motor" => (CmbMotorPort.SelectedItem?.ToString() ?? "", (int)(CmbMotorBaud.SelectedItem ?? 1000000)),
                "sensor_l" => (CmbSensorLPort.SelectedItem?.ToString() ?? "", (int)(CmbSensorLBaud.SelectedItem ?? 921600)),
                "sensor_r" => (CmbSensorRPort.SelectedItem?.ToString() ?? "", (int)(CmbSensorRBaud.SelectedItem ?? 921600)),
                _ => ("", 115200)
            };
        }

        private void OnMotorConnectClick(object sender, RoutedEventArgs e)
        {
            if (BtnMotorConnect.Content.ToString() == "Connect")
            {
                var (port, baud) = GetRoleConfig("motor");
                ConnectRequested?.Invoke("motor", port, baud);
            }
            else
            {
                DisconnectRequested?.Invoke("motor");
            }
        }

        private void OnSensorLConnectClick(object sender, RoutedEventArgs e)
        {
            if (BtnSensorLConnect.Content.ToString() == "Connect")
            {
                var (port, baud) = GetRoleConfig("sensor_l");
                ConnectRequested?.Invoke("sensor_l", port, baud);
            }
            else
            {
                DisconnectRequested?.Invoke("sensor_l");
            }
        }

        private void OnSensorRConnectClick(object sender, RoutedEventArgs e)
        {
            if (BtnSensorRConnect.Content.ToString() == "Connect")
            {
                var (port, baud) = GetRoleConfig("sensor_r");
                ConnectRequested?.Invoke("sensor_r", port, baud);
            }
            else
            {
                DisconnectRequested?.Invoke("sensor_r");
            }
        }

        private void OnConnectAllClick(object sender, RoutedEventArgs e) => ConnectAllRequested?.Invoke();
        private void OnDisconnectAllClick(object sender, RoutedEventArgs e) => DisconnectAllRequested?.Invoke();
        private void OnRefreshPortsClick(object sender, RoutedEventArgs e) => RefreshPorts();

        public void SetRoleStatus(string role, bool connected, bool connecting, string statusText, string state = "off")
        {
            if (role == "motor")
            {
                BtnMotorConnect.Content = connected ? "Disconnect" : "Connect";
                CmbMotorPort.IsEnabled = !connected;
                CmbMotorBaud.IsEnabled = !connected;
                LedMotor.State = state;
                TxtMotorStatus.Text = statusText;
            }
            else if (role == "sensor_l")
            {
                BtnSensorLConnect.Content = connected ? "Disconnect" : "Connect";
                CmbSensorLPort.IsEnabled = !connected;
                CmbSensorLBaud.IsEnabled = !connected;
                LedSensorL.State = state;
                TxtSensorLStatus.Text = statusText;
            }
            else if (role == "sensor_r")
            {
                BtnSensorRConnect.Content = connected ? "Disconnect" : "Connect";
                CmbSensorRPort.IsEnabled = !connected;
                CmbSensorRBaud.IsEnabled = !connected;
                LedSensorR.State = state;
                TxtSensorRStatus.Text = statusText;
            }
        }
    }
}
