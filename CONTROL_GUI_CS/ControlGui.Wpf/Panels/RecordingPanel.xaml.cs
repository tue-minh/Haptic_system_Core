using System;
using System.IO;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using Microsoft.Win32;

namespace ControlGui.Wpf.Panels
{
    public partial class RecordingPanel : UserControl
    {
        public event Action? StartRequested;
        public event Action? StopRequested;
        public event Action<string>? OutputDirChanged;

        private bool _isRecording = false;

        public string OutputDirectory => TxtOutputDir.Text.Trim();
        public double SampleRateHz => double.TryParse(TxtSampleRate.Text, out var r) ? r : 100.0;

        public RecordingPanel()
        {
            InitializeComponent();
            TxtOutputDir.Text = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Unified_Control");
        }

        public void SetDefaultDirs(string workspaceDir)
        {
            TxtOutputDir.Text = workspaceDir;
        }

        private void OnToggleRecordClick(object sender, RoutedEventArgs e)
        {
            if (!_isRecording)
            {
                StartRequested?.Invoke();
            }
            else
            {
                StopRequested?.Invoke();
            }
        }

        public void SetRecordingState(bool recording, double elapsedSeconds, long rows, string filePath)
        {
            _isRecording = recording;

            if (recording)
            {
                BtnRecord.Content = "■  Stop Recording";
                BtnRecord.Background = Theme.DangerBrush;
                BtnRecord.Foreground = Brushes.White;
            }
            else
            {
                BtnRecord.Content = "●  Start Recording";
                BtnRecord.Background = Theme.PrimaryBrush;
                BtnRecord.Foreground = Brushes.White;
            }

            TimeSpan ts = TimeSpan.FromSeconds(elapsedSeconds);
            TxtElapsed.Text = $"{ts.Hours:D2}:{ts.Minutes:D2}:{ts.Seconds:D2}";
            TxtRows.Text = $"Rows logged: {rows}";
            TxtFiles.Text = string.IsNullOrEmpty(filePath) ? "No active recording" : Path.GetFileName(filePath);
        }

        private void OnWorkspaceClick(object sender, RoutedEventArgs e)
        {
            string dir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Unified_Control");
            TxtOutputDir.Text = dir;
            OutputDirChanged?.Invoke(dir);
        }

        private void OnDesktopClick(object sender, RoutedEventArgs e)
        {
            string dir = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.Desktop), "Unified_Control");
            TxtOutputDir.Text = dir;
            OutputDirChanged?.Invoke(dir);
        }

        private void OnBrowseClick(object sender, RoutedEventArgs e)
        {
            var dlg = new OpenFolderDialog
            {
                Title = "Select Recording Output Directory"
            };
            if (dlg.ShowDialog() == true)
            {
                TxtOutputDir.Text = dlg.FolderName;
                OutputDirChanged?.Invoke(dlg.FolderName);
            }
        }
    }
}
