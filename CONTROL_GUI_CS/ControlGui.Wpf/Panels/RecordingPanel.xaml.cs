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
            TxtOutputDir.Text = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Unified_Control", "recordings.db");
        }

        public void SetDefaultDirs(string workspaceDir)
        {
            if (workspaceDir.EndsWith(".db", StringComparison.OrdinalIgnoreCase) ||
                workspaceDir.EndsWith(".sqlite", StringComparison.OrdinalIgnoreCase))
            {
                TxtOutputDir.Text = workspaceDir;
            }
            else
            {
                TxtOutputDir.Text = Path.Combine(workspaceDir, "recordings.db");
            }
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

        public void SetRecordingState(bool recording, double elapsedSeconds, long rows, string dbPath, string currentTable)
        {
            _isRecording = recording;

            if (recording)
            {
                BtnRecord.Content = "■  Stop Recording";
                BtnRecord.Background = Theme.DangerBrush;
                BtnRecord.Foreground = Brushes.White;
                TxtFiles.Text = $"Active DB: {Path.GetFileName(dbPath)}  |  Table: [{currentTable}]";
                TxtFiles.Foreground = Theme.SuccessBrush;
            }
            else
            {
                BtnRecord.Content = "●  Start Recording";
                BtnRecord.Background = Theme.PrimaryBrush;
                BtnRecord.Foreground = Brushes.White;
                TxtFiles.Text = $"Target SQLite DB: {Path.GetFileName(dbPath)} (idle)";
                TxtFiles.Foreground = Theme.TextDimBrush;
            }

            TimeSpan ts = TimeSpan.FromSeconds(elapsedSeconds);
            TxtElapsed.Text = $"{ts.Hours:D2}:{ts.Minutes:D2}:{ts.Seconds:D2}";
            TxtRows.Text = $"Rows logged: {rows}";
        }

        private void OnWorkspaceClick(object sender, RoutedEventArgs e)
        {
            string path = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Unified_Control", "recordings.db");
            TxtOutputDir.Text = path;
            OutputDirChanged?.Invoke(path);
        }

        private void OnDesktopClick(object sender, RoutedEventArgs e)
        {
            string path = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.Desktop), "Unified_Control", "recordings.db");
            TxtOutputDir.Text = path;
            OutputDirChanged?.Invoke(path);
        }

        private void OnBrowseClick(object sender, RoutedEventArgs e)
        {
            var dlg = new SaveFileDialog
            {
                Title = "Select or Name SQLite Database File",
                Filter = "SQLite Database (*.db)|*.db|SQLite 3 Database (*.sqlite)|*.sqlite|All Files (*.*)|*.*",
                FileName = "recordings.db"
            };
            if (dlg.ShowDialog() == true)
            {
                TxtOutputDir.Text = dlg.FileName;
                OutputDirChanged?.Invoke(dlg.FileName);
            }
        }
    }
}
