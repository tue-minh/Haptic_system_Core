using System;
using System.IO;
using System.Windows;
using ControlGui.Wpf.Windows;

namespace ControlGui.Wpf;

/// <summary>
/// Interaction logic for App.xaml
/// </summary>
public partial class App : Application
{
    public App()
    {
        AppDomain.CurrentDomain.UnhandledException += (s, args) =>
        {
            string log = $"AppDomain UnhandledException:\n{args.ExceptionObject}";
            try { File.WriteAllText(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "crash.log"), log); } catch { }
            MessageBox.Show(log, "Fatal Error", MessageBoxButton.OK, MessageBoxImage.Error);
        };

        DispatcherUnhandledException += (s, args) =>
        {
            string log = $"DispatcherUnhandledException:\n{args.Exception}";
            try { File.WriteAllText(Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "crash.log"), log); } catch { }
            MessageBox.Show(log, "Dispatcher Error", MessageBoxButton.OK, MessageBoxImage.Error);
            args.Handled = true;
        };
    }

    protected override void OnStartup(StartupEventArgs e)
    {
        base.OnStartup(e);

        bool bandwidthOnly = Array.Exists(
            e.Args,
            a => string.Equals(a, "--bandwidth", StringComparison.OrdinalIgnoreCase));

        bool gripperTransition = Array.Exists(
            e.Args,
            a => string.Equals(a, "--gripper-transition", StringComparison.OrdinalIgnoreCase));

        Window win;
        if (bandwidthOnly)
        {
            win = new BandwidthExperimentWindow(() => null) { WindowStartupLocation = WindowStartupLocation.CenterScreen };
        }
        else if (gripperTransition)
        {
            // Standalone experiment GUI: no main window, it manages its own
            // STM32 + two PaXini connections.
            win = new GripperTransitionWindow(() => null, () => null, () => null)
            {
                WindowStartupLocation = WindowStartupLocation.CenterScreen
            };
        }
        else
        {
            win = new MainWindow();
        }

        MainWindow = win;
        win.Show();
    }
}
