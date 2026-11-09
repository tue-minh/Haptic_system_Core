using System;
using System.IO;
using System.Windows;

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
}
