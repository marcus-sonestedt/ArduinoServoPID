using System;
using System.Linq;
using System.Windows;
using System.Windows.Threading;

namespace ServoPIDControl
{
    /// <summary>
    /// Interaction logic for App.xaml
    /// </summary>
    public partial class App : Application
    {
        public App()
        {
            DispatcherUnhandledException += OnDispatcherUnhandledException;
            AppDomain.CurrentDomain.UnhandledException += CurrentDomainOnUnhandledException;

            MockMode = Environment.GetCommandLineArgs().Contains("--mock");

        }

        public bool MockMode { get; set; }

        private static void CurrentDomainOnUnhandledException(object sender, UnhandledExceptionEventArgs e)
        {
            var ex = (Exception) e.ExceptionObject;
            MessageBox.Show(ex.Message, ex.GetType().Name);
        }

        private static void OnDispatcherUnhandledException(object sender, DispatcherUnhandledExceptionEventArgs e)
        {
            MessageBox.Show(e.Exception.Message, e.Exception.GetType().Name);
        }
    }
}
