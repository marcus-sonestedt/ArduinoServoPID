using System.ComponentModel;
using System.Linq;
using System.Windows;
using NLog;
using NLog.Config;
using NLog.Targets.Wrappers;
using ServoPIDControl.Annotations;
using ServoPIDControl.Helper;

namespace ServoPIDControl
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    [UsedImplicitly]
    public partial class MainWindow
    {
        private static readonly Logger Log = LogManager.GetCurrentClassLogger();
        private readonly ArduinoCom _arduinoCom = new ArduinoCom();

        public MainWindow()
        {
            InitializeComponent();

            Loaded += OnLoaded;
            Unloaded += OnUnloaded;

            Model.PropertyChanged += ModelOnPropertyChanged;
        }

        private void ModelOnPropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            if (e.PropertyName == nameof(Model.ComPorts) && Model.ConnectedPort == null && Model.ComPorts != null)
                Model.ConnectedPort = Model.ComPorts.FirstOrDefault();
        }

        private void OnLoaded(object sender, RoutedEventArgs e)
        {
            var wpfTarget = new WpfRichTextBoxTarget
            {
                ControlName = LogBox.Name,
                FormName = GetType().Name,
                MaxLines = 500,
                UseDefaultRowColoringRules = true,
                AutoScroll = true,
                // ReSharper disable StringLiteralTypo
                Layout = "${processtime} [${level:uppercase=true}] " +
                         "${logger:shortName=true}: ${message}" +
                    "${exception:innerFormat=tostring:maxInnerExceptionLevel=10:separator=,:format=tostring}",
                // ReSharper restore StringLiteralTypo
            };

            var asyncWrapper = new AsyncTargetWrapper {Name = "RichTextAsync", WrappedTarget = wpfTarget};
            LogManager.Configuration.AddTarget(asyncWrapper.Name, asyncWrapper);
            LogManager.Configuration.LoggingRules.Insert(0, new LoggingRule("*", LogLevel.Info, asyncWrapper));
            LogManager.ReconfigExistingLoggers();

            _arduinoCom.Model = Model;

            if (Model.ConnectedPort == null && Model.ComPorts != null)
                Model.ConnectedPort = Model.ComPorts.FirstOrDefault();

            Log.Info("Ready!\r\n");
        }

        private void OnUnloaded(object sender, RoutedEventArgs e)
        {
            _arduinoCom.Model = null;
        }
    }
}