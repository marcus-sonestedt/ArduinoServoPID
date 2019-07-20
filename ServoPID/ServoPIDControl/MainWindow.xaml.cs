using System.ComponentModel;
using System.Linq;
using System.Windows;
using ServoPIDControl.Annotations;

namespace ServoPIDControl
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    [UsedImplicitly]
    public partial class MainWindow : Window
    {
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
            if (e.PropertyName == nameof(Model.ComPorts) && Model.PortName == null && Model.ComPorts != null)
                Model.PortName = Model.ComPorts.FirstOrDefault();
        }

        private void OnLoaded(object sender, RoutedEventArgs e)
        {
            _arduinoCom.Model = Model;

            if (Model.PortName == null && Model.ComPorts != null)
                Model.PortName = Model.ComPorts.FirstOrDefault();
        }

        private void OnUnloaded(object sender, RoutedEventArgs e)
        {
            _arduinoCom.Model = null;
        }

    }
}
