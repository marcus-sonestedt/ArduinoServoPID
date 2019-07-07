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
        }

        private void OnLoaded(object sender, RoutedEventArgs e)
        {
            _arduinoCom.Model = Model;
        }

        private void OnUnloaded(object sender, RoutedEventArgs e)
        {
            _arduinoCom.Model = null;
        }

    }
}
