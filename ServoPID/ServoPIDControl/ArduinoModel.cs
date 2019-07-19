using System;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.IO.Ports;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Windows.Threading;
using ServoPIDControl.Annotations;

namespace ServoPIDControl
{
    public class ArduinoModel : INotifyPropertyChanged, IDisposable
    {
        private string _portName = "COM3";
        private bool _enabled = true;
        private float _deltaTime;
        private bool _connected;
        private bool _pollPidData;
        private string[] _comPorts;

        private readonly DispatcherTimer _timer;

        public event PropertyChangedEventHandler PropertyChanged;

        public ArduinoModel()
        {
            for (var i = 0; i < 4; ++i)
                Servos.Add(new ServoPidModel(i));

            _timer = new DispatcherTimer {Interval = TimeSpan.FromSeconds(1), IsEnabled = true};
            _timer.Tick += (s, a) => ComPorts = SerialPort.GetPortNames().Distinct().ToArray();
        }

        public string PortName
        {
            get => _portName;
            set
            {
                if (value == _portName) return;
                _portName = value;
                OnPropertyChanged();
            }
        }

        public bool Enabled
        {
            get => _enabled;
            set
            {
                if (value == _enabled) return;
                _enabled = value;
                OnPropertyChanged();
            }
        }

        public ObservableCollection<ServoPidModel> Servos { get; } = new ObservableCollection<ServoPidModel>();

        public float DeltaTime
        {
            get => _deltaTime;
            internal set
            {
                if (value.Equals(_deltaTime)) return;
                _deltaTime = value;
                OnPropertyChanged();
            }
        }

        public bool PollPidData
        {
            get => _pollPidData;
            set
            {
                if (value == _pollPidData) return;
                _pollPidData = value;
                OnPropertyChanged();
            }
        }

        public string[] ComPorts
        {
            get => _comPorts;
            internal set
            {
                if (Equals(value, _comPorts) ||
                    value != null && _comPorts != null && value.SequenceEqual(_comPorts))
                    return;

                _comPorts = value;
                OnPropertyChanged();
            }
        }

        public bool Connected
        {
            get => _connected;
            internal set
            {
                if (value == _connected) return;
                _connected = value;
                OnPropertyChanged();
            }
        }

        [NotifyPropertyChangedInvocator]
        protected virtual void OnPropertyChanged([CallerMemberName] string propertyName = null)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }

        public void Dispose()
        {
            _timer.IsEnabled = false;
        }
    }
}