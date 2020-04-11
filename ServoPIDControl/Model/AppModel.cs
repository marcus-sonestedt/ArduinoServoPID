using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Windows.Threading;
using NLog;
using JetBrains.Annotations;
using Ports = System.IO.Ports;

namespace ServoPIDControl.Model
{
    public sealed class AppModel : INotifyPropertyChanged, IDisposable
    {
        public const int DefaultNumServos = 1;

        private static readonly Logger Log = LogManager.GetCurrentClassLogger();

        private string _connectedPort;
        private float _deltaTime;
        private bool _connected;
        private bool _pollPidData;
        private ReadOnlyCollection<string> _comPorts;

        private readonly DispatcherTimer _timer;
        private float _minDt;
        private float _maxDt;
        private ServoPidModel _currentGraphServo;

        public event PropertyChangedEventHandler PropertyChanged;

        public AppModel()
        {
            for (var i = 0; i < DefaultNumServos; ++i)
                Servos.Add(new ServoPidModel(i));

            GlobalVar = GlobalVars.ToDictionary(gv => gv.Variable, gv => gv);
            GlobalVar[ServoPIDControl.GlobalVar.PidEnabled].PropertyChanged +=
                (s, a) => OnPropertyChanged(nameof(PidEnabled));

            _timer = new DispatcherTimer {Interval = TimeSpan.FromSeconds(1), IsEnabled = true};
            _timer.Tick += UpdateComPorts;
        }

        private void UpdateComPorts(object s, EventArgs a)
        {
            ComPorts = new ReadOnlyCollection<string>(
                Ports.SerialPort.GetPortNames()
                    .Distinct()
                    .Append("Mock")
                    .Append("Simulator")
                    .ToList()
            );
        }

        public string ConnectedPort
        {
            get => _connectedPort;
            set
            {
                if (value == _connectedPort) return;
                _connectedPort = value;
                OnPropertyChanged();
                Log.Info($"{nameof(ConnectedPort)}: {value}");
            }
        }

        public bool PidEnabled
        {
            // ReSharper disable once CompareOfFloatsByEqualityOperator
            get => GlobalVar[ServoPIDControl.GlobalVar.PidEnabled].Value != 0;
            set => GlobalVar[ServoPIDControl.GlobalVar.PidEnabled].Value = value ? 1 : 0;
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

        public ReadOnlyCollection<string> ComPorts
        {
            get => _comPorts;
            internal set
            {
                if (Equals(value, _comPorts) ||
                    value != null && _comPorts != null && value.SequenceEqual(_comPorts))
                    return;

                _comPorts = value;
                OnPropertyChanged();

                Log.Info(value == null
                    ? $"{nameof(ComPorts)}: ''"
                    : $"{nameof(ComPorts)}: '{string.Join(", ", value)}'");
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
                Log.Info($"{nameof(Connected)}: {value}");
            }
        }

        public float MinDt
        {
            get => _minDt;
            set
            {
                if (value.Equals(_minDt)) return;
                _minDt = value;
                OnPropertyChanged();
            }
        }

        public float MaxDt
        {
            get => _maxDt;
            set
            {
                if (value.Equals(_maxDt)) return;
                _maxDt = value;
                OnPropertyChanged();
            }
        }

        public ServoPidModel CurrentGraphServo
        {
            get => _currentGraphServo;
            set
            {
                if (value == _currentGraphServo) return;
                _currentGraphServo = value;
                OnPropertyChanged();
            }
        }

        public IReadOnlyList<GlobalVarModel> GlobalVars { get; } =
            Enum.GetValues(typeof(GlobalVar))
                .Cast<GlobalVar>()
                .Select(v => new GlobalVarModel(v))
                .ToList();

        public IDictionary<GlobalVar, GlobalVarModel> GlobalVar { get; } 

        [NotifyPropertyChangedInvocator]
        private void OnPropertyChanged([CallerMemberName] string propertyName = null)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }

        public void Dispose()
        {
            _timer.IsEnabled = false;
        }
    }
}