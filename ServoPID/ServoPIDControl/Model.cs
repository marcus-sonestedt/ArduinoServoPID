using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Runtime.CompilerServices;
using ServoPIDControl.Annotations;

namespace ServoPIDControl
{
    public class Model : INotifyPropertyChanged
    {
        private string _portName = "COM3";
        private bool _enabled = true;
        private float _deltaTime;

        public event PropertyChangedEventHandler PropertyChanged;

        public Model()
        {
            for(var i = 0; i < 4; ++i)
                Servos.Add(new ServoPidModel(i));
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
            set
            {
                if (value.Equals(_deltaTime)) return;
                _deltaTime = value;
                OnPropertyChanged();
            }
        }

        [NotifyPropertyChangedInvocator]
        protected virtual void OnPropertyChanged([CallerMemberName] string propertyName = null)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }
    }
}
