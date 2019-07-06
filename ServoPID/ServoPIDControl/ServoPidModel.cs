using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;
using ServoPIDControl.Annotations;

namespace ServoPIDControl
{
    public class ServoPidModel : INotifyPropertyChanged
    {
        private float _p;
        private float _i;
        private float _d;
        private float _dLambda = 1;
        private float _setPoint;
        private float _inputScale = 1;
        private float _inputBias;

        public ServoPidModel(int id)
        {
            Id = id;
        }

        public int Id { get;  }

        public float P
        {
            get => _p;
            set
            {
                if (value.Equals(_p)) return;
                _p = value;
                OnPropertyChanged();
            }
        }

        public float I
        {
            get => _i;
            set
            {
                if (value.Equals(_i)) return;
                _i = value;
                OnPropertyChanged();
            }
        }

        public float D
        {
            get => _d;
            set
            {
                if (value.Equals(_d)) return;
                _d = value;
                OnPropertyChanged();
            }
        }

        public float DLambda
        {
            get => _dLambda;
            set
            {
                if (value.Equals(_dLambda)) return;
                _dLambda = value;
                OnPropertyChanged();
            }
        }

        public float SetPoint
        {
            get => _setPoint;
            set
            {
                if (value.Equals(_setPoint)) return;
                _setPoint = value;
                OnPropertyChanged();
            }
        }

        public float Input { get; set; }
        public float Output { get; set; }

        public float Integrator { get; set; }
        public float DFiltered { get; set; }

        public float InputScale
        {
            get => _inputScale;
            set
            {
                if (value.Equals(_inputScale)) return;
                _inputScale = value;
                OnPropertyChanged();
            }
        }

        public float InputBias
        {
            get => _inputBias;
            set
            {
                if (value.Equals(_inputBias)) return;
                _inputBias = value;
                OnPropertyChanged();
            }
        }

        public event PropertyChangedEventHandler PropertyChanged;

        [NotifyPropertyChangedInvocator]
        protected virtual void OnPropertyChanged([CallerMemberName] string propertyName = null)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }
    }
}