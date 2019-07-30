using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Linq;
using System.Runtime.CompilerServices;
using InteractiveDataDisplay.WPF;
using ServoPIDControl.Annotations;

namespace ServoPIDControl.Model
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
        private float _input;
        private float _output;
        private float _integrator;
        private float _dFiltered;

        public ServoPidModel(int id)
        {
            Id = id;

#if DEBUG
            Times = new ObservableCollection<float>(Enumerable.Range(0, 500).Select(i => i / 100.0f));
            SetPoints = new ObservableCollection<float>(Enumerable.Range(id * 100, 500)
                .Select(i => i / 100 % 2 == 0 ? 80.0f : 100.0f));
            Inputs = new ObservableCollection<float>(Enumerable.Range(id * 100, 500)
                .Select(i => 90 + (float) Math.Sin(i / 50.0f)));
            Outputs = new ObservableCollection<float>(Enumerable.Range(id * 100, 500)
                .Select(i => 90 + (float) Math.Cos(i / 50.0f)));
#endif
        }

        public int Id { get; }

        public float  P
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

        public float Input
        {
            get => _input;
            internal set
            {
                if (value.Equals(_input)) return;
                _input = value;
                OnPropertyChanged();
            }
        }

        public float Output
        {
            get => _output;
            internal set
            {
                if (value.Equals(_output)) return;
                _output = value;
                OnPropertyChanged();
            }
        }

        public float Integrator
        {
            get => _integrator;
            internal set
            {
                if (value.Equals(_integrator)) return;
                _integrator = value;
                OnPropertyChanged();
            }
        }

        public float DFiltered
        {
            get => _dFiltered;
            internal set
            {
                if (value.Equals(_dFiltered)) return;
                _dFiltered = value;
                OnPropertyChanged();
            }
        }

        // ReSharper disable MemberInitializerValueIgnored
        public ObservableCollection<float> Times { get; } = new ObservableCollection<float>();
        public ObservableCollection<float> SetPoints { get; } = new ObservableCollection<float>();
        public ObservableCollection<float> Inputs { get; } = new ObservableCollection<float>();
        public ObservableCollection<float> Outputs { get; } = new ObservableCollection<float>();
        // ReSharper restore MemberInitializerValueIgnored

        public struct TimeSeries
        {
            public ObservableCollection<float> X;
            public ObservableCollection<float> Y;
            public string Name;
        }

        public IEnumerable<TimeSeries> AllTimeSeries
        {
            get
            {
                yield return new TimeSeries {X = Times, Y = SetPoints, Name = "SetPoint"};
                yield return new TimeSeries {X = Times, Y = Inputs, Name = "Input"};
                yield return new TimeSeries {X = Times, Y = Outputs, Name = "Output"};
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