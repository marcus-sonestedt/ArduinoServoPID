using System.ComponentModel;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using ServoPIDControl.Annotations;

namespace ServoPIDControl.Model
{
    public class GlobalVarModel : INotifyPropertyChanged
    {
        private float _value;

        public GlobalVarModel(GlobalVar variable)
        {
            Variable = variable;
        }

        public GlobalVar Variable { get; }

        public float Value
        {
            [DebuggerStepThrough]
            get => _value;
            [DebuggerStepThrough]
            set
            {
                if (value.Equals(_value)) return;
                _value = value;
                OnPropertyChanged();
            }
        }

        public event PropertyChangedEventHandler PropertyChanged;

        [NotifyPropertyChangedInvocator]
        [DebuggerStepThrough]
        protected virtual void OnPropertyChanged([CallerMemberName] string propertyName = null)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }

        [DebuggerStepThrough]
        public override string ToString() => $"{Variable}: {Value}";
    }
}
