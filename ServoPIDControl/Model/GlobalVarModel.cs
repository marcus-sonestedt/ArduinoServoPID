using System.ComponentModel;
using System.Runtime.CompilerServices;
using ServoPIDControl.Annotations;

namespace ServoPIDControl.Model
{
    public class GlobalVarModel : INotifyPropertyChanged
    {
        private float _value;

        public GlobalVarModel(GlobalVar var)
        {
            Var = var;
        }

        public GlobalVar Var { get; }

        public float Value
        {
            get => _value;
            set
            {
                if (value.Equals(_value)) return;
                _value = value;
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
