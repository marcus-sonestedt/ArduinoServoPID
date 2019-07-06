using System;
using System.Collections.Specialized;
using System.ComponentModel;
using System.Diagnostics;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Windows.Threading;

namespace ServoPIDControl
{
    internal enum Command : byte
    {
        NoOp,
        SetServoParamFloat,
        EnableRegulator,
        GetNumServos,
        GetServoParams,
        GetServoData
    };

    internal enum ServoParam : byte
    {
        None,
        P,
        I,
        D,
        DLambda,
        SetPoint,
        InputScale,
        InputBias
    };


    public class ArduinoCom : IDisposable
    {
        private SerialPort _port;
        private readonly StringBuilder _readBuf = new StringBuilder();
        private Model _model;
        private readonly DispatcherTimer _timer = new DispatcherTimer() {Interval = TimeSpan.FromMilliseconds(10)};
        private readonly object _portLock = new object();

        public ArduinoCom()
        {
            _timer.Tick += TimerOnTick;
            _timer.Start();
        }

        private void TimerOnTick(object sender, EventArgs e)
        {
            if ((!_port?.IsOpen ?? false) || Model == null)
                return;

            SendCommand(Command.GetServoData);
        }

        public class StringEventArgs : EventArgs
        {
            public string Message;
        }

        public event EventHandler<StringEventArgs> MessageReceived;

        private void PortOnDataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            lock (_portLock)
            {
                _readBuf.Append(_port.ReadExisting());
            }

            while (_readBuf.ToString().Contains('\n'))
            {
                Debug.WriteLine(_readBuf.ToString());

                var lines = _readBuf.ToString().Split('\n');
                foreach (var line in lines)
                    LineReceived(line);

                _readBuf.Clear();
            }
        }

        private void LineReceived(string line)
        {
            if (line.StartsWith("NS "))
            {
                Model.Servos.Clear();

                var numServos = int.Parse(line.Substring(3));
                for (var i = 0; i < numServos; ++i)
                    Model.Servos.Add(new ServoPidModel(Model.Servos.Count));

                return;
            }

            if (line.StartsWith("SP "))
            {
                var parts = line.Split('\n');
                var servoId = int.Parse(parts[1]);
                var servo = Model.Servos[servoId];
                servo.P = float.Parse(parts[2]);
                servo.I = float.Parse(parts[3]);
                servo.D = float.Parse(parts[4]);
                servo.DLambda = float.Parse(parts[5]);
                servo.SetPoint = float.Parse(parts[6]);
                return;
            }

            if (line.StartsWith("SD "))
            {
                var parts = line.Split('\n');
                var servoId = int.Parse(parts[1]);
                var servo = Model.Servos[servoId];
                servo.Input = float.Parse(parts[2]);
                servo.Output = float.Parse(parts[3]);
                servo.Integrator = float.Parse(parts[4]);
                servo.DFiltered = float.Parse(parts[5]);
                return;
            }

            if (line.StartsWith("DT "))
            {
                var dt = float.Parse(line.Substring(3));
                Model.DeltaTime = dt;
                return;
            }

            MessageReceived?.Invoke(this, new StringEventArgs {Message = line});
        }

        public Model Model
        {
            get => _model;
            set
            {
                if (Model != null)
                {
                    Model.PropertyChanged -= ModelOnPropertyChanged;
                    Model.Servos.CollectionChanged -= ServosOnCollectionChanged;

                    foreach (var s in Model.Servos)
                        s.PropertyChanged -= ServoOnPropertyChanged;
                }

                _model = value;

                // ReSharper disable once InvertIf
                if (Model != null)
                {
                    Model.PropertyChanged += ModelOnPropertyChanged;
                    Model.Servos.CollectionChanged += ServosOnCollectionChanged;

                    foreach (var s in Model.Servos)
                        s.PropertyChanged += ServoOnPropertyChanged;
                }

                ConnectPort();
            }
        }

        private void ServoOnPropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            var servo = (ServoPidModel) sender;
            switch (e.PropertyName)
            {
                case nameof(ServoPidModel.P):
                    SendServoParam(servo.Id, ServoParam.P, servo.P);
                    break;
                case nameof(ServoPidModel.I):
                    SendServoParam(servo.Id, ServoParam.I, servo.I);
                    break;
                case nameof(ServoPidModel.D):
                    SendServoParam(servo.Id, ServoParam.D, servo.D);
                    break;
                case nameof(ServoPidModel.DLambda):
                    SendServoParam(servo.Id, ServoParam.DLambda, servo.DLambda);
                    break;
                case nameof(ServoPidModel.SetPoint):
                    SendServoParam(servo.Id, ServoParam.SetPoint, servo.SetPoint);
                    break;
                case nameof(ServoPidModel.InputScale):
                    SendServoParam(servo.Id, ServoParam.InputScale, servo.InputScale);
                    break;
                case nameof(ServoPidModel.InputBias):
                    SendServoParam(servo.Id, ServoParam.InputBias, servo.InputBias);
                    break;
            }
        }

        private void ServosOnCollectionChanged(object sender, NotifyCollectionChangedEventArgs e)
        {
            foreach (var servo in e.OldItems.Cast<ServoPidModel>())
                servo.PropertyChanged -= ServoOnPropertyChanged;

            foreach (var servo in e.NewItems.Cast<ServoPidModel>())
                servo.PropertyChanged += ServoOnPropertyChanged;

            if (e.Action == NotifyCollectionChangedAction.Reset)
                throw new NotImplementedException("Servo collection was reset");
        }

        private void ModelOnPropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            switch (e.PropertyName)
            {
                case nameof(Model.Enabled):
                    SendCommand(Command.EnableRegulator, (byte) (Model.Enabled ? 1 : 0));
                    break;

                case nameof(Model.PortName):
                    ConnectPort();
                    break;
            }
        }

        private void SendCommand(Command cmd, params byte[] data)
        {
            var cmdData = new[] {(byte) (data.Length + 2), (byte) cmd}.Concat(data).ToArray();

            lock (_portLock)
            {
                _port.Write(cmdData, 0, cmdData.Length);
            }
        }

        private void SendServoParam(int servoId, ServoParam servoParam, float value)
        {
            SendCommand(Command.SetServoParamFloat,
                new[] {(byte) servoId, (byte) servoParam}
                    .Concat(BitConverter.GetBytes(value))
                    .ToArray());
        }


        private void ConnectPort()
        {
            lock (_portLock)
            {
                if (_port != null)
                {
                    _port.DataReceived -= PortOnDataReceived;
                    _port.Close();
                    _port.Dispose();
                    _port = null;
                }

                if (Model == null)
                    return;

                try
                {
                    _port = new SerialPort(Model.PortName);
                    _port.Open();
                    _port.DataReceived += PortOnDataReceived;
                }
                catch (Exception)
                {
                    _port?.Dispose();
                    _port = null;
                    throw;
                }
            }

            SendCommand(Command.GetNumServos);
            SendCommand(Command.GetServoParams);
            SendCommand(Command.GetServoData);
        }

        public void Dispose()
        {
            _port?.Dispose();
            _port = null;
            Model = null;
        }
    }
}