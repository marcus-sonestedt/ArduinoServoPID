using System;
using System.Collections.Specialized;
using System.ComponentModel;
using System.Diagnostics;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Windows;
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
        private ArduinoModel _model;
        private readonly DispatcherTimer _timer = new DispatcherTimer {Interval = TimeSpan.FromMilliseconds(250)};
        private readonly object _portLock = new object();

        public ArduinoCom()
        {
            _timer.Tick += TimerOnTick;
            _timer.Start();

            MessageReceived += (s, a) => Debug.WriteLine($"Received: {a.Message}");
        }

        private void TimerOnTick(object sender, EventArgs e)
        {
            if ((!_port?.IsOpen ?? false) || Model == null)
                return;

            SendCommand(Command.GetServoData, (byte) Model.Servos.Count);
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

            string str;
            while ((str = _readBuf.ToString()).IndexOfAny(new [] {'\n', '\r'}) >= 0)
            {
                var lines = str.Split(new[] {'\n','\r'}, StringSplitOptions.RemoveEmptyEntries);
                Application.Current.Dispatcher.Invoke(() => LineReceived(lines.First().Trim()));
                _readBuf.Clear();
                _readBuf.Append(string.Join("\n", lines.Skip(1)));
            }
        }

        private void LineReceived(string line)
        {
            if (line.StartsWith("DT "))
            {
                var parts = line.Split(' ');
                try
                {
                    Model.DeltaTime = float.Parse(parts[1]);
                    Model.MinDt = float.Parse(parts[2]);
                    Model.MaxDt = float.Parse(parts[3]);
                }
                catch (Exception e)
                {
                    Debug.WriteLine($"Bad DT received: {line} - {e.Message}");
                }

                return;
            }

            if (line.StartsWith("NS "))
            {
                Application.Current.Dispatcher.Invoke(() =>
                {
                    if (!int.TryParse(line.Substring(3), out var numServos)) 
                        return;

                    if (numServos >= 33)
                    {
                        Debug.WriteLine("Too many servos: " + numServos);
                        return;
                    }
                   
                    Model.Servos.Clear();
                    for (var i = 0; i < numServos; ++i)
                        Model.Servos.Add(new ServoPidModel(Model.Servos.Count));
                });

                SendCommand(Command.GetServoParams);
            }
            else if (line.StartsWith("SP "))
            {
                var parts = line.Split(' ');
                try
                {
                    var servoId = int.Parse(parts[1]);
                    var servo = Model.Servos[servoId];
                    servo.P = float.Parse(parts[2]);
                    servo.I = float.Parse(parts[3]);
                    servo.D = float.Parse(parts[4]);
                    servo.DLambda = float.Parse(parts[5]);
                    servo.SetPoint = float.Parse(parts[6]);
                }
                catch (Exception e)
                {
                    Debug.WriteLine("Bad servo data: " + line + " - " + e.Message);
                    return;
                }
            }
            else if (line.StartsWith("SD "))
            {
                var parts = line.Split(' ');
                try
                {
                    var servoId = int.Parse(parts[1]);
                    var servo = Model.Servos[servoId];
                    servo.Input = float.Parse(parts[2]);
                    servo.Output = float.Parse(parts[3]);
                    servo.Integrator = float.Parse(parts[4]);
                    servo.DFiltered = float.Parse(parts[5]);
                }
                catch (Exception e)
                {
                    Debug.WriteLine($"Bad servo data: {line} - {e.Message}");
                    return;
                }
            }
            else if (line.StartsWith("ERR: "))
            {
                lock (_portLock)
                {
                    _port.WriteLine("RST");
                }
            }
            else
            {
                Debug.WriteLine($"Ignored: {line}");
                return;
            }

            MessageReceived?.Invoke(this, new StringEventArgs {Message = line});
        }

        public ArduinoModel Model
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

                    ModelOnPropertyChanged(this, new PropertyChangedEventArgs(null));
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
            if (e.OldItems != null)
                foreach (var servo in e.OldItems.Cast<ServoPidModel>())
                    servo.PropertyChanged -= ServoOnPropertyChanged;

            if (e.NewItems != null)
                foreach (var servo in e.NewItems.Cast<ServoPidModel>())
                    servo.PropertyChanged += ServoOnPropertyChanged;

            if (e.Action == NotifyCollectionChangedAction.Reset)
                foreach (var servo in Model.Servos)
                    servo.PropertyChanged -= ServoOnPropertyChanged;
        }

        private void ModelOnPropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            if (e.PropertyName == null || e.PropertyName == nameof(Model.Enabled))
                SendCommand(Command.EnableRegulator, (byte) (Model.Enabled ? 1 : 0));

            if (e.PropertyName == null || e.PropertyName == nameof(Model.PortName))
                ConnectPort();

            if (e.PropertyName == null || e.PropertyName == nameof(Model.PollPidData))
                _timer.IsEnabled = Model.PollPidData;
        }

        private void SendCommand(Command cmd, params byte[] data)
        {
            var cmdData = new[] {(byte) (data.Length + 2), (byte) cmd}.Concat(data).ToArray();

            lock (_portLock)
            {
                if (_port == null || !_port.IsOpen)
                    return;

                Debug.WriteLine($"Sending {cmdData.Length}: {BitConverter.ToString(cmdData)}");
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
                    try
                    {
                        _port.DataReceived -= PortOnDataReceived;
                        _port.Close();
                    }
                    finally
                    {
                        _port.Dispose();
                        _port = null;
                        Model.Connected = false;
                    }
                }

                if (Model?.PortName == null)
                    return;

                try
                {
                    _port = new SerialPort(Model.PortName)
                    {
                        BaudRate = 115200,
                        NewLine = "\n",
                        ReadBufferSize = 4096,
                        WriteBufferSize = 4096,
                    };
                    _port.Open();

                    Debug.WriteLine("Sending: RST");
                    _port.WriteLine("RST");
                    _port.DataReceived += PortOnDataReceived;

                    Model.Connected = true;
                }
                catch (Exception)
                {
                    _port?.Dispose();
                    _port = null;
                    //throw;
                }
            }

            SendCommand(Command.GetNumServos);

            //SendCommand(Command.EnableRegulator, (byte) (Model.Enabled ? 1 : 0));
        }

        public void Dispose()
        {
            _port?.Dispose();
            _port = null;
            Model = null;
        }
    }
}