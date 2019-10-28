using System;
using System.Collections.Specialized;
using System.ComponentModel;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Threading;
using NLog;
using ServoPIDControl.Model;
using ServoPIDControl.Serial;
using static System.Globalization.CultureInfo;
using static ServoPIDControl.Command;
using static ServoPIDControl.GlobalVar;
using SerialPort = ServoPIDControl.Serial.SerialPort;

namespace ServoPIDControl
{
    public enum Command : byte
    {
        // ReSharper disable once UnusedMember.Global
        NoOp = 0,
        SetServoParamFloat,
        EnableRegulator,
        GetNumServos,
        GetServoParams,
        GetServoData,
        SetGlobalVar,
        GetGlobalVars,
        LoadEeprom,
        SaveEeprom,
        ResetToDefault,
    }

    public enum ServoParam : byte
    {
        // ReSharper disable once UnusedMember.Global
        None,
        P,
        I,
        D,
        DLambda,
        SetPoint,
        InputScale,
        InputBias
    }

    public enum GlobalVar : byte
    {
        NumServos,
        PidEnabled,
        PidMaxIntegratorStore,
        AnalogInputRange,
        ServoMinAngle,
        ServoMaxAngle,
    }

    /// <summary>
    /// Communicates with Arduino over SerialPort, updating data on either side
    /// </summary>
    public class ArduinoCom : IDisposable
    {
        private static readonly Logger Log = LogManager.GetCurrentClassLogger();
        private static readonly char[] Separators = {'\n', '\r'};

        private ISerialPort _port;
        private ViewModel _model;
        private readonly StringBuilder _readBuf = new StringBuilder();
        private readonly DispatcherTimer _timer = new DispatcherTimer {Interval = TimeSpan.FromMilliseconds(50)};
        private readonly object _portLock = new object();
        private bool _updatingGlobalVarsFromArduino;

        public ArduinoCom()
        {
            _timer.Tick += TimerOnTick;
            _timer.Start();
        }

        private void TimerOnTick(object sender, EventArgs e)
        {
            if ((!_port?.IsOpen ?? false) || Model == null)
                return;

            SendCommand(GetServoData, (byte) Model.Servos.Count);
        }

        private void PortOnDataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            lock (_portLock)
            {
                _readBuf.Append(_port.ReadExisting());

                while (_readBuf.Length > 0 && _readBuf[0] == '\n')
                    _readBuf.Remove(0, 1);

                if (_readBuf.Length == 0)
                {
                    Log.Warn("Data received triggered but no data read?");
                    return;
                }

                string str;
                var dispatcher = Application.Current.Dispatcher ??
                                 Dispatcher.CurrentDispatcher;
                while ((str = _readBuf.ToString()).IndexOfAny(Separators) > 0)
                {
                    var lines = str.Split(Separators, StringSplitOptions.RemoveEmptyEntries);
                    var line = lines.First().Trim();
                    dispatcher.InvokeAsync(() => LineReceived(line));

                    _readBuf.Clear();
                    _readBuf.Append(string.Join("\n", lines.Skip(1)));

                    if (str.LastOrDefault() == '\n')
                        _readBuf.Append('\n');
                }
            }
        }

        private void LineReceived(string line)
        {
            if (line.StartsWith("DT "))
            {
                var parts = line.Split(' ');
                try
                {
                    Model.DeltaTime = float.Parse(parts[1], InvariantCulture);
                    Model.MinDt = float.Parse(parts[2], InvariantCulture);
                    Model.MaxDt = float.Parse(parts[3], InvariantCulture);
                }
                catch (Exception e)
                {
                    Log.Error($"Bad DT received: {line} - {e.Message}");
                }

                return;
            }

            if (line.StartsWith("NS "))
            {
                var dispatcher = Application.Current.Dispatcher ??
                                 Dispatcher.CurrentDispatcher;

                dispatcher.Invoke(() =>
                {
                    if (!int.TryParse(line.Substring(3), out var numServos))
                        return;

                    if (numServos > 16) // u crazy?
                    {
                        Log.Error("Too many servos: " + numServos);
                        return;
                    }

                    Model.Servos.Clear();
                    for (var i = 0; i < numServos; ++i)
                        Model.Servos.Add(new ServoPidModel(Model.Servos.Count));
                });

                SendCommand(GetServoParams);
                return;
            }

            if (line.StartsWith("SP "))
            {
                var parts = line.Split(' ');
                try
                {
                    var servoId = int.Parse(parts[1]);
                    var servo = Model.Servos[servoId];
                    servo.P = float.Parse(parts[2], InvariantCulture);
                    servo.I = float.Parse(parts[3], InvariantCulture);
                    servo.D = float.Parse(parts[4], InvariantCulture);
                    servo.DLambda = float.Parse(parts[5], InvariantCulture);
                    servo.SetPoint = float.Parse(parts[6], InvariantCulture);

                    if (servoId == Model.Servos.Count - 1)
                        SendCommand(GetServoData, 0x80);
                }
                catch (Exception e)
                {
                    Log.Error($"Bad servo data: {line} - {e.Message}");
                }

                return;
            }

            if (line.StartsWith("SD "))
            {
                var parts = line.Split(' ');
                try
                {
                    var servoId = int.Parse(parts[1]);
                    var servo = Model.Servos[servoId];
                    servo.Input = float.Parse(parts[2], InvariantCulture);
                    servo.Output = float.Parse(parts[3], InvariantCulture);
                    servo.Integrator = float.Parse(parts[4], InvariantCulture);
                    servo.DFiltered = float.Parse(parts[5], InvariantCulture);

                    servo.RecordTimePoint();
                }
                catch (Exception e)
                {
                    Log.Error($"Bad servo data: {line} - {e.Message}");
                }

                return;
            }

            if (line.StartsWith("GV "))
            {
                var parts = line.Split(' ');
                try
                {
                    _updatingGlobalVarsFromArduino = true;
                    Model.GlobalVar[NumServos].Value = int.Parse(parts[1]);
                    Model.GlobalVar[PidEnabled].Value = int.Parse(parts[2]);
                    Model.GlobalVar[PidMaxIntegratorStore].Value = float.Parse(parts[3], InvariantCulture);
                    Model.GlobalVar[AnalogInputRange].Value = float.Parse(parts[4], InvariantCulture);
                    Model.GlobalVar[ServoMinAngle].Value = float.Parse(parts[5], InvariantCulture);
                    Model.GlobalVar[ServoMaxAngle].Value = float.Parse(parts[6], InvariantCulture);
                }
                catch (Exception e)
                {
                    Log.Error($"Bad global variable data: {line} - {e.Message}");
                }
                finally
                {
                    _updatingGlobalVarsFromArduino = false;
                }

                return;
            }


            if (line.StartsWith("ERR: "))
            {
                Log.Error($"Received: {line}");

                lock (_portLock)
                {
                    _port.WriteLine("RST");
                }

                return;
            }

            if (line == "RST ACK" || line == "OK")
            {
                Log.Debug("Received: " + line);
                return;
            }

            Log.Warn($"Unknown message: {line}");
        }

        public ViewModel Model
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

                    foreach (var gv in Model.GlobalVars)
                        gv.PropertyChanged -= GlobalVarOnPropertyChanged;
                }

                _model = value;

                if (Model != null)
                {
                    Model.PropertyChanged += ModelOnPropertyChanged;
                    Model.Servos.CollectionChanged += ServosOnCollectionChanged;

                    foreach (var s in Model.Servos)
                        s.PropertyChanged += ServoOnPropertyChanged;

                    foreach (var gv in Model.GlobalVars)
                        gv.PropertyChanged += GlobalVarOnPropertyChanged;

                    ModelOnPropertyChanged(this, new PropertyChangedEventArgs(null));
                }

                ConnectPort();
            }
        }

        private void GlobalVarOnPropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            if (_updatingGlobalVarsFromArduino)
                return;

            var globalVar = (GlobalVarModel) sender;

            var data = new[] {(byte) globalVar.Variable}
                .Concat(BitConverter.GetBytes(globalVar.Value))
                .ToArray();

            SendCommand(SetGlobalVar, data);
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
                default:
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
                // it's probably empty, but just in case...
                foreach (var servo in Model.Servos)
                    servo.PropertyChanged -= ServoOnPropertyChanged;
        }

        private void ModelOnPropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            if (e.PropertyName == null || e.PropertyName == nameof(Model.PidEnabled))
                SendCommand(EnableRegulator, (byte) (Model.PidEnabled ? 1 : 0));

            if (e.PropertyName == null || e.PropertyName == nameof(Model.ConnectedPort))
                ConnectPort();

            if (e.PropertyName == null || e.PropertyName == nameof(Model.PollPidData))
                _timer.IsEnabled = Model.PollPidData;
        }

        public void SendCommand(Command cmd, params byte[] data)
        {
            var cmdData = new[] {(byte) (data.Length + 2), (byte) cmd}.Concat(data).ToArray();

            lock (_portLock)
            {
                if (_port == null || !_port.IsOpen)
                    return;

                Log.Debug($"Sending {cmdData.Length}: {BitConverter.ToString(cmdData)}");
                _port.Write(cmdData, 0, cmdData.Length);
            }
        }

        private void SendServoParam(int servoId, ServoParam servoParam, float value)
        {
            SendCommand(SetServoParamFloat,
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

                        if (Model is ViewModel model)
                            model.Connected = false;
                    }
                }

                if (Model?.ConnectedPort == null)
                    return;

                try
                {
                    switch (Model.ConnectedPort)
                    {
                        case "Mock":
                            _port = CreateMockPort();
                            break;
                        case "Simulator":
                            _port = CreateSimulatorPort();
                            break;
                        default:
                            _port = new SerialPort(Model.ConnectedPort)
                            {
                                BaudRate = 115200,
                                NewLine = "\n"
                            };
                            break;
                    }

                    _port.Open();

                    Log.Info("Sending: RST");
                    _port.WriteLine("RST");
                    _port.DataReceived += PortOnDataReceived;

                    Model.Connected = true;
                }
                catch (Exception e)
                {
                    Log.Warn(e, $"Failed to open port: {e.Message}");

                    _port?.Dispose();
                    _port = null;

                    Model.Connected = false;
                    //throw;
                }
            }

            SendCommand(GetNumServos);
            SendCommand(GetGlobalVars);

            //SendCommand(Command.EnableRegulator, (byte) (Model.Enabled ? 1 : 0));
        }

        private static ISerialPort CreateSimulatorPort()
        {
#pragma warning disable IDE0067 // Dispose objects before losing scope
             var sim = new ArduinoSimulator();
             sim.Serial.Disposed += (s, a) => sim.Dispose();
             return sim.Serial;
#pragma warning restore IDE0067 // Dispose objects before losing scope
        }

        private static ISerialPort CreateMockPort()
        {
            return new MockSerialPort();
        }

        public void Dispose()
        {
            _port?.Dispose();
            _port = null;
            Model = null;
        }
    }
}