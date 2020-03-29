using NLog;
using ServoPIDControl.Serial;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Windows;
using System.Windows.Threading;

namespace ServoPIDControl
{
    public sealed class ArduinoSimulator : IDisposable
    {
        private static readonly Logger Log = LogManager.GetCurrentClassLogger();

        private readonly DispatcherTimer _loopTimer;

        private ushort[] _on;
        private ushort[] _off;
        private byte[] _eeprom;
        private uint _arduinoTimeMicros;

        private readonly List<MassSpringModel> _physModels = new List<MassSpringModel>();
        private readonly ArduinoCom _com;

        public ArduinoCppMockSerial Serial { get; private set; } = new ArduinoCppMockSerial();

        public ArduinoSimulator(ArduinoCom com)
        {
            _com = com;
            _loopTimer = _loopTimer = new DispatcherTimer(
                DispatcherPriority.Normal,
                Application.Current.Dispatcher ?? Dispatcher.CurrentDispatcher
            )
            {
                Interval = TimeSpan.FromMilliseconds(100),
                IsEnabled = false,
            };

            _loopTimer.Tick += LoopTimerOnTick;
            Serial.PropertyChanged += SerialOnPropertyChanged;

            ReadArduinoExternalState();
            Log.Info($"{_on.Length} PWM Servo(s) and {_eeprom.Length} bytes of EEPROM");
        }

   
        private void SerialOnPropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            switch (e.PropertyName)
            {
                case nameof(Serial.IsOpen) when Serial.IsOpen:
                    Log.Info("Resetting arduino and starting loop timer");
                    NativeMethods.Arduino_Setup();
                    _loopTimer.Start();
                    break;
                case nameof(Serial.IsOpen):
                    Log.Info("Stopping loop timer");
                    _loopTimer.Stop();
                    break;
            }
        }

        private void LoopTimerOnTick(object sender, EventArgs eventArgs)
        {
            const int subSteps = 20;
            var dt = _loopTimer.Interval.TotalSeconds / subSteps;

            for (var i = 0; i < subSteps; ++i)
            {
                _arduinoTimeMicros += (uint)(dt * 1e6f);
                NativeMethods.SetMicros(_arduinoTimeMicros);
                _com.SetTime = _arduinoTimeMicros * 1e-6f;

                _com.SendCommand(Command.GetServoData, (byte)255);
                NativeMethods.Arduino_Loop();
                Serial.SendReceivedEvents();
                ReadArduinoExternalState();
                SimulateInputs((float)dt);
            }
        }

        private void SimulateInputs(float dt)
        {
            for (var i = 0; i < Math.Min(4, _on.Length); ++i)
            {
                // generate external force (road bumps)
                var externalForce = Math.Pow(Math.Sin(3 * _arduinoTimeMicros * 1e-6 + i / (2.0f * Math.PI)), 3) * 30.0f;
                var servoPos = (((_off[i] - _on[i]) - 544.0f) / (2400.0f - 544.0f)) * 320.0f;

                _physModels[i].Update(externalForce, servoPos - 90, dt);

                var value = _physModels[i].Position + 90;
                value *= 1 / 320.0; // vary between 80 and 100 on 320 deg scale
                value *= 1023; // analog inputs are 10-bit integers
                NativeMethods.SetAnalogInput((byte)i, (ushort)value);
            }
        }


        public void ReadArduinoExternalState()
        {
            (_on, _off) = ReadPwm();
            _eeprom = ReadEeprom();

            if (_on.Length != _physModels.Count)
            {
                _physModels.Clear();
                _physModels.AddRange(Enumerable.Repeat(new MassSpringModel(), _on.Length));
            }
        }


        private static byte[] ReadEeprom()
        {
            var size = NativeMethods.EepromSize();
            var data = new byte[size];
            NativeMethods.Read_Eeprom(data, size);
            return data;
        }

        private static (ushort[], ushort[]) ReadPwm()
        {
            var nServos = NativeMethods.PwmNumServos();
            var on = new ushort[nServos];
            var off = new ushort[nServos];
            NativeMethods.PWM_Read(on, off);
            return (on, off);
        }

        public void Dispose()
        {
            Serial?.Close();
            Serial?.Dispose();
            Serial = null;

            _loopTimer.Stop();
        }
    }
}