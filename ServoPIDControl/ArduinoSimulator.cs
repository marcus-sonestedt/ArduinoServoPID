using NLog;
using ServoPIDControl.Serial;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Linq;
using System.Runtime.InteropServices;
using System.Windows;
using System.Windows.Threading;
using static System.Runtime.InteropServices.CallingConvention;

namespace ServoPIDControl
{
    public class ArduinoSimulator : IDisposable
    {
        internal class PhysicalModel
        {
            public double Acceleration { get; private set; }
            public double DamperConstant { get; set; } = 0.01;
            public double Mass { get; set; } = 0.4;
            public double SpringConstant { get; set; } = 10;
            public double Position { get; private set; }
            public double Velocity { get; private set; }

            public void Update(double extForce, double attachPos, double dt)
            {
                var force = (attachPos - Position) * SpringConstant;
                force += extForce;
                force += Velocity * -DamperConstant;

                Acceleration = force / Mass;
                Velocity += Acceleration * dt;
                Position += Velocity * dt;

                Position =
                    Position < -10 ? -10
                    : Position > 10 ? 10
                    : Position;
            }
        }

        private static readonly Logger Log = LogManager.GetCurrentClassLogger();

        private readonly DispatcherTimer _loopTimer;

        private ushort[] _on;
        private ushort[] _off;
        private byte[] _eeprom;
        private uint _arduinoTimeMicros;

        private readonly List<PhysicalModel> _physModels = new List<PhysicalModel>();

        public ArduinoCppMockSerial Serial { get; private set; } = new ArduinoCppMockSerial();

        public ArduinoSimulator()
        {
            _loopTimer = _loopTimer = new DispatcherTimer(
                DispatcherPriority.Normal,
                Application.Current.Dispatcher ?? Dispatcher.CurrentDispatcher
            )
            {
                Interval = TimeSpan.FromMilliseconds(10),
                IsEnabled = false,
            };

            _loopTimer.Tick += LoopTimerOnTick;
            Serial.PropertyChanged += SerialOnPropertyChanged;

            ReadArduinoExternalState();
            Log.Info($"{_on.Length} PWM Servo(s) and {_eeprom.Length} bytes of EEPROM");
        }

        private void LoopTimerOnTick(object sender, EventArgs eventArgs)
        {
            _arduinoTimeMicros += (uint) _loopTimer.Interval.TotalMilliseconds * 1000;
            SetMicros(_arduinoTimeMicros);

            Arduino_Loop();
            
            Serial.SendReceivedEvents();
            ReadArduinoExternalState();
            SimulateInputs();
        }

        private void SimulateInputs()
        {
            var dt = _loopTimer.Interval.TotalSeconds;


            for (var i = 0; i < Math.Min(4, _on.Length); ++i)
            {
                // generate external force (road bumps)
                var externalForce = Math.Pow(Math.Sin(3 * _arduinoTimeMicros * 1e-6), (i * 2 + 1)) * 10.0f;
                var servoPos = (((_off[i] - _on[i]) - 544.0f) / (2400.0f - 544.0f)) * 320.0f;

                _physModels[i].Update(externalForce, servoPos - 90, dt);

                var value = _physModels[i].Position + 90;
                value *= 1 / 320.0; // vary between 80 and 100 on 320 deg scale
                value *= 1023; // analog inputs are 10-bit integers
                SetAnalogInput((byte) i, (ushort)value);
            }
        }

        private void SerialOnPropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            switch (e.PropertyName)
            {
                case nameof(Serial.IsOpen) when Serial.IsOpen:
                    Log.Info("Resetting arduino and starting loop timer");
                    Arduino_Setup();
                    _loopTimer.Start();
                    break;
                case nameof(Serial.IsOpen):
                    Log.Info("Stopping loop timer");
                    _loopTimer.Stop();
                    break;
            }
        }

        public void ReadArduinoExternalState()
        {
            (_on, _off) = ReadPwm();
            _eeprom = ReadEeprom();

            if (_on.Length != _physModels.Count)
            {
                _physModels.Clear();
                _physModels.AddRange(Enumerable.Repeat(new PhysicalModel(), _on.Length));
            }
        }


        private const string DllName = "ArduinoMock";
        private const CallingConvention CallingConvention = Cdecl;

        [DllImport(DllName, EntryPoint = "Arduino_Setup", CallingConvention = CallingConvention)]
        public static extern void Arduino_Setup();

        [DllImport(DllName, EntryPoint = "Arduino_Loop", CallingConvention = CallingConvention)]
        private static extern void Arduino_Loop();

        [DllImport(DllName, EntryPoint = "Set_Micros", CallingConvention = CallingConvention)]
        private static extern void SetMicros(uint micros);

        [DllImport(DllName, EntryPoint = "EEPROM_Size", CallingConvention = CallingConvention)]
        public static extern int EepromSize();

        public static byte[] ReadEeprom()
        {
            var size = EepromSize();
            var data = new byte[size];
            Read_Eeprom(data, size);
            return data;
        }

        [DllImport(DllName, EntryPoint = "EEPROM_Read", CallingConvention = CallingConvention)]
        private static extern void Read_Eeprom(
            [MarshalAs(UnmanagedType.LPArray)] byte[] data,
            int len
        );


        [DllImport(DllName, EntryPoint = "PWM_NumServos", CallingConvention = CallingConvention)]
        private static extern int PwmNumServos();

        public static (ushort[], ushort[]) ReadPwm()
        {
            var nServos = PwmNumServos();
            var on = new ushort[nServos];
            var off = new ushort[nServos];
            PWM_Read(on, off);
            return (on, off);
        }

        [DllImport(DllName, EntryPoint = "PWM_Read", CallingConvention = CallingConvention)]
        private static extern void PWM_Read(
            [MarshalAs(UnmanagedType.LPArray)] ushort[] on,
            [MarshalAs(UnmanagedType.LPArray)] ushort[] off
        );

        [DllImport(DllName, EntryPoint = "AnalogInput_Set", CallingConvention = CallingConvention)]
        private static extern void SetAnalogInput(byte pin, ushort value);

        public void Dispose()
        {
            Serial?.Close();
            Serial?.Dispose();
            Serial = null;

            _loopTimer.Stop();
        }
    }
}