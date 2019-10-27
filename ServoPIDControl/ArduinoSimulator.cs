using NLog;
using ServoPIDControl.Serial;
using System;
using System.ComponentModel;
using System.Runtime.InteropServices;
using System.Windows;
using System.Windows.Threading;
using static System.Runtime.InteropServices.CallingConvention;

namespace ServoPIDControl
{
    public class ArduinoSimulator : IDisposable
    {
        private static readonly Logger Log = LogManager.GetCurrentClassLogger();

        private readonly DispatcherTimer _loopTimer;

        private ushort[] _on;
        private ushort[] _off;
        private byte[] _eeprom;
        private uint _arduinoTime;

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
            UpdateState();
            Log.Info($"{_on.Length} PWM Servos and {_eeprom.Length} bytes of EEPROM");
            Arduino_Setup();
        }

        private void LoopTimerOnTick(object sender, EventArgs eventArgs)
        {
            _arduinoTime += (uint) _loopTimer.Interval.TotalMilliseconds * 1000;
            SetMicros(_arduinoTime);
            Arduino_Loop();
            Serial.SendReceivedEvents();
        }

        private void SerialOnPropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            switch (e.PropertyName)
            {
                case nameof(Serial.IsOpen) when Serial.IsOpen:
                    Log.Info("Starting loop timer");
                    _loopTimer.Start();
                    break;
                case nameof(Serial.IsOpen):
                    Log.Info("Stopping loop timer");
                    _loopTimer.Stop();
                    break;
                default:
                    break;
            }
        }

        public void UpdateState()
        {
            (_on, _off) = ReadPwm();
            _eeprom = ReadEeprom();
        }

        private const string DllName = "ArduinoMock_Win32";
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

        public void Dispose()
        {
            Serial?.Close();
            Serial?.Dispose();
            Serial = null;

            _loopTimer.Stop();
        }
    }
}