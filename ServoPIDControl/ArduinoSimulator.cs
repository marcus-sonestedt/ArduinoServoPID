using System;
using System.ComponentModel;
using System.Runtime.InteropServices;
using System.Threading;
using System.Windows.Threading;
using NLog;
using ServoPIDControl.Serial;
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

        public ArduinoCppMockSerial Serial { get; private set; } = new ArduinoCppMockSerial();

        public ArduinoSimulator()
        {
            _loopTimer = _loopTimer = new DispatcherTimer
            {
                Interval = TimeSpan.FromMilliseconds(10),
                IsEnabled = false,
            };
            _loopTimer.Tick += (s,a) => _Loop();
            Serial.PropertyChanged += SerialOnPropertyChanged;
            UpdateState();
            Log.Info($"{_on.Length} PWM Servos and {_eeprom.Length} bytes of EEPROM");
            _Setup();
        }

        private void SerialOnPropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            switch (e.PropertyName)
            {
                case nameof(Serial.IsOpen) when Serial.IsOpen:
                    Log.Info("Starting loop timer");
                    _loopTimer.Stop();
                    break;
                case nameof(Serial.IsOpen):
                    Log.Info("Stopping loop timer");
                    _loopTimer.Start();
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

        [DllImport(DllName, EntryPoint = "Arduino_Setup",CallingConvention = CallingConvention)]
        public static extern void _Setup();

        [DllImport(DllName, EntryPoint = "Arduino_Loop",CallingConvention = CallingConvention)]
        private static extern void _Loop();

        [DllImport(DllName, EntryPoint = "EEPROM_Size",CallingConvention = CallingConvention)]
        public static extern int EepromSize();

        public static byte[] ReadEeprom()
        {
            var size = EepromSize();
            var data = new byte[size];
            _ReadEeprom(data, size);
            return data;
        }

        [DllImport(DllName, EntryPoint = "EEPROM_Read",CallingConvention = CallingConvention)]
        private static extern void _ReadEeprom(
            [MarshalAs(UnmanagedType.LPArray, ArraySubType = UnmanagedType.ByValArray)]
            byte[] data,
            int len
        );


        [DllImport(DllName, EntryPoint = "PWM_NumServos", CallingConvention = CallingConvention)]
        private static extern int PwmNumServos();

        public static (ushort[], ushort[]) ReadPwm()
        {
            var nServos = PwmNumServos();
            var on = new ushort[nServos];
            var off = new ushort[nServos];
            _PWMRead(on, off);
            return (on, off);
        }

        [DllImport(DllName, EntryPoint = "PWM_Read", CallingConvention = CallingConvention)]
        private static extern void _PWMRead(ushort[] on, ushort[] off);

        public void Dispose()
        {
            Serial?.Close();
            Serial?.Dispose();
            Serial = null;

            _loopTimer.Stop();
        }
    }
}