using System;
using System.ComponentModel;
using System.Runtime.InteropServices;
using System.Threading;
using NLog;
using ServoPIDControl.Serial;

namespace ServoPIDControl
{
    public class ArduinoSimulator : IDisposable
    {
        private static readonly Logger Log = LogManager.GetCurrentClassLogger();
        
        private Timer _loopTimer;

        public ArduinoCppMockSerial Serial { get; } = new ArduinoCppMockSerial();

        public ArduinoSimulator()
        {
            Serial.PropertyChanged += SerialOnPropertyChanged;
            _Setup();
        }

        private void SerialOnPropertyChanged(object sender, PropertyChangedEventArgs e)
        {
            if (e.PropertyName == nameof(Serial.IsOpen))
            {
                if (Serial.IsOpen)
                {
                    _loopTimer?.Dispose();
                    _loopTimer = new Timer(_ => _Loop(), null, TimeSpan.MinValue, TimeSpan.FromMilliseconds(10));
                }
                else
                {
                    _loopTimer?.Dispose();
                    _loopTimer = null;
                }
            }
        }

        private const string DllName = "ArduinoMock_Win32";

        [DllImport(DllName, EntryPoint = "Arduino_Setup")]
        public static extern void _Setup();

        [DllImport(DllName, EntryPoint = "Arduino_Loop")]
        private static extern void _Loop();

        [DllImport(DllName, EntryPoint = "EEPROM_Size")]
        public static extern int EepromSize();

        public static byte[] ReadEeprom()
        {
            var size = EepromSize();
            var data = new byte[size];
            _ReadEeprom(data, size);
            return data;
        }

        [DllImport(DllName, EntryPoint = "EEPROM_Read")]
        private static extern void _ReadEeprom(
            [MarshalAs(UnmanagedType.LPArray, ArraySubType = UnmanagedType.ByValArray)]
            byte[] data,
            int len
        );

        public void Dispose()
        {
            Serial?.Close();
            Serial?.Dispose();
            _loopTimer?.Dispose();
        }
    }
}