using NLog;
using ServoPIDControl.Annotations;
using System;
using System.ComponentModel;
using System.IO.Ports;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;

namespace ServoPIDControl.Serial
{
    public class ArduinoCppMockSerial : ISerialPort, INotifyPropertyChanged
    {
        private static readonly Logger Log = LogManager.GetCurrentClassLogger();

        private readonly Action _serialCallback;
        private bool _isOpen;
        private bool _callBackTriggered;
        private bool _disposed;

        public event EventHandler Disposed;

        public ArduinoCppMockSerial()
        {
            _serialCallback = () =>
            {
                _callBackTriggered = true;
            };
        }

        public void Dispose()
        {
            if (_disposed)
                return;

            Log.Info($"Disposing {nameof(ArduinoCppMockSerial)}");

            _disposed = true;
            Close();
            Disposed?.Invoke(this, EventArgs.Empty);
        }

        public void Open()
        {
            Log.Debug("Open()");

            SetCallback(_serialCallback);

            IsOpen = true;
        }

        public void Close()
        {
            Log.Debug("Close()");

            SetCallback(null);

            IsOpen = false;
        }

        public bool IsOpen
        {
            get => _isOpen;
            private set
            {
                if (value == _isOpen) return;
                _isOpen = value;
                OnPropertyChanged();
            }
        }

        public void SendReceivedEvents()
        {
            if (_callBackTriggered)
                DataReceived?.Invoke(this, MockSerialPort.SerialCharsReceivedEventArgs);
            _callBackTriggered = false;
        }

        public string ReadExisting()
        {
            var sb = new StringBuilder(1024);
            var buf = new byte[1024];
            int available;

            do
            {
                Read(buf, buf.Length, out available);
                var str = Encoding.ASCII.GetString(buf, 0, Math.Min(available, buf.Length));
                sb.Append(str);
            } while (available > buf.Length);

            return sb.ToString();
        }

        public void WriteLine(string s)
        {
            var bytes = Encoding.ASCII.GetBytes($"{s}\n");
            Write(bytes, bytes.Length);
        }

        public void Write(byte[] data, int i, int len)
        {
            // ugh
            var str = Encoding.ASCII.GetString(data, i, len);
            var bytes = Encoding.ASCII.GetBytes(str);
            Write(bytes, bytes.Length);
        }

        public event SerialDataReceivedEventHandler DataReceived;

        private const string DllName = "ArduinoMock_Win32.dll";
        private const CallingConvention CallConvention = CallingConvention.Cdecl;

        [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "Serial_Write", CallingConvention = CallConvention)]
        private static extern void Write(byte[] str, int strLen);

        [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "Serial_Read", CallingConvention = CallConvention)]
        private static extern void Read(byte[] buf, int bufLen, out int available);

        [DllImport(DllName, EntryPoint = "Serial_SetCallback", CallingConvention = CallConvention)]
        private static extern void SetCallback(Action action);


        public event PropertyChangedEventHandler PropertyChanged;

        [NotifyPropertyChangedInvocator]
        protected virtual void OnPropertyChanged([CallerMemberName] string propertyName = null)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }
    }
}