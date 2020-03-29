using NLog;
using JetBrains.Annotations;
using System;
using System.ComponentModel;
using System.IO.Ports;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;

namespace ServoPIDControl.Serial
{
    public sealed class ArduinoCppMockSerial : ISerialPort, INotifyPropertyChanged
    {
        private static readonly Logger Log = LogManager.GetCurrentClassLogger();

        // ReSharper disable once PrivateFieldCanBeConvertedToLocalVariable
        private readonly Action _serialCallback;
        private bool _isOpen;
        private bool _callBackTriggered;
        private bool _disposed;

        public event EventHandler Disposed;

        public ArduinoCppMockSerial()
        {
            _serialCallback = () => _callBackTriggered = true;
            SetCallback(_serialCallback);
        }

        public void Dispose()
        {
            if (_disposed)
                return;

            Log.Info($"Disposing {nameof(ArduinoCppMockSerial)}");
            _disposed = true;

            SetCallback(null);
            Close();
            Disposed?.Invoke(this, EventArgs.Empty);
        }

        public void Open()
        {
            Log.Debug("Open()");

            IsOpen = true;
        }

        public void Close()
        {
            Log.Debug("Close()");


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
            if (i != 0) 
                throw new NotImplementedException("Can only handle index = 0");

            Write(data, len);
        }

        public event SerialDataReceivedEventHandler DataReceived;

        private const string DllName = "ArduinoMock.dll";
        private const CallingConvention CallConvention = CallingConvention.Cdecl;

        [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "Serial_Write", CallingConvention = CallConvention)]
        private static extern void Write(byte[] data, int dataLen);

        [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "Serial_Read", CallingConvention = CallConvention)]
        private static extern void Read(byte[] data, int dataLen, out int available);

        [DllImport(DllName, EntryPoint = "Serial_SetCallback", CallingConvention = CallConvention)]
        private static extern void SetCallback(Action action);


        public event PropertyChangedEventHandler PropertyChanged;

        [NotifyPropertyChangedInvocator]
        private void OnPropertyChanged([CallerMemberName] string propertyName = null)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }
    }
}