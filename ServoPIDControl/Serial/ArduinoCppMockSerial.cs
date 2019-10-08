using System;
using System.ComponentModel;
using System.IO.Ports;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;
using ServoPIDControl.Annotations;

namespace ServoPIDControl.Serial
{
    public class ArduinoCppMockSerial : ISerialPort, INotifyPropertyChanged
    {
        private bool _isOpen;
        public void Dispose() => Close();

        public void Open()
        {
            SetCallback(() => DataReceived?.Invoke(this,
                MockSerialPort.SerialCharsReceivedEventArgs));
        
            IsOpen = true;
        }

        public void Close()
        {
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

        public string ReadExisting()
        {
            var sb = new StringBuilder();
            var buf = new string('\0', 1024);
            int available;
            
            do
            {
                Read(ref buf, buf.Length, out available);
                sb.Append(buf, 0, Math.Min(available, buf.Length));
            } while (available > buf.Length);

            return sb.ToString();
        }

        public void WriteLine(string s)
        {
            Write(s);
            Write("\n");
        }

        public void Write(byte[] data, int i, int cmdDataLength)
        {
            var str = Encoding.ASCII.GetString(data);
            Write(str);
        }

        public event SerialDataReceivedEventHandler DataReceived;

        private const string DllName = "ArduinoMock_Win32.dll";

        [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "Serial_Write")]
        private static extern void Write(string s);

        [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "Serial_Read")]
        private static extern void Read(ref string s, int bufLen, out int available);

        [DllImport(DllName, EntryPoint = "Serial_SetCallback")]
        private static extern void SetCallback(Action action);

        public event PropertyChangedEventHandler PropertyChanged;

        [NotifyPropertyChangedInvocator]
        protected virtual void OnPropertyChanged([CallerMemberName] string propertyName = null)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }
    }
}