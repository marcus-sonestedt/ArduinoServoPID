using System;
using System.IO.Ports;

namespace ServoPIDControl.Serial
{
    /// <summary>
    /// Implements <see cref="ISerialPort"/> using <see cref="System.IO.Ports.SerialPort"/>
    /// </summary>
    public sealed class SerialPort : ISerialPort
    {
        private readonly System.IO.Ports.SerialPort _port;

        public SerialPort(string portName)
        {
            _port = new System.IO.Ports.SerialPort(portName);
        }

        public int BaudRate
        {
            get => _port.BaudRate;
            set => _port.BaudRate = value;
        }

        public string NewLine
        {
            get => _port.NewLine;
            set => _port.NewLine = value;
        }

        public void Dispose()
        {
            _port.Dispose();
        }

        public bool IsOpen => _port.IsOpen;
        public string ReadExisting() => _port.ReadExisting();
        public void WriteLine(string s) => _port.WriteLine(s);

        public void Write(byte[] data, int i, int len)
            => _port.Write(data, i, len);

        public void Close() => _port.Close();
        public void Open() => _port.Open();

        public event SerialDataReceivedEventHandler DataReceived
        {
            add => _port.DataReceived += value;
            remove => _port.DataReceived -= value;
        }
    }
}