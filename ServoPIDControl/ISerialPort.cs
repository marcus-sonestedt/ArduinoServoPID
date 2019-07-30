using System;
using Ports = System.IO.Ports;

namespace ServoPIDControl
{
    /// <summary>
    /// Minimal 'interfacification' of <see cref="Ports.SerialPort"/> to allow mocking
    /// </summary>
    public interface ISerialPort : IDisposable
    {
        void Open();
        void Close();
        bool IsOpen { get; }

        string ReadExisting();
        void WriteLine(string s);
        void Write(byte[] data, int i, int cmdDataLength);
        
        event Ports.SerialDataReceivedEventHandler DataReceived;
    }

    /// <summary>
    /// Implements <see cref="ISerialPort"/> using <see cref="Ports.SerialPort"/>
    /// </summary>
    public class SerialPort : ISerialPort
    {
        private readonly Ports.SerialPort _port;

        public SerialPort(string portName)
        {
            _port = new Ports.SerialPort(portName);
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

        public void Dispose() => _port.Dispose();

        public bool IsOpen => _port.IsOpen;
        public string ReadExisting() => _port.ReadExisting();
        public void WriteLine(string s) => _port.WriteLine(s);

        public void Write(byte[] data, int i, int len)
            => _port.Write(data, i, len);

        public void Close() => _port.Close();
        public void Open() => _port.Open();

        public event Ports.SerialDataReceivedEventHandler DataReceived
        {
            add => _port.DataReceived += value;
            remove => _port.DataReceived -= value;
        }
    }
}