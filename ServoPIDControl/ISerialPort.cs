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
}