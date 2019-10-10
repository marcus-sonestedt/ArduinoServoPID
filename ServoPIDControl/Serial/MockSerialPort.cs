using System;
using System.Diagnostics;
using System.Reflection;
using System.Text;
using NLog;

namespace ServoPIDControl.Serial
{
    public class MockSerialPort : ISerialPort
    {
        private static readonly Logger Log = LogManager.GetCurrentClassLogger();
        private static readonly Stopwatch StopwatchTotal = Stopwatch.StartNew();
        private static readonly Stopwatch StopwatchDelta = Stopwatch.StartNew();

        private readonly StringBuilder _mockReadData = new StringBuilder();
        private readonly StringBuilder _mockWriteData = new StringBuilder();

        public void Dispose()
        {
            IsOpen = false;
        }

        public void Open()
        {
            IsOpen = true;
        }

        public void Close()
        {
            IsOpen = false;
        }

        public bool IsOpen { get; private set; }

        public string ReadExisting()
        {
            var data = _mockReadData.ToString();
            _mockReadData.Clear();
            return data;
        }

        public void WriteLine(string s)
        {
            _mockWriteData.Append(s);
            _mockWriteData.Append('\n');
            HandleWrittenData();
        }

        public void Write(byte[] data, int i, int len)
        {
            _mockWriteData.Append(Encoding.ASCII.GetString(data, i, len));
            HandleWrittenData();
        }

        public event System.IO.Ports.SerialDataReceivedEventHandler DataReceived;

        private void HandleWrittenData()
        {
            Log.Debug($"Received: '{BitConverter.ToString(Encoding.ASCII.GetBytes(_mockWriteData.ToString()))}'");

            if (_mockWriteData.ToString().Contains("RST\n"))
            {
                Send("RST ACK\n");
                var d = _mockWriteData.ToString();
                _mockWriteData.Remove(0, d.IndexOf("RST\n", StringComparison.Ordinal) + 4);
            }

            while (_mockWriteData.Length >= 2 && _mockWriteData.Length >= _mockWriteData[0])
            {
                var cmd = _mockWriteData.ToString().Substring(0, _mockWriteData[0]);

                switch (cmd)
                {
                    case "\u0002\u0003": // get num servos
                        Send("NS 4\n");
                        break;
                    case "\u0002\u0004": // get servo params
                        Send("SP 0 1 2 3 4 5\n");
                        Send("SP 1 2 2 3 4 5\n");
                        Send("SP 2 3 2 3 4 5\n");
                        Send("SP 3 4 2 3 4 5\n");
                        break;
                    case "\u0002\u0007": // get global vars
                        Send("GV 4 1 50 320 80 100\n");
                        break;
                    case "\u0002\u0008": // load eeprom
                        Send("OK\n");
                        break;
                    case "\u0002\u0009": // save eeprom
                        Send("OK\n");
                        break;
                    default:
                        if (cmd.StartsWith("\u0008\u0001")) // SetServoParamFloat (8 bytes)
                        {
                            Send("OK\n");
                        }
                        else if (cmd.StartsWith("\u0003\u0005")) // get servo data (3 bytes, ignore servo id)
                        {
                            var t = (float) StopwatchTotal.Elapsed.TotalSeconds * 10;
                            var dt = (float) StopwatchDelta.Elapsed.TotalSeconds;
                            StopwatchDelta.Restart();

                            Send($"DT {dt:F3} 0.2 0.3\n");
                            Send($"SD 0 {Math.Sin(t) * 2:F3} 0 3 4\n");
                            Send($"SD 1 {Math.Sin(t) * 4:F3} .5 3 4\n");
                            Send($"SD 2 {Math.Sin(t) * 6:F3} 1 3 4\n");
                            Send($"SD 3 {Math.Sin(t) * 8:F3} 1.5 3 4\n");
                        }
                        else if (cmd.StartsWith("\u0007\u0006")) // set global var (7 bytes)
                        {
                            Send("OK\n");
                        }
                        else
                        {
                            Log.Warn($"Unknown command: {BitConverter.ToString(Encoding.ASCII.GetBytes(cmd))}");
                        }

                        break;
                }

                _mockWriteData.Remove(0, cmd.Length);
            }
        }

        private void Send(string data)
        {
            Log.Debug($"Sending: {data.Trim()}");
            _mockReadData.Append(data);
            DataReceived?.BeginInvoke(this, SerialCharsReceivedEventArgs, null, null);
        }

        internal static readonly ConstructorInfo SerialDataReceivedEventArgsConstructorInfo
            = typeof(System.IO.Ports.SerialDataReceivedEventArgs).GetConstructor(
                BindingFlags.NonPublic | BindingFlags.Instance,
                null,
                new[] {typeof(System.IO.Ports.SerialData)},
                null);

        internal static readonly System.IO.Ports.SerialDataReceivedEventArgs SerialCharsReceivedEventArgs =
            (System.IO.Ports.SerialDataReceivedEventArgs) SerialDataReceivedEventArgsConstructorInfo.Invoke(new object[]
                {System.IO.Ports.SerialData.Chars});
    }
}