using System;
using System.Runtime.InteropServices;
using static System.Runtime.InteropServices.CallingConvention;

namespace ServoPIDControl
{
    internal static class NativeMethods
    {
        internal const string DllName = "ArduinoMock";
        internal const CallingConvention CallingConvention = Cdecl;

        [DllImport(DllName, EntryPoint = "Arduino_Setup", CallingConvention = CallingConvention)]
        internal static extern void Arduino_Setup();

        [DllImport(DllName, EntryPoint = "Arduino_Loop", CallingConvention = CallingConvention)]
        internal static extern void Arduino_Loop();

        [DllImport(DllName, EntryPoint = "Set_Micros", CallingConvention = CallingConvention)]
        internal static extern void SetMicros(uint micros);

        [DllImport(DllName, EntryPoint = "EEPROM_Size", CallingConvention = CallingConvention)]
        internal static extern int EepromSize();

        [DllImport(DllName, EntryPoint = "EEPROM_Read", CallingConvention = CallingConvention)]
        internal static extern void Read_Eeprom(
            [MarshalAs(UnmanagedType.LPArray)] byte[] data,
            int len
        );

        [DllImport(DllName, EntryPoint = "PWM_NumServos", CallingConvention = CallingConvention)]
        internal static extern int PwmNumServos();

        [DllImport(DllName, EntryPoint = "PWM_Read", CallingConvention = CallingConvention)]
        internal static extern void PWM_Read(
            [MarshalAs(UnmanagedType.LPArray)] ushort[] on,
            [MarshalAs(UnmanagedType.LPArray)] ushort[] off
        );

        [DllImport(DllName, EntryPoint = "AnalogInput_Set", CallingConvention = CallingConvention)]
        internal static extern void SetAnalogInput(
            byte pin, 
            ushort value
        );


        [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "Serial_Write", CallingConvention = CallingConvention)]
        internal static extern void Write(byte[] data, int dataLen);

        [DllImport(DllName, CharSet = CharSet.Ansi, EntryPoint = "Serial_Read", CallingConvention = CallingConvention)]
        internal static extern void Read(byte[] data, int dataLen, out int available);

        [DllImport(DllName, EntryPoint = "Serial_SetCallback", CallingConvention = CallingConvention)]
        internal static extern void SetCallback(Action action);

    }
}