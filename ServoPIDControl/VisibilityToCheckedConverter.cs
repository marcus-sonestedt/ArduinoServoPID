using System;
using System.Windows;
using System.Windows.Data;

namespace ServoPIDControl
{
    public class VisibilityToCheckedConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            if (value == null)
                return Visibility.Collapsed;

            return ((Visibility)value) == Visibility.Visible;
        }

        public object ConvertBack(object value, Type targetType, object parameter, System.Globalization.CultureInfo culture)
        {
            if (value == null)
                value = false;

            return ((bool)value) ? Visibility.Visible : Visibility.Collapsed;
        }
    }
}