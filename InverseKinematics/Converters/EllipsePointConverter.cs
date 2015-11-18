using System;
using System.Globalization;
using System.Windows.Data;

namespace InverseKinematics.Converters
{
    public class EllipsePointConverter : IValueConverter
    {
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            return (double)value - int.Parse((string)parameter) / 2d;
        }

        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            throw new NotImplementedException();
        }
    }
}
