using System.Globalization;
using System.Windows;
using System.Windows.Data;

namespace InverseKinematics.Converters
{
    /// <summary>
    /// Simple workaround for the margin binding - because binding directly to coordinates 
    /// for non-canvas components is currently not possible :(
    /// </summary>
    public class MarginConverter : IMultiValueConverter
    {
        public object Convert(object[] values, System.Type targetType, object parameter, CultureInfo culture)
        {
            return new Thickness((double)values[0], (double)values[1], 0, 0);
        }

        public object[] ConvertBack(object value, System.Type[] targetTypes, object parameter, CultureInfo culture)
        {
            throw new System.NotImplementedException();
        }
    }
}
