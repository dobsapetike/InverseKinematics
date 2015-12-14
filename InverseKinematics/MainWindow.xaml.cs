using System.Windows;
using System.Windows.Input;
using InverseKinematics.ViewModel;

namespace InverseKinematics
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow
    {
        public MainWindow()
        {
            InitializeComponent();
        }

        private void Window_OnMouseDown(object sender, MouseButtonEventArgs e)
        {
            ((KinematicsViewModel)MainCanvas.DataContext).HandleMouseClick(e, e.GetPosition(this));
        }

        private void Window_OnMouseMove(object sender, MouseEventArgs e)
        {
            ((KinematicsViewModel)MainCanvas.DataContext).HandleMouseMove(e, e.GetPosition(this));
        }
    }
}
