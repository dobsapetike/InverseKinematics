using System;
using System.Windows.Input;

namespace InverseKinematics.Framework
{
    /// <summary>
    /// Simple command implementation for the viewmodel
    /// </summary>
    class DelegateCommand : ICommand
    {
        private readonly Action<object> _executeAction;
        private readonly Func<object, bool> _canExecuteAction;
        public event EventHandler CanExecuteChanged;

        public DelegateCommand(Action<object> executeAction, Func<object, bool> canExecuteAction)
        {
            _executeAction = executeAction;
            _canExecuteAction = canExecuteAction;
        }

        public bool CanExecute(object parameter)
        {
            return _canExecuteAction(parameter);
        }

        public void Execute(object parameter)
        {
            _executeAction.Invoke(parameter);
        }

        public void RaiseCanExecuteChanged()
        {
            CanExecuteChanged(this, new EventArgs());
        }
    }
}
