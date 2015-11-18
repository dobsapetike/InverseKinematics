using System;
using System.ComponentModel;
using System.Linq.Expressions;

namespace InverseKinematics.Framework
{
    /// <summary>
    /// Implements interface for the observable pattern, every notification
    /// object used in this projects inherts from this class.
    /// Allows raising notifications through lambda expressions
    /// </summary>
    public abstract class ObservableObject : INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler PropertyChanged;

        protected virtual void RaisePropertyChanged(Expression<Func<object>> expression)
        {
            var handler = PropertyChanged;
            if (handler == null) return;
            if (expression.NodeType != ExpressionType.Lambda)
            {
                throw new ArgumentException("Value must be a lambda expression");
            }

            var body = expression.Body as MemberExpression ??
                ((UnaryExpression)expression.Body).Operand as MemberExpression;
            if (body == null) return;
            var propertyName = body.Member.Name;

            handler(this, new PropertyChangedEventArgs(propertyName));
        }
    }
}
