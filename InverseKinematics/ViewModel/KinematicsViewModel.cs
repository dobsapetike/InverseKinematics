using System.Collections.ObjectModel;
using System.Linq;
using System.Windows;
using System.Windows.Input;
using InverseKinematics.Framework;
using InverseKinematics.Geometry;
using InverseKinematics.Geometry.Mathematics;
using Vector = InverseKinematics.Geometry.Mathematics.Vector;

namespace InverseKinematics.ViewModel
{
    /// <summary>
    /// View model for the whole application, binding the view with the 
    /// underlying model
    /// </summary>
    public class KinematicsViewModel : ObservableObject
    {

        #region Properties

        #region Clickpoint

        /// <summary>
        /// Last clicked point
        /// </summary>
        public Vector ClickPoint
        {
            get { return _clickPoint; }
            private set
            {
                _clickPoint = value;
                RaisePropertyChanged(() => ClickPoint);
            }
        }
        private Vector _clickPoint;

        #endregion

        #region Movepoint

        /// <summary>
        /// Last potential endpoint visited with the mouse cursor 
        /// </summary>
        public Vector MovePoint
        {
            get { return _movePoint; }
            private set
            {
                _movePoint = value;
                RaisePropertyChanged(() => MovePoint);
            }
        }
        private Vector _movePoint;

        #endregion

        #region Targetpoint

        /// <summary>
        /// Target point of the inverse kinematics
        /// </summary>
        public Vector TargetPoint
        {
            get { return _targetPoint; }
            private set
            {
                _targetPoint = value;
                RaisePropertyChanged(() => TargetPoint);
            }
        }
        private Vector _targetPoint;

        #endregion

        #region Bones

        /// <summary>
        /// Properties for hihglighted bones
        /// </summary>
        private Bone _selectedEndPointBone, _selectedBone, 
            _mainBone, _selectedInverseBaseBone, _selectedInverseEffectorBone;

        /// <summary>
        /// The main point of the skeleton
        /// </summary>
        public Vector MainPoint
        {
            get
            {
                if (_mainBone == null) return null;
                return _mainBone.StartPosition;
            }
        }

        /// <summary>
        /// Notifiable collection of all the bones in the skeleton
        /// </summary>
        public ObservableCollection<Bone> Bones { get; private set; }

        #endregion

        #endregion

        #region Constructor

        public KinematicsViewModel()
        {
            Bones = new ObservableCollection<Bone>();
        }

        #endregion

        #region Delegates

        #region End point click delegate

        private DelegateCommand _endPointClickCommand;

        /// <summary>
        /// Handles the bone endpoint click events and routes it to the model
        /// </summary>
        public ICommand EndPointClickCommand
        {
            get
            {
                return _endPointClickCommand ?? (_endPointClickCommand =
                    new DelegateCommand(OnEndPointClickCommand, _ => true));
            }
        }

        private void OnEndPointClickCommand(object value)
        {
            var bone = (Bone)value;
            _selectedEndPointBone = bone;
            ClickPoint = bone.EndPosition;
        }

        #endregion

        #region Bone click command

        private DelegateCommand _boneClickCommand;

        /// <summary>
        /// Handles the bone click events and routes it to the model
        /// </summary>
        public ICommand BoneClickCommand
        {
            get
            {
                return _boneClickCommand ?? (_boneClickCommand =
                    new DelegateCommand(OnBoneClickCommand, _ => true));
            }
        }

        private void OnBoneClickCommand(object value)
        {
            var bone = (Bone)value;
            if (_selectedBone != null)
                _selectedBone.IsSelected = false;
            bone.IsSelected = true;
            _selectedBone = bone;
        }

        #endregion

        #region Bone inverse click command

        private DelegateCommand _boneInverseClickCommand;

        /// <summary>
        /// Handles the bone click events and routes it to the model
        /// </summary>
        public ICommand BoneInverseClickCommand
        {
            get
            {
                return _boneInverseClickCommand ?? (_boneInverseClickCommand =
                    new DelegateCommand(OnBoneInverseClickCommand, _ => true));
            }
        }

        private void OnBoneInverseClickCommand(object value)
        {
            var bone = (Bone)value;
            if (!bone.IsEndEffector)
            {
                if (_selectedInverseBaseBone != null) 
                    _selectedInverseBaseBone.IsInverseBaseSelected = false;
                bone.IsInverseBaseSelected = !bone.IsInverseBaseSelected;
                _selectedInverseBaseBone = bone.IsInverseBaseSelected ? bone : null;
            }
            else
            {
                if (_selectedInverseEffectorBone != null)
                    _selectedInverseEffectorBone.IsInverseEffectorSelected = false;
                bone.IsInverseEffectorSelected = !bone.IsInverseEffectorSelected;
                _selectedInverseEffectorBone = bone.IsInverseEffectorSelected ? bone : null;
            }
        }

        #endregion

        #endregion

        #region Event handlers

        public void HandleMouseClick(MouseButtonEventArgs e, Point position)
        {
            if (e.ChangedButton != MouseButton.Left) return;

            // first click is a special case since the main structure hasn't been created yet
            if (ClickPoint == null && Bones.Count == 0)
            {
                ClickPoint = new Vector(position.X, position.Y);
                return;
            }

            if (ClickPoint != null)
            {
                // a bone has been created
                Bones.Add(new Bone(ClickPoint, MovePoint, _selectedEndPointBone));
                if (_mainBone == null)
                {
                    _mainBone = Bones.First();
                    RaisePropertyChanged(() => MainPoint);
                } 
                ClickPoint = MovePoint = null;
            }
            else
            {
                // set target for inverse kinematics and start algorithm if bones are selected
                if (_selectedInverseBaseBone != null && _selectedInverseEffectorBone != null)
                {
                    TargetPoint = new Vector(position.X, position.Y);
                    _selectedInverseBaseBone.InverseKinematics(_selectedInverseEffectorBone, TargetPoint);
                }
            }
        }

        public void HandleMouseMove(MouseEventArgs e, Point position)
        {
            // righ drag rearranges the structure by modifiing the selected bone's rotation angle
            if (e.RightButton == MouseButtonState.Pressed)
            {
                if (_selectedBone != null)
                    _selectedBone.ForwardKinematics(Angles.ComputeAngle(
                        _selectedBone.StartPosition, new Vector(position.X, position.Y)));
            }
            else
            {
                if (_selectedBone != null)
                {
                    _selectedBone.IsSelected = false;
                    _selectedBone = null;
                }
                if (ClickPoint != null) 
                    MovePoint = new Vector(position.X, position.Y);
            }
        }

        #endregion

    }
}
