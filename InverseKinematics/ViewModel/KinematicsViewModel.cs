using System;
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
        private Bone _selectedEndPointBone, _selectedBone, _mainBone;

        /// <summary>
        /// Main bone of the sceleton
        /// </summary>
        public Bone MainBone
        {
            get {  return _mainBone; }
        }

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
            ClickPoint = MovePoint = bone.EndPosition;
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
            Bone.SelectNewInverse(bone);
        }

        #endregion

        #region Show about command

        private DelegateCommand _showAboutCommand;

        /// <summary>
        /// Shows about message
        /// </summary>
        public ICommand ShowAboutCommand
        {
            get
            {
                return _showAboutCommand ?? (_showAboutCommand =
                    new DelegateCommand(OnShowAboutCommad, _ => true));
            }
        }

        private void OnShowAboutCommad(object value)
        {
            MessageBox.Show(
                "Mathematical modelling project" + Environment.NewLine
                + "Simulation of forward and inverse kinematics on 2D skeleton" + Environment.NewLine 
                + "Authors: Peter Dobsa, Marek Zajko" + Environment.NewLine + Environment.NewLine
                + "Controls" + Environment.NewLine
                + "Bone creation: left click in the canvas to set starting and ending points of a bone, "
                + "when skeleton exists the starting point has to be an ending point of an existing bone" + Environment.NewLine
                + "Forward kinematics: right click and hold the selected bone, drag to modify rotation angle" + Environment.NewLine
                + "Inverse kinematics: left click to select the starting bone of the IK sequence and another left click to "
                + "select the end effector, a consecutive click into the canvas starts up the algorithm "
                + "and at the same time sets the target position of the end effector",
                "About", MessageBoxButton.OK, MessageBoxImage.Information);
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
                MovePoint = ClickPoint = new Vector(position.X, position.Y);
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
                MovePoint = ClickPoint = null;
            }
            else
            {
                // set target for inverse kinematics and start algorithm if bones are selected
                var target = new Vector(position.X, position.Y);
                if (Bone.StartInverseKinematics(target))
                    TargetPoint = target;
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
                TargetPoint = null;
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
