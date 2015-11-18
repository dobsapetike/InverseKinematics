using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using InverseKinematics.Framework;
using InverseKinematics.Geometry.Mathematics;
using Vector = InverseKinematics.Geometry.Mathematics.Vector;

namespace InverseKinematics.Geometry
{
    /// <summary>
    /// Class representing one single bone of the sceleton 
    /// </summary>
    public class Bone : ObservableObject
    {

        #region Properties

        #region Start position

        /// <summary>
        /// Starting point of the bone
        /// </summary>
        public Vector StartPosition 
        { 
            get { return _startPosition; }
            set
            {
                _startPosition = value;
                RaisePropertyChanged(() => StartPosition);
                RaisePropertyChanged(() => EndPosition);
            }
        }
        private Vector _startPosition;

        #endregion

        #region End position

        /// <summary>
        /// End position of the bone, computed as a polar coordinate
        /// using the start pos as origin and lenght as radius 
        /// </summary>
        public Vector EndPosition
        {
            get
            {
                return new Vector(
                        StartPosition.X + Length * Math.Cos(Angle),
                        StartPosition.Y + Length * Math.Sin(Angle)
                    );
            }
        }

        #endregion

        #region Rotation angle

        /// <summary>
        /// Angle between the starting and end point
        /// </summary>
        public double Angle
        {
            get { return _angle; }
            private set
            {
                _angle = Angles.AdjustAngle(value);
                RaisePropertyChanged(() => EndPosition);
            }
        }
        private double _angle;

        #endregion

        //public double RelativeAngle
        //{
        //    get { return Parent == null ? Angle : Angles.AdjustAngle(Angle - Parent.Angle); }
        //}

        #region IsSelected

        /// <summary>
        /// Whether the bone has been selected for rotating
        /// </summary>
        public bool IsSelected
        {
            get { return _isSelected; }
            set
            {
                if (_isSelected == value) return;
                _isSelected = value;
                RaisePropertyChanged(() => IsSelected);
            }
        }
        private bool _isSelected;

        #endregion

        /// <summary>
        /// Length of the bone
        /// </summary>
        public double Length { get; private set; }

        /// <summary>
        /// Reference to the parent bone
        /// </summary>
        public Bone Parent { get; private set; }

        #region Children

        /// <summary>
        /// Read-only list of all the child bones
        /// </summary>
        public ReadOnlyCollection<Bone> Children
        {
            get { return _children.AsReadOnly(); }
        }
        private readonly List<Bone> _children = new List<Bone>();

        #endregion

        #endregion

        #region Constructor

        public Bone(Vector start, Vector end, Bone parent)
        {
            StartPosition = start;
            // set lenght as the euclidean distance between the starting and endpoint
            Length = Math.Sqrt((start.X - end.X)*(start.X - end.X) +
                     (start.Y - end.Y)*(start.Y - end.Y));
            Angle = Angles.ComputeAngle(start, end);

            // set parent references
            Parent = parent;
            if (parent != null)
                parent._children.Add(this);
        }

        #endregion

        #region Forward kinematics

        /// <summary>
        /// Given a new rotation angle, restructure the sceleton by 
        /// computing the new positions of the desdentant bones using 
        /// forward kinematics
        /// </summary>
        public void ForwardKinematics(double angle)
        {
            // set new angle of the selected bone
            var delta = Angle - angle;
            Angle = angle;
            var transMatrix = Matrix.Identity;
            transMatrix *= Matrix.Translate(StartPosition) 
                * Matrix.Translate(EndPosition - StartPosition);
            // now do a depth-first search and reaarange children
            foreach (var ch in Children)
                ForwardKinematics(ch, delta, transMatrix);
        }

        /// <summary>
        /// Applied forward kinematics to the descendants of the rotated bone
        /// </summary>
        private static void ForwardKinematics(Bone bone, double delta, Matrix transMatrix)
        {
            // compute new position and angle
            bone.StartPosition = transMatrix * Vector.UnitW;
            bone.Angle -= delta;
            // and recursively apply it to children
            foreach (var ch in bone.Children)
                ForwardKinematics(ch, delta,
                    transMatrix * Matrix.Translate(bone.EndPosition - bone.StartPosition));
        }

        #endregion

    }
}
