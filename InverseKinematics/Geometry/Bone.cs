using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Linq;
using System.Net;
using System.Threading;
using System.Windows.Threading;
using InverseKinematics.Framework;
using InverseKinematics.Geometry.Mathematics;
using Vector = InverseKinematics.Geometry.Mathematics.Vector;

namespace InverseKinematics.Geometry
{
    /// <summary>
    /// Class representing one single bone of the skeleton 
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

        /// <summary>
        /// Whether the bone has been selected for inverse kinematics base
        /// </summary>
        public bool IsInverseBaseSelected
        {
            get { return _isInverseBaseSelected; }
            set
            {
                if (_isInverseBaseSelected == value) return;
                _isInverseBaseSelected = value;
                RaisePropertyChanged(() => IsInverseBaseSelected);
            }
        }
        private bool _isInverseBaseSelected;

        /// <summary>
        /// Whether the bone has been selected for inverse kinematics end effector
        /// </summary>
        public bool IsInverseEffectorSelected
        {
            get { return _isInverseEffectorSelected; }
            set
            {
                if (_isInverseEffectorSelected == value) return;
                _isInverseEffectorSelected = value;
                RaisePropertyChanged(() => IsInverseEffectorSelected);
            }
        }
        private bool _isInverseEffectorSelected;

        #endregion

        /// <summary>
        /// Length of the bone
        /// </summary>
        public double Length { get; private set; }

        /// <summary>
        /// Reference to the parent bone
        /// </summary>
        public Bone Parent { get; private set; }

        /// <summary>
        /// Whether the bone is an end effector - ie. a last link
        /// </summary>
        public bool IsEndEffector { get { return Children.Count == 0; } }

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
            Length = Vector.Distance(start, end);
            Angle = Angles.ComputeAngle(start, end);

            // set parent references
            Parent = parent;
            if (parent != null)
                parent._children.Add(this);
        }

        #endregion

        #region Forward kinematics

        /// <summary>
        /// Given a new rotation angle, restructure the skeleton by 
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
            var stack = new Stack<Tuple<Bone, Matrix>>(
                Children.Select(x => Tuple.Create(x, transMatrix)));
            while (stack.Count > 0)
            {
                var bp = stack.Pop();
                var bone = bp.Item1;
                var tMatrix = bp.Item2;
                bone.StartPosition = tMatrix * Vector.UnitW;
                bone.Angle -= delta;

                var ntMatrix = tMatrix * Matrix.Translate(bone.EndPosition - bone.StartPosition);
                bone._children.ForEach(x => stack.Push(Tuple.Create(x, ntMatrix)));
            }
        }

        #endregion

        #region Inverse kinematics

        /// <summary>
        /// Halting criterions
        /// </summary>
        private const int IterationLimit = 200;
        private const double DistanceLimit = .5d;

        /// <summary>
        /// Returns a list of all bones between the the current and 
        /// bone 'desc', which is a descendant of the current
        /// </summary>
        private List<Bone> GatherAncestors(Bone desc)
        {
            var bones = new List<Bone>();
            while (desc != null)
            {
                bones.Add(desc);
                if (desc == this) return bones;
                desc = desc.Parent;
            }
            return null;
        }

        /// <summary>
        /// Performs inverse kinematics, using the current bone as the starting point of the IK sequence
        /// </summary>
        public void InverseKinematics(Bone endEffector, Vector target)
        {
            var bones = GatherAncestors(endEffector);
            var angles = bones.Select(x => x.Angle).ToArray();

            var iterations = 0;
            while (Vector.Distance(target, endEffector.EndPosition)
                > DistanceLimit && iterations++ < IterationLimit)
            {
                InverseKinematicsStep(bones, endEffector, target);
            }

            var anglesNew = bones.Select(x => x.Angle).ToArray();
            for (var i = 0; i < bones.Count; ++i) bones[i]._angle = angles[i];

            // apply new angles gradually
            var worker = new BackgroundWorker();
            worker.DoWork += (s, e) =>
            {
                for (var j = 0; j < 10; ++j)
                {
                    for (var i = 0; i < bones.Count; ++i)
                    {
                        bones[i].Angle += (anglesNew[i] - angles[i]) / 10d;
                        bones[i]._children.ForEach(x => x.StartPosition = bones[i].EndPosition);
                    }
                    ForwardKinematics(Angle);
                    Thread.Sleep(25);
                }
            };
            worker.RunWorkerAsync();
        }

        /// <summary>
        /// Performs a step of inverse kinematics using jacobian relaxation
        /// </summary>
        private void InverseKinematicsStep(List<Bone> ikSequence, Bone endEffector, Vector target)
        {
            var jacobian = new double[2, ikSequence.Count];
            for (var i = 0; i < ikSequence.Count; ++i)
            {
                var v = ikSequence[i].EndPosition - ikSequence[i].StartPosition;
                v /= v.Length;
                v.Z = 1;
                // compute partial derivatives
                var partderiv = v % (target - ikSequence[i].StartPosition);
                jacobian[0, i] = partderiv.X;
                jacobian[1, i] = partderiv.Y;
            }
            var e = target - endEffector.EndPosition;
            var jinv = Accord.Math.Matrix.PseudoInverse(jacobian);
            var angles = Accord.Math.Matrix.Multiply(jinv, new[] { e.X, e.Y });

            for (var i = 0; i < ikSequence.Count; ++i)
            {
                ikSequence[i]._angle += angles[i];
                ikSequence[i]._children.ForEach(x => x._startPosition = ikSequence[i].EndPosition);
            }
        }

        #endregion

    }
}
