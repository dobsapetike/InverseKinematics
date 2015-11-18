using InverseKinematics.Framework;

namespace InverseKinematics.Geometry.Mathematics
{
	public class Vector : ObservableObject
	{
		#region Properties

		public static readonly Vector Zero = new Vector(0, 0, 0, 0);
		public static readonly Vector UnitX = new Vector(1, 0, 0, 0);
		public static readonly Vector UnitY = new Vector(0, 1, 0, 0);
		public static readonly Vector UnitZ = new Vector(0, 0, 1, 0);
		public static readonly Vector UnitW = new Vector(0, 0, 0, 1);

		public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
        public double W { get; set; }

        public double Length
		{
			get { return System.Math.Sqrt(X * X + Y * Y + Z * Z + W * W); }
		}

		#endregion

		#region Init

        public Vector(double x, double y, double z = 0, double w = 1)
		{
			X = x;
			Y = y;
			Z = z;
			W = w;
		}

		#endregion

		#region Arithmetic Operations

		public static Vector operator +(Vector a, Vector b)
		{
			return new Vector(a.X + b.X, a.Y + b.Y, a.Z + b.Z, a.W + b.W);
		}

		public static Vector operator -(Vector a, Vector b)
		{
			return new Vector(a.X - b.X, a.Y - b.Y, a.Z - b.Z, a.W - b.W);
		}

        public static Vector operator *(Vector a, double b)
		{
			return new Vector(a.X * b, a.Y * b, a.Z * b, a.W * b);
		}

        public static Vector operator *(double a, Vector b)
		{
			return new Vector(a * b.X, a * b.Y, a * b.Z, a * b.W);
		}

		/// <summary>
		/// Dot Product
		/// </summary>
        public static double operator *(Vector a, Vector b)
		{
			return a.X * b.X + a.Y * b.Y + a.Z * b.Z + a.W * b.W;
		}

		/// <summary>
		/// 3D Cross Product
		/// </summary>
		public static Vector operator %(Vector a, Vector b)
		{
			return new Vector(a.Y * b.Z - a.Z * b.Y, a.Z * b.X - a.X * b.Z, a.X * b.Y - a.Y * b.X, 0);
		}

		#endregion

	}
}
