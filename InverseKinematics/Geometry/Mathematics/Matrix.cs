namespace InverseKinematics.Geometry.Mathematics
{
    public class Matrix
    {

        #region Properties

        public const int Dim = 4;

        public double M00, M01, M02, M03;
        public double M10, M11, M12, M13;
        public double M20, M21, M22, M23;
        public double M30, M31, M32, M33;


        public static readonly Matrix Zero = new Matrix(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

        public static Matrix Identity
        {
            get { return new Matrix(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1); }
        }

        #endregion

        #region Init

        public Matrix(
            double m00, double m01, double m02, double m03,
            double m10, double m11, double m12, double m13,
            double m20, double m21, double m22, double m23,
            double m30, double m31, double m32, double m33)
        {
            M00 = m00; M01 = m01; M02 = m02; M03 = m03;
            M10 = m10; M11 = m11; M12 = m12; M13 = m13;
            M20 = m20; M21 = m21; M22 = m22; M23 = m23;
            M30 = m30; M31 = m31; M32 = m32; M33 = m33;
        }

        #endregion

        #region Arithmetic Operations

        public static Matrix operator -(Matrix a)
        {
            return new Matrix(
                -a.M00, -a.M01, -a.M02, -a.M03,
                -a.M10, -a.M11, -a.M12, -a.M13,
                -a.M20, -a.M21, -a.M22, -a.M23,
                -a.M30, -a.M31, -a.M32, -a.M33
            );
        }

        public static Matrix operator +(Matrix a, Matrix b)
        {
            return new Matrix(
                a.M00 + b.M00, a.M01 + b.M01, a.M02 + b.M02, a.M03 + b.M03,
                a.M10 + b.M10, a.M11 + b.M11, a.M12 + b.M12, a.M13 + b.M13,
                a.M20 + b.M20, a.M21 + b.M21, a.M22 + b.M22, a.M23 + b.M23,
                a.M30 + b.M30, a.M31 + b.M31, a.M32 + b.M32, a.M33 + b.M33
            );
        }

        public static Matrix operator -(Matrix a, Matrix b)
        {
            return new Matrix(
                a.M00 - b.M00, a.M01 - b.M01, a.M02 - b.M02, a.M03 - b.M03,
                a.M10 - b.M10, a.M11 - b.M11, a.M12 - b.M12, a.M13 - b.M13,
                a.M20 - b.M20, a.M21 - b.M21, a.M22 - b.M22, a.M23 - b.M23,
                a.M30 - b.M30, a.M31 - b.M31, a.M32 - b.M32, a.M33 - b.M33
            );
        }

        public static Matrix operator *(Matrix a, Matrix b)
        {
            return new Matrix(
                a.M00 * b.M00 + a.M01 * b.M10 + a.M02 * b.M20 + a.M03 * b.M30,
                a.M00 * b.M01 + a.M01 * b.M11 + a.M02 * b.M21 + a.M03 * b.M31,
                a.M00 * b.M02 + a.M01 * b.M12 + a.M02 * b.M22 + a.M03 * b.M32,
                a.M00 * b.M03 + a.M01 * b.M13 + a.M02 * b.M23 + a.M03 * b.M33,

                a.M10 * b.M00 + a.M11 * b.M10 + a.M12 * b.M20 + a.M13 * b.M30,
                a.M10 * b.M01 + a.M11 * b.M11 + a.M12 * b.M21 + a.M13 * b.M31,
                a.M10 * b.M02 + a.M11 * b.M12 + a.M12 * b.M22 + a.M13 * b.M32,
                a.M10 * b.M03 + a.M11 * b.M13 + a.M12 * b.M23 + a.M13 * b.M33,

                a.M20 * b.M00 + a.M21 * b.M10 + a.M22 * b.M20 + a.M23 * b.M30,
                a.M20 * b.M01 + a.M21 * b.M11 + a.M22 * b.M21 + a.M23 * b.M31,
                a.M20 * b.M02 + a.M21 * b.M12 + a.M22 * b.M22 + a.M23 * b.M32,
                a.M20 * b.M03 + a.M21 * b.M13 + a.M22 * b.M23 + a.M23 * b.M33,

                a.M30 * b.M00 + a.M31 * b.M10 + a.M32 * b.M20 + a.M33 * b.M30,
                a.M30 * b.M01 + a.M31 * b.M11 + a.M32 * b.M21 + a.M33 * b.M31,
                a.M30 * b.M02 + a.M31 * b.M12 + a.M32 * b.M22 + a.M33 * b.M32,
                a.M30 * b.M03 + a.M31 * b.M13 + a.M32 * b.M23 + a.M33 * b.M33
            );
        }

        public static Vector operator *(Vector a, Matrix b)
        {
            return new Vector(
                a.X * b.M00 + a.Y * b.M10 + a.Z * b.M20 + a.W * b.M30,
                a.X * b.M01 + a.Y * b.M11 + a.Z * b.M21 + a.W * b.M31,
                a.X * b.M02 + a.Y * b.M12 + a.Z * b.M22 + a.W * b.M32,
                a.X * b.M03 + a.Y * b.M13 + a.Z * b.M23 + a.W * b.M33
            );
        }

        public static Vector operator *(Matrix a, Vector b)
        {
            return new Vector(
                a.M00 * b.X + a.M01 * b.Y + a.M02 * b.Z + a.M03 * b.W,
                a.M10 * b.X + a.M11 * b.Y + a.M12 * b.Z + a.M13 * b.W,
                a.M20 * b.X + a.M21 * b.Y + a.M22 * b.Z + a.M23 * b.W,
                a.M30 * b.X + a.M31 * b.Y + a.M32 * b.Z + a.M33 * b.W
            );
        }

        #endregion

        #region Transformations

        /// <summary>
        /// Create scale matrix
        /// </summary>
        public static Matrix Scale(Vector scaleVector)
        {
            return new Matrix(
                scaleVector.X, 0, 0, 0,
                0, scaleVector.Y, 0, 0,
                0, 0, scaleVector.Z, 0,
                0, 0, 0, 1);
        }

        /// <summary>
        /// Create translate matrix
        /// </summary>
        public static Matrix Translate(Vector translateVector)
        {
            return new Matrix(
                1, 0, 0, translateVector.X,
                0, 1, 0, translateVector.Y,
                0, 0, 1, translateVector.Z,
                0, 0, 0, 1);
        }

        /// <summary>
        /// Create rotate matrix in X-dir
        /// </summary>
        public static Matrix RotateX(double angleRad)
        {
            return new Matrix(
                1, 0, 0, 0,
                0, System.Math.Cos(angleRad), -System.Math.Sin(angleRad), 0,
                0, System.Math.Sin(angleRad), System.Math.Cos(angleRad), 0,
                0, 0, 0, 1);
        }

        /// <summary>
        /// Create rotate matrix in Y-dir
        /// </summary>
        public static Matrix RotateY(double angleRad)
        {
            return new Matrix(
                System.Math.Cos(angleRad), 0, System.Math.Sin(angleRad), 0,
                0, 1, 0, 0,
                -System.Math.Sin(angleRad), 0, System.Math.Cos(angleRad), 0,
                0, 0, 0, 1);
        }

        /// <summary>
        /// Create rotate matrix in Z-dir
        /// </summary>
        public static Matrix RotateZ(double angleRad)
        {
            angleRad = Angles.DegToRad(angleRad);
            return new Matrix(
                System.Math.Cos(angleRad), -System.Math.Sin(angleRad), 0, 0,
                System.Math.Sin(angleRad), System.Math.Cos(angleRad), 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1);
        }

        #endregion
    }
}
