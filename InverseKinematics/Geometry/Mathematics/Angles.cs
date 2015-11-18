using System;

namespace InverseKinematics.Geometry.Mathematics
{
	public static class Angles
	{
        /// <summary>
        /// Convert numbers to radians.
        /// </summary>
		public static double DegToRad(double angleDeg)
		{
            return angleDeg * Math.PI / 180;
		}

        /// <summary>
        /// Convert radians to numbers.
        /// </summary>
		public static double RadToDeg(double angleRad)
		{
            return angleRad * 180 / Math.PI;
		}

        /// <summary>
        /// Given an angle in radians, returns it's shifted version between 0 and 2PI
        /// </summary>
	    public static double AdjustAngle(double angleRad)
	    {
	        if (angleRad < 0) angleRad += Math.PI * 2;
            if (angleRad > Math.PI * 2) angleRad -= Math.PI * 2;
	        return angleRad;
	    }

        /// <summary>
        /// Computes the angle between two points
        /// </summary>
	    public static double ComputeAngle(Vector v1, Vector v2)
	    {
	        return AdjustAngle(Math.Atan2(v2.Y - v1.Y, v2.X - v1.X));
	    }
	}
}
