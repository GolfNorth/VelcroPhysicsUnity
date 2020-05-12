using UnityEngine;

namespace VelcroPhysics.Unity.Utilities
{
    public static class MatrixUtils
    {
        public static void CreateRotationZ(this ref Matrix4x4 matrix, float radians)
        {
            matrix.m00 = Mathf.Cos(radians);
            matrix.m01 = Mathf.Sin(radians);
            matrix.m10 = -matrix.m01;
            matrix.m11 = matrix.m00;
        }
    }
}