using UnityEngine;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Shared
{
    /// <summary>
    /// A 3-by-3 matrix. Stored in column-major order.
    /// </summary>
    public struct Mat33
    {
        public Vector3 ex, ey, ez;

        /// <summary>
        /// Construct this matrix using columns.
        /// </summary>
        /// <param name="c1">The c1.</param>
        /// <param name="c2">The c2.</param>
        /// <param name="c3">The c3.</param>
        public Mat33(Vector3 c1, Vector3 c2, Vector3 c3)
        {
            ex = c1;
            ey = c2;
            ez = c3;
        }

        /// <summary>
        /// Set this matrix to all zeros.
        /// </summary>
        public void SetZero()
        {
            ex = Vector3.zero;
            ey = Vector3.zero;
            ez = Vector3.zero;
        }

        /// <summary>
        /// Solve A * x = b, where b is a column vector. This is more efficient
        /// than computing the inverse in one-shot cases.
        /// </summary>
        /// <param name="b">The b.</param>
        /// <returns></returns>
        public Vector3 Solve33(Vector3 b)
        {
            var det = Vector3.Dot(ex, Vector3.Cross(ey, ez));
            if (det != 0.0f) det = 1.0f / det;

            return new Vector3(det * Vector3.Dot(b, Vector3.Cross(ey, ez)), det * Vector3.Dot(ex, Vector3.Cross(b, ez)),
                det * Vector3.Dot(ex, Vector3.Cross(ey, b)));
        }

        /// <summary>
        /// Solve A * x = b, where b is a column vector. This is more efficient
        /// than computing the inverse in one-shot cases. Solve only the upper
        /// 2-by-2 matrix equation.
        /// </summary>
        /// <param name="b">The b.</param>
        /// <returns></returns>
        public Vector2 Solve22(Vector2 b)
        {
            float a11 = ex.x, a12 = ey.x, a21 = ex.y, a22 = ey.y;
            var det = a11 * a22 - a12 * a21;

            if (det != 0.0f) det = 1.0f / det;

            return new Vector2(det * (a22 * b.x - a12 * b.y), det * (a11 * b.y - a21 * b.x));
        }

        /// Get the inverse of this matrix as a 2-by-2.
        /// Returns the zero matrix if singular.
        public void GetInverse22(ref Mat33 M)
        {
            float a = ex.x, b = ey.x, c = ex.y, d = ey.y;
            var det = a * d - b * c;
            if (det != 0.0f) det = 1.0f / det;

            M.ex.x = det * d;
            M.ey.x = -det * b;
            M.ex.z = 0.0f;
            M.ex.y = -det * c;
            M.ey.y = det * a;
            M.ey.z = 0.0f;
            M.ez.x = 0.0f;
            M.ez.y = 0.0f;
            M.ez.z = 0.0f;
        }

        /// Get the symmetric inverse of this matrix as a 3-by-3.
        /// Returns the zero matrix if singular.
        public void GetSymInverse33(ref Mat33 M)
        {
            var det = MathUtils.Dot(ex, MathUtils.Cross((Vector3) ey, ez));
            if (det != 0.0f) det = 1.0f / det;

            float a11 = ex.x, a12 = ey.x, a13 = ez.x;
            float a22 = ey.y, a23 = ez.y;
            var a33 = ez.z;

            M.ex.x = det * (a22 * a33 - a23 * a23);
            M.ex.y = det * (a13 * a23 - a12 * a33);
            M.ex.z = det * (a12 * a23 - a13 * a22);

            M.ey.x = M.ex.y;
            M.ey.y = det * (a11 * a33 - a13 * a13);
            M.ey.z = det * (a13 * a12 - a11 * a23);

            M.ez.x = M.ex.z;
            M.ez.y = M.ey.z;
            M.ez.z = det * (a11 * a22 - a12 * a12);
        }
    }
}