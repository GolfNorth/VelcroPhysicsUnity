using UnityEngine;
using VelcroPhysics.Shared;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Tools.ConvexHull.GiftWrap
{
    /// <summary>
    /// Giftwrap convex hull algorithm.
    /// O(n * h) time complexity, where n is the number of points and h is the number of points on the convex hull.
    /// See http://en.wikipedia.org/wiki/Gift_wrapping_algorithm for more details.
    /// </summary>
    public static class GiftWrap
    {
        //Extracted from Box2D

        /// <summary>
        /// Returns the convex hull from the given vertices.
        /// </summary>
        /// <param name="vertices">The vertices.</param>
        public static Vertices GetConvexHull(Vertices vertices)
        {
            if (vertices.Count <= 3)
                return vertices;

            // Find the right most point on the hull
            var i0 = 0;
            var x0 = vertices[0].x;
            for (var i = 1; i < vertices.Count; ++i)
            {
                var x = vertices[i].x;
                if (x > x0 || x == x0 && vertices[i].y < vertices[i0].y)
                {
                    i0 = i;
                    x0 = x;
                }
            }

            var hull = new int[vertices.Count];
            var m = 0;
            var ih = i0;

            for (;;)
            {
                hull[m] = ih;

                var ie = 0;
                for (var j = 1; j < vertices.Count; ++j)
                {
                    if (ie == ih)
                    {
                        ie = j;
                        continue;
                    }

                    var r = vertices[ie] - vertices[hull[m]];
                    var v = vertices[j] - vertices[hull[m]];
                    var c = MathUtils.Cross(ref r, ref v);
                    if (c < 0.0f) ie = j;

                    // Collinearity check
                    if (c == 0.0f && v.sqrMagnitude > r.sqrMagnitude) ie = j;
                }

                ++m;
                ih = ie;

                if (ie == i0) break;
            }

            var result = new Vertices(m);

            // Copy vertices.
            for (var i = 0; i < m; ++i) result.Add(vertices[hull[i]]);
            return result;
        }
    }
}