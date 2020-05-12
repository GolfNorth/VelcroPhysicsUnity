using UnityEngine;
using VelcroPhysics.Shared;
using VelcroPhysics.Shared.Optimization;
using VelcroPhysics.Utilities;
using Transform = VelcroPhysics.Shared.Transform;

namespace VelcroPhysics.Collision.Narrowphase
{
    public static class WorldManifold
    {
        /// <summary>
        /// Evaluate the manifold with supplied transforms. This assumes
        /// modest motion from the original state. This does not change the
        /// point count, impulses, etc. The radii must come from the Shapes
        /// that generated the manifold.
        /// </summary>
        public static void Initialize(ref Manifold manifold, ref Transform xfA, float radiusA, ref Transform xfB,
            float radiusB, out Vector2 normal, out FixedArray2<Vector2> points, out FixedArray2<float> separations)
        {
            normal = Vector2.zero;
            points = new FixedArray2<Vector2>();
            separations = new FixedArray2<float>();

            if (manifold.PointCount == 0) return;

            switch (manifold.Type)
            {
                case ManifoldType.Circles:
                {
                    normal = new Vector2(1.0f, 0.0f);
                    var pointA = MathUtils.Mul(ref xfA, manifold.LocalPoint);
                    var pointB = MathUtils.Mul(ref xfB, manifold.Points.Value0.LocalPoint);
                    if (Mathf.Sqrt(Vector2.Distance(pointA, pointB)) > Settings.Epsilon * Settings.Epsilon)
                    {
                        normal = pointB - pointA;
                        normal.Normalize();
                    }

                    var cA = pointA + radiusA * normal;
                    var cB = pointB - radiusB * normal;
                    points.Value0 = 0.5f * (cA + cB);
                    separations.Value0 = Vector2.Dot(cB - cA, normal);
                }
                    break;

                case ManifoldType.FaceA:
                {
                    normal = MathUtils.Mul(xfA.q, manifold.LocalNormal);
                    var planePoint = MathUtils.Mul(ref xfA, manifold.LocalPoint);

                    for (var i = 0; i < manifold.PointCount; ++i)
                    {
                        var clipPoint = MathUtils.Mul(ref xfB, manifold.Points[i].LocalPoint);
                        var cA = clipPoint + (radiusA - Vector2.Dot(clipPoint - planePoint, normal)) * normal;
                        var cB = clipPoint - radiusB * normal;
                        points[i] = 0.5f * (cA + cB);
                        separations[i] = Vector2.Dot(cB - cA, normal);
                    }
                }
                    break;

                case ManifoldType.FaceB:
                {
                    normal = MathUtils.Mul(xfB.q, manifold.LocalNormal);
                    var planePoint = MathUtils.Mul(ref xfB, manifold.LocalPoint);

                    for (var i = 0; i < manifold.PointCount; ++i)
                    {
                        var clipPoint = MathUtils.Mul(ref xfA, manifold.Points[i].LocalPoint);
                        var cB = clipPoint + (radiusB - Vector2.Dot(clipPoint - planePoint, normal)) * normal;
                        var cA = clipPoint - radiusA * normal;
                        points[i] = 0.5f * (cA + cB);
                        separations[i] = Vector2.Dot(cA - cB, normal);
                    }

                    // Ensure normal points from A to B.
                    normal = -normal;
                }
                    break;
            }
        }
    }
}