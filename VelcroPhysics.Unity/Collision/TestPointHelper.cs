using UnityEngine;
using VelcroPhysics.Shared;
using VelcroPhysics.Utilities;
using Transform = VelcroPhysics.Shared.Transform;

namespace VelcroPhysics.Collision
{
    public static class TestPointHelper
    {
        public static bool TestPointCircle(ref Vector2 pos, float radius, ref Vector2 point, ref Transform transform)
        {
            var center = transform.p + MathUtils.Mul(transform.q, pos);
            var d = point - center;
            return Vector2.Dot(d, d) <= radius * radius;
        }

        public static bool TestPointPolygon(Vertices vertices, Vertices normals, ref Vector2 point,
            ref Transform transform)
        {
            var pLocal = MathUtils.MulT(transform.q, point - transform.p);

            for (var i = 0; i < vertices.Count; ++i)
            {
                var dot = Vector2.Dot(normals[i], pLocal - vertices[i]);
                if (dot > 0.0f) return false;
            }

            return true;
        }
    }
}