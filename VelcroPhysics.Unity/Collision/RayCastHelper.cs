using UnityEngine;
using VelcroPhysics.Shared;
using VelcroPhysics.Utilities;
using Transform = VelcroPhysics.Shared.Transform;

namespace VelcroPhysics.Collision.RayCast
{
    public static class RayCastHelper
    {
        public static bool RayCastEdge(ref Vector2 start, ref Vector2 end, ref RayCastInput input,
            ref Transform transform, out RayCastOutput output)
        {
            // p = p1 + t * d
            // v = v1 + s * e
            // p1 + t * d = v1 + s * e
            // s * e - t * d = p1 - v1

            output = new RayCastOutput();

            // Put the ray into the edge's frame of reference.
            var p1 = MathUtils.MulT(transform.q, input.Point1 - transform.p);
            var p2 = MathUtils.MulT(transform.q, input.Point2 - transform.p);
            var d = p2 - p1;

            var v1 = start;
            var v2 = end;
            var e = v2 - v1;
            var normal = new Vector2(e.y, -e.x); //TODO: Could possibly cache the normal.
            normal.Normalize();

            // q = p1 + t * d
            // dot(normal, q - v1) = 0
            // dot(normal, p1 - v1) + t * dot(normal, d) = 0
            var numerator = Vector2.Dot(normal, v1 - p1);
            var denominator = Vector2.Dot(normal, d);

            if (denominator == 0.0f) return false;

            var t = numerator / denominator;
            if (t < 0.0f || input.MaxFraction < t) return false;

            var q = p1 + t * d;

            // q = v1 + s * r
            // s = dot(q - v1, r) / dot(r, r)
            var r = v2 - v1;
            var rr = Vector2.Dot(r, r);
            if (rr == 0.0f) return false;

            var s = Vector2.Dot(q - v1, r) / rr;
            if (s < 0.0f || 1.0f < s) return false;

            output.Fraction = t;
            if (numerator > 0.0f)
                output.Normal = -MathUtils.MulT(transform.q, normal);
            else
                output.Normal = MathUtils.MulT(transform.q, normal);
            return true;
        }

        public static bool RayCastCircle(ref Vector2 pos, float radius, ref RayCastInput input, ref Transform transform,
            out RayCastOutput output)
        {
            // Collision Detection in Interactive 3D Environments by Gino van den Bergen
            // From Section 3.1.2
            // x = s + a * r
            // norm(x) = radius

            output = new RayCastOutput();

            var position = transform.p + MathUtils.Mul(transform.q, pos);
            var s = input.Point1 - position;
            var b = Vector2.Dot(s, s) - radius * radius;

            // Solve quadratic equation.
            var r = input.Point2 - input.Point1;
            var c = Vector2.Dot(s, r);
            var rr = Vector2.Dot(r, r);
            var sigma = c * c - rr * b;

            // Check for negative discriminant and short segment.
            if (sigma < 0.0f || rr < Settings.Epsilon) return false;

            // Find the point of intersection of the line with the circle.
            var a = -(c + Mathf.Sqrt(sigma));

            // Is the intersection point on the segment?
            if (0.0f <= a && a <= input.MaxFraction * rr)
            {
                a /= rr;
                output.Fraction = a;
                output.Normal = s + a * r;
                output.Normal.Normalize();
                return true;
            }

            return false;
        }

        public static bool RayCastPolygon(Vertices vertices, Vertices normals, ref RayCastInput input,
            ref Transform transform, out RayCastOutput output)
        {
            output = new RayCastOutput();

            // Put the ray into the polygon's frame of reference.
            var p1 = MathUtils.MulT(transform.q, input.Point1 - transform.p);
            var p2 = MathUtils.MulT(transform.q, input.Point2 - transform.p);
            var d = p2 - p1;

            float lower = 0.0f, upper = input.MaxFraction;

            var index = -1;

            for (var i = 0; i < vertices.Count; ++i)
            {
                // p = p1 + a * d
                // dot(normal, p - v) = 0
                // dot(normal, p1 - v) + a * dot(normal, d) = 0
                var numerator = Vector2.Dot(normals[i], vertices[i] - p1);
                var denominator = Vector2.Dot(normals[i], d);

                if (denominator == 0.0f)
                {
                    if (numerator < 0.0f) return false;
                }
                else
                {
                    // Note: we want this predicate without division:
                    // lower < numerator / denominator, where denominator < 0
                    // Since denominator < 0, we have to flip the inequality:
                    // lower < numerator / denominator <==> denominator * lower > numerator.
                    if (denominator < 0.0f && numerator < lower * denominator)
                    {
                        // Increase lower.
                        // The segment enters this half-space.
                        lower = numerator / denominator;
                        index = i;
                    }
                    else if (denominator > 0.0f && numerator < upper * denominator)
                    {
                        // Decrease upper.
                        // The segment exits this half-space.
                        upper = numerator / denominator;
                    }
                }

                // The use of epsilon here causes the assert on lower to trip
                // in some cases. Apparently the use of epsilon was to make edge
                // shapes work, but now those are handled separately.
                //if (upper < lower - b2_epsilon)
                if (upper < lower) return false;
            }

            Debug.Assert(0.0f <= lower && lower <= input.MaxFraction);

            if (index >= 0)
            {
                output.Fraction = lower;
                output.Normal = MathUtils.Mul(transform.q, normals[index]);
                return true;
            }

            return false;
        }
    }
}