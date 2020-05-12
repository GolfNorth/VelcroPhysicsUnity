using UnityEngine;
using VelcroPhysics.Collision.ContactSystem;
using VelcroPhysics.Collision.Shapes;
using VelcroPhysics.Utilities;
using Transform = VelcroPhysics.Shared.Transform;

namespace VelcroPhysics.Collision.Narrowphase
{
    public static class CollideEdge
    {
        /// <summary>
        /// Compute contact points for edge versus circle.
        /// This accounts for edge connectivity.
        /// </summary>
        /// <param name="manifold">The manifold.</param>
        /// <param name="edgeA">The edge A.</param>
        /// <param name="transformA">The transform A.</param>
        /// <param name="circleB">The circle B.</param>
        /// <param name="transformB">The transform B.</param>
        public static void CollideEdgeAndCircle(ref Manifold manifold, EdgeShape edgeA, ref Transform transformA,
            CircleShape circleB, ref Transform transformB)
        {
            manifold.PointCount = 0;

            // Compute circle in frame of edge
            var Q = MathUtils.MulT(ref transformA, MathUtils.Mul(ref transformB, ref circleB._position));

            Vector2 A = edgeA.Vertex1, B = edgeA.Vertex2;
            var e = B - A;

            // Barycentric coordinates
            var u = Vector2.Dot(e, B - Q);
            var v = Vector2.Dot(e, Q - A);

            var radius = edgeA.Radius + circleB.Radius;

            ContactFeature cf;
            cf.IndexB = 0;
            cf.TypeB = ContactFeatureType.Vertex;

            // Region A
            if (v <= 0.0f)
            {
                var P1 = A;
                var d1 = Q - P1;
                var dd1 = Vector2.Dot(d1, d1);
                if (dd1 > radius * radius) return;

                // Is there an edge connected to A?
                if (edgeA.HasVertex0)
                {
                    var A1 = edgeA.Vertex0;
                    var B1 = A;
                    var e1 = B1 - A1;
                    var u1 = Vector2.Dot(e1, B1 - Q);

                    // Is the circle in Region AB of the previous edge?
                    if (u1 > 0.0f) return;
                }

                cf.IndexA = 0;
                cf.TypeA = ContactFeatureType.Vertex;
                manifold.PointCount = 1;
                manifold.Type = ManifoldType.Circles;
                manifold.LocalNormal = Vector2.zero;
                manifold.LocalPoint = P1;
                manifold.Points.Value0.Id.Key = 0;
                manifold.Points.Value0.Id.ContactFeature = cf;
                manifold.Points.Value0.LocalPoint = circleB.Position;
                return;
            }

            // Region B
            if (u <= 0.0f)
            {
                var P2 = B;
                var d2 = Q - P2;
                var dd2 = Vector2.Dot(d2, d2);
                if (dd2 > radius * radius) return;

                // Is there an edge connected to B?
                if (edgeA.HasVertex3)
                {
                    var B2 = edgeA.Vertex3;
                    var A2 = B;
                    var e2 = B2 - A2;
                    var v2 = Vector2.Dot(e2, Q - A2);

                    // Is the circle in Region AB of the next edge?
                    if (v2 > 0.0f) return;
                }

                cf.IndexA = 1;
                cf.TypeA = (byte) ContactFeatureType.Vertex;
                manifold.PointCount = 1;
                manifold.Type = ManifoldType.Circles;
                manifold.LocalNormal = Vector2.zero;
                manifold.LocalPoint = P2;
                manifold.Points.Value0.Id.Key = 0;
                manifold.Points.Value0.Id.ContactFeature = cf;
                manifold.Points.Value0.LocalPoint = circleB.Position;
                return;
            }

            // Region AB
            var den = Vector2.Dot(e, e);
            Debug.Assert(den > 0.0f);
            var P = 1.0f / den * (u * A + v * B);
            var d = Q - P;
            var dd = Vector2.Dot(d, d);
            if (dd > radius * radius) return;

            var n = new Vector2(-e.y, e.x);
            if (Vector2.Dot(n, Q - A) < 0.0f) n = new Vector2(-n.x, -n.y);
            n.Normalize();

            cf.IndexA = 0;
            cf.TypeA = ContactFeatureType.Face;
            manifold.PointCount = 1;
            manifold.Type = ManifoldType.FaceA;
            manifold.LocalNormal = n;
            manifold.LocalPoint = A;
            manifold.Points.Value0.Id.Key = 0;
            manifold.Points.Value0.Id.ContactFeature = cf;
            manifold.Points.Value0.LocalPoint = circleB.Position;
        }

        /// <summary>
        /// Collides and edge and a polygon, taking into account edge adjacency.
        /// </summary>
        public static void CollideEdgeAndPolygon(ref Manifold manifold, EdgeShape edgeA, ref Transform xfA,
            PolygonShape polygonB, ref Transform xfB)
        {
            EPCollider.Collide(ref manifold, edgeA, ref xfA, polygonB, ref xfB);
        }
    }
}