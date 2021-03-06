using UnityEngine;
using VelcroPhysics.Collision.ContactSystem;
using VelcroPhysics.Collision.Shapes;
using VelcroPhysics.Shared.Optimization;
using VelcroPhysics.Utilities;
using Transform = VelcroPhysics.Shared.Transform;

namespace VelcroPhysics.Collision.Narrowphase
{
    public static class EPCollider
    {
        public static void Collide(ref Manifold manifold, EdgeShape edgeA, ref Transform xfA, PolygonShape polygonB,
            ref Transform xfB)
        {
            // Algorithm:
            // 1. Classify v1 and v2
            // 2. Classify polygon centroid as front or back
            // 3. Flip normal if necessary
            // 4. Initialize normal range to [-pi, pi] about face normal
            // 5. Adjust normal range according to adjacent edges
            // 6. Visit each separating axes, only accept axes within the range
            // 7. Return if _any_ axis indicates separation
            // 8. Clip
            bool front;
            Vector2 lowerLimit, upperLimit;
            Vector2 normal;
            var normal0 = Vector2.zero;
            var normal2 = Vector2.zero;

            var xf = MathUtils.MulT(xfA, xfB);

            var centroidB = MathUtils.Mul(ref xf, polygonB.MassData.Centroid);

            var v0 = edgeA.Vertex0;
            var v1 = edgeA._vertex1;
            var v2 = edgeA._vertex2;
            var v3 = edgeA.Vertex3;

            var hasVertex0 = edgeA.HasVertex0;
            var hasVertex3 = edgeA.HasVertex3;

            var edge1 = v2 - v1;
            edge1.Normalize();
            var normal1 = new Vector2(edge1.y, -edge1.x);
            var offset1 = Vector2.Dot(normal1, centroidB - v1);
            float offset0 = 0.0f, offset2 = 0.0f;
            bool convex1 = false, convex2 = false;

            // Is there a preceding edge?
            if (hasVertex0)
            {
                var edge0 = v1 - v0;
                edge0.Normalize();
                normal0 = new Vector2(edge0.y, -edge0.x);
                convex1 = MathUtils.Cross(edge0, edge1) >= 0.0f;
                offset0 = Vector2.Dot(normal0, centroidB - v0);
            }

            // Is there a following edge?
            if (hasVertex3)
            {
                var edge2 = v3 - v2;
                edge2.Normalize();
                normal2 = new Vector2(edge2.y, -edge2.x);
                convex2 = MathUtils.Cross(edge1, edge2) > 0.0f;
                offset2 = Vector2.Dot(normal2, centroidB - v2);
            }

            // Determine front or back collision. Determine collision normal limits.
            if (hasVertex0 && hasVertex3)
            {
                if (convex1 && convex2)
                {
                    front = offset0 >= 0.0f || offset1 >= 0.0f || offset2 >= 0.0f;
                    if (front)
                    {
                        normal = normal1;
                        lowerLimit = normal0;
                        upperLimit = normal2;
                    }
                    else
                    {
                        normal = -normal1;
                        lowerLimit = -normal1;
                        upperLimit = -normal1;
                    }
                }
                else if (convex1)
                {
                    front = offset0 >= 0.0f || offset1 >= 0.0f && offset2 >= 0.0f;
                    if (front)
                    {
                        normal = normal1;
                        lowerLimit = normal0;
                        upperLimit = normal1;
                    }
                    else
                    {
                        normal = -normal1;
                        lowerLimit = -normal2;
                        upperLimit = -normal1;
                    }
                }
                else if (convex2)
                {
                    front = offset2 >= 0.0f || offset0 >= 0.0f && offset1 >= 0.0f;
                    if (front)
                    {
                        normal = normal1;
                        lowerLimit = normal1;
                        upperLimit = normal2;
                    }
                    else
                    {
                        normal = -normal1;
                        lowerLimit = -normal1;
                        upperLimit = -normal0;
                    }
                }
                else
                {
                    front = offset0 >= 0.0f && offset1 >= 0.0f && offset2 >= 0.0f;
                    if (front)
                    {
                        normal = normal1;
                        lowerLimit = normal1;
                        upperLimit = normal1;
                    }
                    else
                    {
                        normal = -normal1;
                        lowerLimit = -normal2;
                        upperLimit = -normal0;
                    }
                }
            }
            else if (hasVertex0)
            {
                if (convex1)
                {
                    front = offset0 >= 0.0f || offset1 >= 0.0f;
                    if (front)
                    {
                        normal = normal1;
                        lowerLimit = normal0;
                        upperLimit = -normal1;
                    }
                    else
                    {
                        normal = -normal1;
                        lowerLimit = normal1;
                        upperLimit = -normal1;
                    }
                }
                else
                {
                    front = offset0 >= 0.0f && offset1 >= 0.0f;
                    if (front)
                    {
                        normal = normal1;
                        lowerLimit = normal1;
                        upperLimit = -normal1;
                    }
                    else
                    {
                        normal = -normal1;
                        lowerLimit = normal1;
                        upperLimit = -normal0;
                    }
                }
            }
            else if (hasVertex3)
            {
                if (convex2)
                {
                    front = offset1 >= 0.0f || offset2 >= 0.0f;
                    if (front)
                    {
                        normal = normal1;
                        lowerLimit = -normal1;
                        upperLimit = normal2;
                    }
                    else
                    {
                        normal = -normal1;
                        lowerLimit = -normal1;
                        upperLimit = normal1;
                    }
                }
                else
                {
                    front = offset1 >= 0.0f && offset2 >= 0.0f;
                    if (front)
                    {
                        normal = normal1;
                        lowerLimit = -normal1;
                        upperLimit = normal1;
                    }
                    else
                    {
                        normal = -normal1;
                        lowerLimit = -normal2;
                        upperLimit = normal1;
                    }
                }
            }
            else
            {
                front = offset1 >= 0.0f;
                if (front)
                {
                    normal = normal1;
                    lowerLimit = -normal1;
                    upperLimit = -normal1;
                }
                else
                {
                    normal = -normal1;
                    lowerLimit = normal1;
                    upperLimit = normal1;
                }
            }

            // Get polygonB in frameA
            var normals = new Vector2[Settings.MaxPolygonVertices];
            var vertices = new Vector2[Settings.MaxPolygonVertices];
            var count = polygonB.Vertices.Count;
            for (var i = 0; i < polygonB.Vertices.Count; ++i)
            {
                vertices[i] = MathUtils.Mul(ref xf, polygonB.Vertices[i]);
                normals[i] = MathUtils.Mul(xf.q, polygonB.Normals[i]);
            }

            var radius = polygonB.Radius + edgeA.Radius;

            manifold.PointCount = 0;

            //Velcro: ComputeEdgeSeparation() was manually inlined here
            EPAxis edgeAxis;
            edgeAxis.Type = EPAxisType.EdgeA;
            edgeAxis.Index = front ? 0 : 1;
            edgeAxis.Separation = Settings.MaxFloat;

            for (var i = 0; i < count; ++i)
            {
                var s = Vector2.Dot(normal, vertices[i] - v1);
                if (s < edgeAxis.Separation) edgeAxis.Separation = s;
            }

            // If no valid normal can be found than this edge should not collide.
            if (edgeAxis.Type == EPAxisType.Unknown) return;

            if (edgeAxis.Separation > radius) return;

            //Velcro: ComputePolygonSeparation() was manually inlined here
            EPAxis polygonAxis;
            polygonAxis.Type = EPAxisType.Unknown;
            polygonAxis.Index = -1;
            polygonAxis.Separation = -Settings.MaxFloat;

            var perp = new Vector2(-normal.y, normal.x);

            for (var i = 0; i < count; ++i)
            {
                var n = -normals[i];

                var s1 = Vector2.Dot(n, vertices[i] - v1);
                var s2 = Vector2.Dot(n, vertices[i] - v2);
                var s = Mathf.Min(s1, s2);

                if (s > radius)
                {
                    // No collision
                    polygonAxis.Type = EPAxisType.EdgeB;
                    polygonAxis.Index = i;
                    polygonAxis.Separation = s;
                    break;
                }

                // Adjacency
                if (Vector2.Dot(n, perp) >= 0.0f)
                {
                    if (Vector2.Dot(n - upperLimit, normal) < -Settings.AngularSlop) continue;
                }
                else
                {
                    if (Vector2.Dot(n - lowerLimit, normal) < -Settings.AngularSlop) continue;
                }

                if (s > polygonAxis.Separation)
                {
                    polygonAxis.Type = EPAxisType.EdgeB;
                    polygonAxis.Index = i;
                    polygonAxis.Separation = s;
                }
            }

            if (polygonAxis.Type != EPAxisType.Unknown && polygonAxis.Separation > radius) return;

            // Use hysteresis for jitter reduction.
            const float k_relativeTol = 0.98f;
            const float k_absoluteTol = 0.001f;

            EPAxis primaryAxis;
            if (polygonAxis.Type == EPAxisType.Unknown)
                primaryAxis = edgeAxis;
            else if (polygonAxis.Separation > k_relativeTol * edgeAxis.Separation + k_absoluteTol)
                primaryAxis = polygonAxis;
            else
                primaryAxis = edgeAxis;

            var ie = new FixedArray2<ClipVertex>();
            ReferenceFace rf;
            if (primaryAxis.Type == EPAxisType.EdgeA)
            {
                manifold.Type = ManifoldType.FaceA;

                // Search for the polygon normal that is most anti-parallel to the edge normal.
                var bestIndex = 0;
                var bestValue = Vector2.Dot(normal, normals[0]);
                for (var i = 1; i < count; ++i)
                {
                    var value = Vector2.Dot(normal, normals[i]);
                    if (value < bestValue)
                    {
                        bestValue = value;
                        bestIndex = i;
                    }
                }

                var i1 = bestIndex;
                var i2 = i1 + 1 < count ? i1 + 1 : 0;

                ie.Value0.V = vertices[i1];
                ie.Value0.ID.ContactFeature.IndexA = 0;
                ie.Value0.ID.ContactFeature.IndexB = (byte) i1;
                ie.Value0.ID.ContactFeature.TypeA = ContactFeatureType.Face;
                ie.Value0.ID.ContactFeature.TypeB = ContactFeatureType.Vertex;

                ie.Value1.V = vertices[i2];
                ie.Value1.ID.ContactFeature.IndexA = 0;
                ie.Value1.ID.ContactFeature.IndexB = (byte) i2;
                ie.Value1.ID.ContactFeature.TypeA = ContactFeatureType.Face;
                ie.Value1.ID.ContactFeature.TypeB = ContactFeatureType.Vertex;

                if (front)
                {
                    rf.i1 = 0;
                    rf.i2 = 1;
                    rf.v1 = v1;
                    rf.v2 = v2;
                    rf.Normal = normal1;
                }
                else
                {
                    rf.i1 = 1;
                    rf.i2 = 0;
                    rf.v1 = v2;
                    rf.v2 = v1;
                    rf.Normal = -normal1;
                }
            }
            else
            {
                manifold.Type = ManifoldType.FaceB;

                ie.Value0.V = v1;
                ie.Value0.ID.ContactFeature.IndexA = 0;
                ie.Value0.ID.ContactFeature.IndexB = (byte) primaryAxis.Index;
                ie.Value0.ID.ContactFeature.TypeA = ContactFeatureType.Vertex;
                ie.Value0.ID.ContactFeature.TypeB = ContactFeatureType.Face;

                ie.Value1.V = v2;
                ie.Value1.ID.ContactFeature.IndexA = 0;
                ie.Value1.ID.ContactFeature.IndexB = (byte) primaryAxis.Index;
                ie.Value1.ID.ContactFeature.TypeA = ContactFeatureType.Vertex;
                ie.Value1.ID.ContactFeature.TypeB = ContactFeatureType.Face;

                rf.i1 = primaryAxis.Index;
                rf.i2 = rf.i1 + 1 < count ? rf.i1 + 1 : 0;
                rf.v1 = vertices[rf.i1];
                rf.v2 = vertices[rf.i2];
                rf.Normal = normals[rf.i1];
            }

            rf.SideNormal1 = new Vector2(rf.Normal.y, -rf.Normal.x);
            rf.SideNormal2 = -rf.SideNormal1;
            rf.SideOffset1 = Vector2.Dot(rf.SideNormal1, rf.v1);
            rf.SideOffset2 = Vector2.Dot(rf.SideNormal2, rf.v2);

            // Clip incident edge against extruded edge1 side edges.
            FixedArray2<ClipVertex> clipPoints1;
            FixedArray2<ClipVertex> clipPoints2;
            int np;

            // Clip to box side 1
            np = Collision.ClipSegmentToLine(out clipPoints1, ref ie, rf.SideNormal1, rf.SideOffset1, rf.i1);

            if (np < Settings.MaxManifoldPoints) return;

            // Clip to negative box side 1
            np = Collision.ClipSegmentToLine(out clipPoints2, ref clipPoints1, rf.SideNormal2, rf.SideOffset2, rf.i2);

            if (np < Settings.MaxManifoldPoints) return;

            // Now clipPoints2 contains the clipped points.
            if (primaryAxis.Type == EPAxisType.EdgeA)
            {
                manifold.LocalNormal = rf.Normal;
                manifold.LocalPoint = rf.v1;
            }
            else
            {
                manifold.LocalNormal = polygonB.Normals[rf.i1];
                manifold.LocalPoint = polygonB.Vertices[rf.i1];
            }

            var pointCount = 0;
            for (var i = 0; i < Settings.MaxManifoldPoints; ++i)
            {
                var separation = Vector2.Dot(rf.Normal, clipPoints2[i].V - rf.v1);

                if (separation <= radius)
                {
                    var cp = manifold.Points[pointCount];

                    if (primaryAxis.Type == EPAxisType.EdgeA)
                    {
                        cp.LocalPoint = MathUtils.MulT(ref xf, clipPoints2[i].V);
                        cp.Id = clipPoints2[i].ID;
                    }
                    else
                    {
                        cp.LocalPoint = clipPoints2[i].V;
                        cp.Id.ContactFeature.TypeA = clipPoints2[i].ID.ContactFeature.TypeB;
                        cp.Id.ContactFeature.TypeB = clipPoints2[i].ID.ContactFeature.TypeA;
                        cp.Id.ContactFeature.IndexA = clipPoints2[i].ID.ContactFeature.IndexB;
                        cp.Id.ContactFeature.IndexB = clipPoints2[i].ID.ContactFeature.IndexA;
                    }

                    manifold.Points[pointCount] = cp;
                    ++pointCount;
                }
            }

            manifold.PointCount = pointCount;
        }
    }
}