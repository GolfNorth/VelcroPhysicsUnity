using System;
using System.Collections.Generic;
using UnityEngine;
using VelcroPhysics.Shared;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Tools.PolygonManipulation
{
    /// <summary>
    /// Provides a set of tools to simplify polygons in various ways.
    /// </summary>
    public static class SimplifyTools
    {
        /// <summary>
        /// Removes all collinear points on the polygon.
        /// </summary>
        /// <param name="vertices">The polygon that needs simplification.</param>
        /// <param name="collinearityTolerance">The collinearity tolerance.</param>
        /// <returns>A simplified polygon.</returns>
        public static Vertices CollinearSimplify(Vertices vertices, float collinearityTolerance = 0)
        {
            if (vertices.Count <= 3)
                return vertices;

            var simplified = new Vertices(vertices.Count);

            for (var i = 0; i < vertices.Count; i++)
            {
                var prev = vertices.PreviousVertex(i);
                var current = vertices[i];
                var next = vertices.NextVertex(i);

                //If they collinear, continue
                if (MathUtils.IsCollinear(ref prev, ref current, ref next, collinearityTolerance))
                    continue;

                simplified.Add(current);
            }

            return simplified;
        }

        /// <summary>
        /// Ramer-Douglas-Peucker polygon simplification algorithm. This is the general recursive version that does not use the
        /// speed-up technique by using the Melkman convex hull.
        /// If you pass in 0, it will remove all collinear points.
        /// </summary>
        /// <returns>The simplified polygon</returns>
        public static Vertices DouglasPeuckerSimplify(Vertices vertices, float distanceTolerance)
        {
            if (vertices.Count <= 3)
                return vertices;

            var usePoint = new bool[vertices.Count];

            for (var i = 0; i < vertices.Count; i++)
                usePoint[i] = true;

            SimplifySection(vertices, 0, vertices.Count - 1, usePoint, distanceTolerance);

            var simplified = new Vertices(vertices.Count);

            for (var i = 0; i < vertices.Count; i++)
                if (usePoint[i])
                    simplified.Add(vertices[i]);

            return simplified;
        }

        private static void SimplifySection(Vertices vertices, int i, int j, bool[] usePoint, float distanceTolerance)
        {
            if (i + 1 == j)
                return;

            var a = vertices[i];
            var b = vertices[j];

            var maxDistance = -1.0f;
            var maxIndex = i;
            for (var k = i + 1; k < j; k++)
            {
                var point = vertices[k];

                var distance = LineUtils.DistanceBetweenPointAndLineSegment(ref point, ref a, ref b);

                if (distance > maxDistance)
                {
                    maxDistance = distance;
                    maxIndex = k;
                }
            }

            if (maxDistance <= distanceTolerance)
            {
                for (var k = i + 1; k < j; k++) usePoint[k] = false;
            }
            else
            {
                SimplifySection(vertices, i, maxIndex, usePoint, distanceTolerance);
                SimplifySection(vertices, maxIndex, j, usePoint, distanceTolerance);
            }
        }

        /// <summary>
        /// Merges all parallel edges in the list of vertices
        /// </summary>
        /// <param name="vertices">The vertices.</param>
        /// <param name="tolerance">The tolerance.</param>
        public static Vertices MergeParallelEdges(Vertices vertices, float tolerance)
        {
            //From Eric Jordan's convex decomposition library

            if (vertices.Count <= 3)
                return vertices; //Can't do anything useful here to a triangle

            var mergeMe = new bool[vertices.Count];
            var newNVertices = vertices.Count;

            //Gather points to process
            for (var i = 0; i < vertices.Count; ++i)
            {
                var lower = i == 0 ? vertices.Count - 1 : i - 1;
                var middle = i;
                var upper = i == vertices.Count - 1 ? 0 : i + 1;

                var dx0 = vertices[middle].x - vertices[lower].x;
                var dy0 = vertices[middle].y - vertices[lower].y;
                var dx1 = vertices[upper].x - vertices[middle].x;
                var dy1 = vertices[upper].y - vertices[middle].y;
                var norm0 = Mathf.Sqrt(dx0 * dx0 + dy0 * dy0);
                var norm1 = Mathf.Sqrt(dx1 * dx1 + dy1 * dy1);

                if (!(norm0 > 0.0f && norm1 > 0.0f) && newNVertices > 3)
                {
                    //Merge identical points
                    mergeMe[i] = true;
                    --newNVertices;
                }

                dx0 /= norm0;
                dy0 /= norm0;
                dx1 /= norm1;
                dy1 /= norm1;
                var cross = dx0 * dy1 - dx1 * dy0;
                var dot = dx0 * dx1 + dy0 * dy1;

                if (Mathf.Abs(cross) < tolerance && dot > 0 && newNVertices > 3)
                {
                    mergeMe[i] = true;
                    --newNVertices;
                }
                else
                {
                    mergeMe[i] = false;
                }
            }

            if (newNVertices == vertices.Count || newNVertices == 0)
                return vertices;

            var currIndex = 0;

            //Copy the vertices to a new list and clear the old
            var newVertices = new Vertices(newNVertices);

            for (var i = 0; i < vertices.Count; ++i)
            {
                if (mergeMe[i] || currIndex == newNVertices)
                    continue;

                Debug.Assert(currIndex < newNVertices);

                newVertices.Add(vertices[i]);
                ++currIndex;
            }

            return newVertices;
        }

        /// <summary>
        /// Merges the identical points in the polygon.
        /// </summary>
        /// <param name="vertices">The vertices.</param>
        public static Vertices MergeIdenticalPoints(Vertices vertices)
        {
            var unique = new HashSet<Vector2>();

            foreach (var vertex in vertices) unique.Add(vertex);

            return new Vertices(unique);
        }

        /// <summary>
        /// Reduces the polygon by distance.
        /// </summary>
        /// <param name="vertices">The vertices.</param>
        /// <param name="distance">The distance between points. Points closer than this will be removed.</param>
        public static Vertices ReduceByDistance(Vertices vertices, float distance)
        {
            if (vertices.Count <= 3)
                return vertices;

            var distance2 = distance * distance;

            var simplified = new Vertices(vertices.Count);

            for (var i = 0; i < vertices.Count; i++)
            {
                var current = vertices[i];
                var next = vertices.NextVertex(i);

                //If they are closer than the distance, continue
                if ((next - current).sqrMagnitude <= distance2)
                    continue;

                simplified.Add(current);
            }

            return simplified;
        }

        /// <summary>
        /// Reduces the polygon by removing the Nth vertex in the vertices list.
        /// </summary>
        /// <param name="vertices">The vertices.</param>
        /// <param name="nth">The Nth point to remove. Example: 5.</param>
        /// <returns></returns>
        public static Vertices ReduceByNth(Vertices vertices, int nth)
        {
            if (vertices.Count <= 3)
                return vertices;

            if (nth == 0)
                return vertices;

            var simplified = new Vertices(vertices.Count);

            for (var i = 0; i < vertices.Count; i++)
            {
                if (i % nth == 0)
                    continue;

                simplified.Add(vertices[i]);
            }

            return simplified;
        }

        /// <summary>
        /// Simplify the polygon by removing all points that in pairs of 3 have an area less than the tolerance.
        /// Pass in 0 as tolerance, and it will only remove collinear points.
        /// </summary>
        /// <param name="vertices"></param>
        /// <param name="areaTolerance"></param>
        /// <returns></returns>
        public static Vertices ReduceByArea(Vertices vertices, float areaTolerance)
        {
            //From physics2d.net

            if (vertices.Count <= 3)
                return vertices;

            if (areaTolerance < 0)
                throw new ArgumentOutOfRangeException(nameof(areaTolerance), "must be equal to or greater than zero.");

            var simplified = new Vertices(vertices.Count);
            Vector2 v3;
            var v1 = vertices[vertices.Count - 2];
            var v2 = vertices[vertices.Count - 1];
            areaTolerance *= 2;

            for (var i = 0; i < vertices.Count; ++i, v2 = v3)
            {
                v3 = i == vertices.Count - 1 ? simplified[0] : vertices[i];

                float old1;
                MathUtils.Cross(ref v1, ref v2, out old1);

                float old2;
                MathUtils.Cross(ref v2, ref v3, out old2);

                float new1;
                MathUtils.Cross(ref v1, ref v3, out new1);

                if (Mathf.Abs(new1 - (old1 + old2)) > areaTolerance)
                {
                    simplified.Add(v2);
                    v1 = v2;
                }
            }

            return simplified;
        }
    }
}