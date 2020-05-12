/*
* Velcro Physics:
* Copyright (c) 2017 Ian Qvist
*/

using System.Collections.Generic;
using UnityEngine;
using VelcroPhysics.Shared;

namespace VelcroPhysics.Tools.Triangulation.Seidel
{
    /// <summary>
    /// Convex decomposition algorithm created by Raimund Seidel
    /// Properties:
    /// - Decompose the polygon into trapezoids, then triangulate.
    /// - To use the trapezoid data, use ConvexPartitionTrapezoid()
    /// - Generate a lot of garbage due to incapsulation of the Poly2Tri library.
    /// - Running time is O(n log n), n = number of vertices.
    /// - Running time is almost linear for most simple polygons.
    /// - Does not care about winding order.
    /// For more information, see Raimund Seidel's paper "A simple and fast incremental randomized
    /// algorithm for computing trapezoidal decompositions and for triangulating polygons"
    /// See also: "Computational Geometry", 3rd edition, by Mark de Berg et al, Chapter 6.2
    /// "Computational Geometry in C", 2nd edition, by Joseph O'Rourke
    /// Original code from the Poly2Tri project by Mason Green.
    /// http://code.google.com/p/poly2tri/source/browse?repo=archive#hg/scala/src/org/poly2tri/seidel
    /// This implementation is from Dec 14, 2010
    /// </summary>
    internal static class SeidelDecomposer
    {
        /// <summary>
        /// Decompose the polygon into several smaller non-concave polygons.
        /// </summary>
        /// <param name="vertices">The polygon to decompose.</param>
        /// <param name="sheer">The sheer to use if you get bad results, try using a higher value.</param>
        /// <returns>A list of triangles</returns>
        public static List<Vertices> ConvexPartition(Vertices vertices, float sheer = 0.001f)
        {
            Debug.Assert(vertices.Count > 3);

            var compatList = new List<Point>(vertices.Count);

            foreach (var vertex in vertices) compatList.Add(new Point(vertex.x, vertex.y));

            var t = new Triangulator(compatList, sheer);

            var list = new List<Vertices>();

            foreach (var triangle in t.Triangles)
            {
                var outTriangles = new Vertices(triangle.Count);

                foreach (var outTriangle in triangle) outTriangles.Add(new Vector2(outTriangle.X, outTriangle.Y));

                list.Add(outTriangles);
            }

            return list;
        }

        /// <summary>
        /// Decompose the polygon into several smaller non-concave polygons.
        /// </summary>
        /// <param name="vertices">The polygon to decompose.</param>
        /// <param name="sheer">The sheer to use if you get bad results, try using a higher value.</param>
        /// <returns>A list of trapezoids</returns>
        public static List<Vertices> ConvexPartitionTrapezoid(Vertices vertices, float sheer = 0.001f)
        {
            var compatList = new List<Point>(vertices.Count);

            foreach (var vertex in vertices) compatList.Add(new Point(vertex.x, vertex.y));

            var t = new Triangulator(compatList, sheer);

            var list = new List<Vertices>();

            foreach (var trapezoid in t.Trapezoids)
            {
                var verts = new Vertices();

                var points = trapezoid.GetVertices();
                foreach (var point in points) verts.Add(new Vector2(point.X, point.Y));

                list.Add(verts);
            }

            return list;
        }
    }
}