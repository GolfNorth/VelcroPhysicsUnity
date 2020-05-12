/*
* Velcro Physics:
* Copyright (c) 2017 Ian Qvist
*/

using System.Collections.Generic;
using UnityEngine;
using VelcroPhysics.Shared;
using VelcroPhysics.Tools.Triangulation.Delaunay.Delaunay;
using VelcroPhysics.Tools.Triangulation.Delaunay.Delaunay.Sweep;

namespace VelcroPhysics.Tools.Triangulation.Delaunay
{
    /// <summary>
    /// 2D constrained Delaunay triangulation algorithm.
    /// Based on the paper "Sweep-line algorithm for constrained Delaunay triangulation" by V. Domiter and and B. Zalik
    /// 
    /// Properties:
    /// - Creates triangles with a large interior angle.
    /// - Supports holes
    /// - Generate a lot of garbage due to incapsulation of the Poly2Tri library.
    /// - Running time is O(n^2), n = number of vertices.
    /// - Does not care about winding order.
    /// 
    /// Source: http://code.google.com/p/poly2tri/
    /// </summary>
    internal static class CDTDecomposer
    {
        /// <summary>
        /// Decompose the polygon into several smaller non-concave polygon.
        /// </summary>
        public static List<Vertices> ConvexPartition(Vertices vertices)
        {
            Debug.Assert(vertices.Count > 3);

            var poly = new Polygon.Polygon();

            foreach (var vertex in vertices)
                poly.Points.Add(new TriangulationPoint(vertex.x, vertex.y));

            if (vertices.Holes != null)
                foreach (var holeVertices in vertices.Holes)
                {
                    var hole = new Polygon.Polygon();

                    foreach (var vertex in holeVertices)
                        hole.Points.Add(new TriangulationPoint(vertex.x, vertex.y));

                    poly.AddHole(hole);
                }

            var tcx = new DTSweepContext();
            tcx.PrepareTriangulation(poly);
            DTSweep.Triangulate(tcx);

            var results = new List<Vertices>();

            foreach (var triangle in poly.Triangles)
            {
                var v = new Vertices();
                foreach (var p in triangle.Points) v.Add(new Vector2(p.X, p.Y));
                results.Add(v);
            }

            return results;
        }
    }
}