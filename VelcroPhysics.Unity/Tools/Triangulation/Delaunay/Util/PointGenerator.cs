using System.Collections.Generic;
using UnityEngine;

namespace VelcroPhysics.Tools.Triangulation.Delaunay.Util
{
    internal class PointGenerator
    {
        public static List<TriangulationPoint> UniformDistribution(int n, float scale)
        {
            var points = new List<TriangulationPoint>();
            for (var i = 0; i < n; i++)
                points.Add(new TriangulationPoint(scale * (0.5f - Random.value), scale * (0.5f - Random.value)));
            return points;
        }

        public static List<TriangulationPoint> UniformGrid(int n, float scale)
        {
            float x = 0;
            var size = scale / n;
            var halfScale = 0.5f * scale;

            var points = new List<TriangulationPoint>();
            for (var i = 0; i < n + 1; i++)
            {
                x = halfScale - i * size;
                for (var j = 0; j < n + 1; j++) points.Add(new TriangulationPoint(x, halfScale - j * size));
            }

            return points;
        }
    }
}