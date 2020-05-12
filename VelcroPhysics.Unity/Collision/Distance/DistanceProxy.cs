using System;
using UnityEngine;
using VelcroPhysics.Collision.Shapes;

namespace VelcroPhysics.Collision.Distance
{
    /// <summary>
    /// A distance proxy is used by the GJK algorithm.
    /// It encapsulates any shape.
    /// </summary>
    public struct DistanceProxy
    {
        internal readonly float Radius;
        internal readonly Vector2[] Vertices;

        public DistanceProxy(Shape shape, int index)
        {
            switch (shape.ShapeType)
            {
                case ShapeType.Circle:
                {
                    var circle = (CircleShape) shape;
                    Vertices = new Vector2[1];
                    Vertices[0] = circle.Position;
                    Radius = circle.Radius;
                }
                    break;

                case ShapeType.Polygon:
                {
                    var polygon = (PolygonShape) shape;
                    Vertices = new Vector2[polygon.Vertices.Count];

                    for (var i = 0; i < polygon.Vertices.Count; i++) Vertices[i] = polygon.Vertices[i];

                    Radius = polygon.Radius;
                }
                    break;

                case ShapeType.Chain:
                {
                    var chain = (ChainShape) shape;
                    Debug.Assert(0 <= index && index < chain.Vertices.Count);

                    Vertices = new Vector2[2];
                    Vertices[0] = chain.Vertices[index];
                    Vertices[1] = index + 1 < chain.Vertices.Count ? chain.Vertices[index + 1] : chain.Vertices[0];

                    Radius = chain.Radius;
                }
                    break;

                case ShapeType.Edge:
                {
                    var edge = (EdgeShape) shape;
                    Vertices = new Vector2[2];
                    Vertices[0] = edge.Vertex1;
                    Vertices[1] = edge.Vertex2;
                    Radius = edge.Radius;
                }
                    break;

                default:
                    throw new NotSupportedException();
            }
        }

        /// <summary>
        /// Get the supporting vertex index in the given direction.
        /// </summary>
        /// <param name="direction">The direction.</param>
        public int GetSupport(Vector2 direction)
        {
            var bestIndex = 0;
            var bestValue = Vector2.Dot(Vertices[0], direction);
            for (var i = 1; i < Vertices.Length; ++i)
            {
                var value = Vector2.Dot(Vertices[i], direction);
                if (value > bestValue)
                {
                    bestIndex = i;
                    bestValue = value;
                }
            }

            return bestIndex;
        }
    }
}