using System;
using System.Collections.Generic;
using System.Text;
using UnityEngine;
using VelcroPhysics.Shared;
using VelcroPhysics.Unity.Utilities;
using VelcroPhysics.Utilities;

namespace VelcroPhysics.Tools.PathGenerator
{
    //Contributed by Matthew Bettcher

    /// <summary>
    /// Path:
    /// Very similar to Vertices, but this
    /// class contains vectors describing
    /// control points on a Catmull-Rom
    /// curve.
    /// </summary>
    public class Path
    {
        private float _deltaT;

        /// <summary>
        /// All the points that makes up the curve
        /// </summary>
        public List<Vector2> ControlPoints;

        /// <summary>
        /// Initializes a new instance of the <see cref="Path" /> class.
        /// </summary>
        public Path()
        {
            ControlPoints = new List<Vector2>();
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="Path" /> class.
        /// </summary>
        /// <param name="vertices">The vertices to created the path from.</param>
        public Path(Vector2[] vertices)
        {
            ControlPoints = new List<Vector2>(vertices.Length);

            for (var i = 0; i < vertices.Length; i++) Add(vertices[i]);
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="Path" /> class.
        /// </summary>
        /// <param name="vertices">The vertices to created the path from.</param>
        public Path(IList<Vector2> vertices)
        {
            ControlPoints = new List<Vector2>(vertices.Count);
            for (var i = 0; i < vertices.Count; i++) Add(vertices[i]);
        }

        /// <summary>
        /// True if the curve is closed.
        /// </summary>
        /// <value><c>true</c> if closed; otherwise, <c>false</c>.</value>
        public bool Closed { get; set; }

        /// <summary>
        /// Gets the next index of a controlpoint
        /// </summary>
        /// <param name="index">The index.</param>
        /// <returns></returns>
        public int NextIndex(int index)
        {
            if (index == ControlPoints.Count - 1) return 0;
            return index + 1;
        }

        /// <summary>
        /// Gets the previous index of a controlpoint
        /// </summary>
        /// <param name="index">The index.</param>
        /// <returns></returns>
        public int PreviousIndex(int index)
        {
            if (index == 0) return ControlPoints.Count - 1;
            return index - 1;
        }

        /// <summary>
        /// Translates the control points by the specified vector.
        /// </summary>
        /// <param name="vector">The vector.</param>
        public void Translate(ref Vector2 vector)
        {
            for (var i = 0; i < ControlPoints.Count; i++)
                ControlPoints[i] = ControlPoints[i] + vector;
        }

        /// <summary>
        /// Scales the control points by the specified vector.
        /// </summary>
        /// <param name="value">The Value.</param>
        public void Scale(ref Vector2 value)
        {
            for (var i = 0; i < ControlPoints.Count; i++)
                ControlPoints[i] = ControlPoints[i] * value;
        }

        /// <summary>
        /// Rotate the control points by the defined value in radians.
        /// </summary>
        /// <param name="value">The amount to rotate by in radians.</param>
        public void Rotate(float value)
        {
            var rotationMatrix = Matrix4x4.identity;

            rotationMatrix.CreateRotationZ(value);

            for (var i = 0; i < ControlPoints.Count; i++)
                ControlPoints[i].Transform(rotationMatrix);
        }

        public override string ToString()
        {
            var builder = new StringBuilder();
            for (var i = 0; i < ControlPoints.Count; i++)
            {
                builder.Append(ControlPoints[i]);
                if (i < ControlPoints.Count - 1) builder.Append(" ");
            }

            return builder.ToString();
        }

        /// <summary>
        /// Returns a set of points defining the
        /// curve with the specifed number of divisions
        /// between each control point.
        /// </summary>
        /// <param name="divisions">Number of divisions between each control point.</param>
        /// <returns></returns>
        public Vertices GetVertices(int divisions)
        {
            var verts = new Vertices();

            var timeStep = 1f / divisions;

            for (float i = 0; i < 1f; i += timeStep) verts.Add(GetPosition(i));

            return verts;
        }

        public Vector2 GetPosition(float time)
        {
            Vector2 temp;

            if (ControlPoints.Count < 2)
                throw new Exception("You need at least 2 control points to calculate a position.");

            if (Closed)
            {
                Add(ControlPoints[0]);

                _deltaT = 1f / (ControlPoints.Count - 1);

                var p = (int) (time / _deltaT);

                // use a circular indexing system
                var p0 = p - 1;
                if (p0 < 0) p0 = p0 + (ControlPoints.Count - 1);
                else if (p0 >= ControlPoints.Count - 1) p0 = p0 - (ControlPoints.Count - 1);
                var p1 = p;
                if (p1 < 0) p1 = p1 + (ControlPoints.Count - 1);
                else if (p1 >= ControlPoints.Count - 1) p1 = p1 - (ControlPoints.Count - 1);
                var p2 = p + 1;
                if (p2 < 0) p2 = p2 + (ControlPoints.Count - 1);
                else if (p2 >= ControlPoints.Count - 1) p2 = p2 - (ControlPoints.Count - 1);
                var p3 = p + 2;
                if (p3 < 0) p3 = p3 + (ControlPoints.Count - 1);
                else if (p3 >= ControlPoints.Count - 1) p3 = p3 - (ControlPoints.Count - 1);

                // relative time
                var lt = (time - _deltaT * p) / _deltaT;

                temp = VectorUtils.CatmullRom(ControlPoints[p0], ControlPoints[p1], ControlPoints[p2],
                    ControlPoints[p3], lt);

                RemoveAt(ControlPoints.Count - 1);
            }
            else
            {
                var p = (int) (time / _deltaT);

                // 
                var p0 = p - 1;
                if (p0 < 0) p0 = 0;
                else if (p0 >= ControlPoints.Count - 1) p0 = ControlPoints.Count - 1;
                var p1 = p;
                if (p1 < 0) p1 = 0;
                else if (p1 >= ControlPoints.Count - 1) p1 = ControlPoints.Count - 1;
                var p2 = p + 1;
                if (p2 < 0) p2 = 0;
                else if (p2 >= ControlPoints.Count - 1) p2 = ControlPoints.Count - 1;
                var p3 = p + 2;
                if (p3 < 0) p3 = 0;
                else if (p3 >= ControlPoints.Count - 1) p3 = ControlPoints.Count - 1;

                // relative time
                var lt = (time - _deltaT * p) / _deltaT;

                temp = VectorUtils.CatmullRom(ControlPoints[p0], ControlPoints[p1], ControlPoints[p2],
                    ControlPoints[p3], lt);
            }

            return temp;
        }

        /// <summary>
        /// Gets the normal for the given time.
        /// </summary>
        /// <param name="time">The time</param>
        /// <returns>The normal.</returns>
        public Vector2 GetPositionNormal(float time)
        {
            var offsetTime = time + 0.0001f;

            var a = GetPosition(time);
            var b = GetPosition(offsetTime);

            Vector2 output, temp;

            temp = a - b;

#if (XBOX360 || WINDOWS_PHONE)
output = new Vector2();
#endif
            output.x = -temp.y;
            output.y = temp.x;

            output = output.normalized;

            return output;
        }

        public void Add(Vector2 point)
        {
            ControlPoints.Add(point);
            _deltaT = 1f / (ControlPoints.Count - 1);
        }

        public void Remove(Vector2 point)
        {
            ControlPoints.Remove(point);
            _deltaT = 1f / (ControlPoints.Count - 1);
        }

        public void RemoveAt(int index)
        {
            ControlPoints.RemoveAt(index);
            _deltaT = 1f / (ControlPoints.Count - 1);
        }

        public float GetLength()
        {
            List<Vector2> verts = GetVertices(ControlPoints.Count * 25);
            float length = 0;

            for (var i = 1; i < verts.Count; i++) length += Vector2.Distance(verts[i - 1], verts[i]);

            if (Closed)
                length += Vector2.Distance(verts[ControlPoints.Count - 1], verts[0]);

            return length;
        }

        public List<Vector3> SubdivideEvenly(int divisions)
        {
            var verts = new List<Vector3>();

            var length = GetLength();

            var deltaLength = length / divisions + 0.001f;
            var t = 0.000f;

            // we always start at the first control point
            var start = ControlPoints[0];
            var end = GetPosition(t);

            // increment t until we are at half the distance
            while (deltaLength * 0.5f >= Vector2.Distance(start, end))
            {
                end = GetPosition(t);
                t += 0.0001f;

                if (t >= 1f)
                    break;
            }

            start = end;

            // for each box
            for (var i = 1; i < divisions; i++)
            {
                var normal = GetPositionNormal(t);
                var angle = Mathf.Atan2(normal.y, normal.x);

                verts.Add(new Vector3(end.x, end.y, angle));

                // until we reach the correct distance down the curve
                while (deltaLength >= Vector2.Distance(start, end))
                {
                    end = GetPosition(t);
                    t += 0.00001f;

                    if (t >= 1f)
                        break;
                }

                if (t >= 1f)
                    break;

                start = end;
            }

            return verts;
        }
    }
}