using UnityEngine;
using VelcroPhysics.Shared;

namespace VelcroPhysics.Tools.Triangulation.Earclip
{
    public class Triangle : Vertices
    {
        //Constructor automatically fixes orientation to ccw
        public Triangle(float x1, float y1, float x2, float y2, float x3, float y3)
        {
            var cross = (x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1);
            if (cross > 0)
            {
                Add(new Vector2(x1, y1));
                Add(new Vector2(x2, y2));
                Add(new Vector2(x3, y3));
            }
            else
            {
                Add(new Vector2(x1, y1));
                Add(new Vector2(x3, y3));
                Add(new Vector2(x2, y2));
            }
        }

        public bool IsInside(float x, float y)
        {
            var a = this[0];
            var b = this[1];
            var c = this[2];

            if (x < a.x && x < b.x && x < c.x) return false;
            if (x > a.x && x > b.x && x > c.x) return false;
            if (y < a.y && y < b.y && y < c.y) return false;
            if (y > a.y && y > b.y && y > c.y) return false;

            var vx2 = x - a.x;
            var vy2 = y - a.y;
            var vx1 = b.x - a.x;
            var vy1 = b.y - a.y;
            var vx0 = c.x - a.x;
            var vy0 = c.y - a.y;

            var dot00 = vx0 * vx0 + vy0 * vy0;
            var dot01 = vx0 * vx1 + vy0 * vy1;
            var dot02 = vx0 * vx2 + vy0 * vy2;
            var dot11 = vx1 * vx1 + vy1 * vy1;
            var dot12 = vx1 * vx2 + vy1 * vy2;
            var invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01);
            var u = (dot11 * dot02 - dot01 * dot12) * invDenom;
            var v = (dot00 * dot12 - dot01 * dot02) * invDenom;

            return u > 0 && v > 0 && u + v < 1;
        }
    }
}