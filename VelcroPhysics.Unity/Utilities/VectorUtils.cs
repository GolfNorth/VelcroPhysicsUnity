using System;
using UnityEngine;

namespace VelcroPhysics.Utilities
{
    public static class VectorUtils
    {
        public static Vector2 Transform(this Vector2 position, Matrix4x4 matrix)
        {
            Transform(ref position, ref matrix, out position);
            return position;
        }

        public static void Transform(this ref Vector2 position, ref Matrix4x4 matrix, out Vector2 result)
        {
            result = new Vector2(position.x * matrix.m00 + position.y * matrix.m10 + matrix.m30,
                position.x * matrix.m01 + position.y * matrix.m11 + matrix.m31);
        }

        public static void Transform(this Vector2[] sourceArray, ref Matrix4x4 matrix, Vector2[] destinationArray)
        {
            throw new NotImplementedException();
        }

        public static void Transform(this Vector2[] sourceArray, int sourceIndex, ref Matrix4x4 matrix,
            Vector2[] destinationArray, int destinationIndex, int length)
        {
            throw new NotImplementedException();
        }

        public static Vector2 CatmullRom(Vector2 value1, Vector2 value2, Vector2 value3, Vector2 value4, float amount)
        {
            return new Vector2(
                CatmullRom(value1.x, value2.x, value3.x, value4.x, amount),
                CatmullRom(value1.y, value2.y, value3.y, value4.y, amount));
        }

        public static void CatmullRom(ref Vector2 value1, ref Vector2 value2, ref Vector2 value3, ref Vector2 value4,
            float amount, out Vector2 result)
        {
            result = new Vector2(
                CatmullRom(value1.x, value2.x, value3.x, value4.x, amount),
                CatmullRom(value1.y, value2.y, value3.y, value4.y, amount));
        }

        public static float CatmullRom(float value1, float value2, float value3, float value4, float amount)
        {
            // Using formula from http://www.mvps.org/directx/articles/catmull/
            // Internally using floats not to lose precission
            var amountSquared = amount * amount;
            var amountCubed = amountSquared * amount;

            return (float) (0.5 * (2.0 * value2 + (value3 - value1) * amount
                                                + (2.0 * value1 - 5.0 * value2 + 4.0 * value3 - value4) * amountSquared
                                                + (3.0 * value2 - value1 - 3.0 * value3 + value4) * amountCubed));
        }
    }
}