using System;
using System.Collections.Generic;
using UnityEngine;
using VelcroPhysics.Collision.Shapes;
using VelcroPhysics.Dynamics;
using VelcroPhysics.Extensions.Controllers.ControllerBase;
using VelcroPhysics.Shared;
using VelcroPhysics.Utilities;
using Transform = VelcroPhysics.Shared.Transform;

namespace VelcroPhysics.Extensions.Controllers.Buoyancy
{
    public sealed class BuoyancyController : Controller
    {
        private AABB _container;

        private Vector2 _gravity;
        private Vector2 _normal;
        private float _offset;
        private Dictionary<int, Body> _uniqueBodies = new Dictionary<int, Body>();

        /// <summary>
        /// Controls the rotational drag that the fluid exerts on the bodies within it. Use higher values will simulate thick
        /// fluid, like honey, lower values to
        /// simulate water-like fluids.
        /// </summary>
        public float AngularDragCoefficient;

        /// <summary>
        /// Density of the fluid. Higher values will make things more buoyant, lower values will cause things to sink.
        /// </summary>
        public float Density;

        /// <summary>
        /// Controls the linear drag that the fluid exerts on the bodies within it.  Use higher values will simulate thick fluid,
        /// like honey, lower values to
        /// simulate water-like fluids.
        /// </summary>
        public float LinearDragCoefficient;

        /// <summary>
        /// Acts like waterflow. Defaults to 0,0.
        /// </summary>
        public Vector2 Velocity;

        /// <summary>
        /// Initializes a new instance of the <see cref="BuoyancyController" /> class.
        /// </summary>
        /// <param name="container">Only bodies inside this AABB will be influenced by the controller</param>
        /// <param name="density">Density of the fluid</param>
        /// <param name="linearDragCoefficient">Linear drag coefficient of the fluid</param>
        /// <param name="rotationalDragCoefficient">Rotational drag coefficient of the fluid</param>
        /// <param name="gravity">The direction gravity acts. Buoyancy force will act in opposite direction of gravity.</param>
        public BuoyancyController(AABB container, float density, float linearDragCoefficient,
            float rotationalDragCoefficient, Vector2 gravity)
            : base(ControllerType.BuoyancyController)
        {
            Container = container;
            _normal = new Vector2(0, 1);
            Density = density;
            LinearDragCoefficient = linearDragCoefficient;
            AngularDragCoefficient = rotationalDragCoefficient;
            _gravity = gravity;
        }

        public AABB Container
        {
            get => _container;
            set
            {
                _container = value;
                _offset = _container.UpperBound.y;
            }
        }

        public override void Update(float dt)
        {
            _uniqueBodies.Clear();
            World.QueryAABB(fixture =>
            {
                if (fixture.Body.IsStatic || !fixture.Body.Awake)
                    return true;

                if (!_uniqueBodies.ContainsKey(fixture.Body.BodyId))
                    _uniqueBodies.Add(fixture.Body.BodyId, fixture.Body);

                return true;
            }, ref _container);

            foreach (var kv in _uniqueBodies)
            {
                var body = kv.Value;

                var areac = Vector2.zero;
                var massc = Vector2.zero;
                float area = 0;
                float mass = 0;

                for (var j = 0; j < body.FixtureList.Count; j++)
                {
                    var fixture = body.FixtureList[j];

                    if (fixture.Shape.ShapeType != ShapeType.Polygon && fixture.Shape.ShapeType != ShapeType.Circle)
                        continue;

                    var shape = fixture.Shape;

                    Vector2 sc;
                    var sarea = ComputeSubmergedArea(shape, ref _normal, _offset, ref body._xf, out sc);
                    area += sarea;
                    areac.x += sarea * sc.x;
                    areac.y += sarea * sc.y;

                    mass += sarea * shape.Density;
                    massc.x += sarea * sc.x * shape.Density;
                    massc.y += sarea * sc.y * shape.Density;
                }

                areac.x /= area;
                areac.y /= area;
                massc.x /= mass;
                massc.y /= mass;

                if (area < Settings.Epsilon)
                    continue;

                //Buoyancy
                var buoyancyForce = -Density * area * _gravity;
                body.ApplyForce(buoyancyForce, massc);

                //Linear drag
                var dragForce = body.GetLinearVelocityFromWorldPoint(areac) - Velocity;
                dragForce *= -LinearDragCoefficient * area;
                body.ApplyForce(dragForce, areac);

                //Angular drag
                body.ApplyTorque(-body.Inertia / body.Mass * area * body.AngularVelocity * AngularDragCoefficient);
            }
        }

        private float ComputeSubmergedArea(Shape shape, ref Vector2 normal, float offset, ref Transform xf,
            out Vector2 sc)
        {
            switch (shape.ShapeType)
            {
                case ShapeType.Circle:
                {
                    var circleShape = (CircleShape) shape;

                    sc = Vector2.zero;

                    var p = MathUtils.Mul(ref xf, circleShape.Position);
                    var l = -(Vector2.Dot(normal, p) - offset);
                    if (l < -circleShape.Radius + Settings.Epsilon)
                        //Completely dry
                        return 0;
                    if (l > circleShape.Radius)
                    {
                        //Completely wet
                        sc = p;
                        return Settings.Pi * circleShape._2radius;
                    }

                    //Magic
                    var l2 = l * l;
                    var area = circleShape._2radius *
                               (Mathf.Asin(l / circleShape.Radius) + Settings.Pi / 2 +
                                        l * Mathf.Sqrt(circleShape._2radius - l2));
                    var com = -2.0f / 3.0f * Mathf.Pow(circleShape._2radius - l2, 1.5f) / area;

                    sc.x = p.x + normal.x * com;
                    sc.y = p.y + normal.y * com;

                    return area;
                }
                case ShapeType.Edge:
                    sc = Vector2.zero;
                    return 0;
                case ShapeType.Polygon:
                {
                    sc = Vector2.zero;

                    var polygonShape = (PolygonShape) shape;

                    //Transform plane into shape co-ordinates
                    var normalL = MathUtils.MulT(xf.q, normal);
                    var offsetL = offset - Vector2.Dot(normal, xf.p);

                    var depths = new float[Settings.MaxPolygonVertices];
                    var diveCount = 0;
                    var intoIndex = -1;
                    var outoIndex = -1;

                    var lastSubmerged = false;
                    int i;
                    for (i = 0; i < polygonShape.Vertices.Count; i++)
                    {
                        depths[i] = Vector2.Dot(normalL, polygonShape.Vertices[i]) - offsetL;
                        var isSubmerged = depths[i] < -Settings.Epsilon;
                        if (i > 0)
                        {
                            if (isSubmerged)
                            {
                                if (!lastSubmerged)
                                {
                                    intoIndex = i - 1;
                                    diveCount++;
                                }
                            }
                            else
                            {
                                if (lastSubmerged)
                                {
                                    outoIndex = i - 1;
                                    diveCount++;
                                }
                            }
                        }

                        lastSubmerged = isSubmerged;
                    }

                    switch (diveCount)
                    {
                        case 0:
                            if (lastSubmerged)
                            {
                                //Completely submerged
                                sc = MathUtils.Mul(ref xf, polygonShape.MassData.Centroid);
                                return polygonShape.MassData.Mass / Density;
                            }

                            //Completely dry
                            return 0;
                        case 1:
                            if (intoIndex == -1)
                                intoIndex = polygonShape.Vertices.Count - 1;
                            else
                                outoIndex = polygonShape.Vertices.Count - 1;
                            break;
                    }

                    var intoIndex2 = (intoIndex + 1) % polygonShape.Vertices.Count;
                    var outoIndex2 = (outoIndex + 1) % polygonShape.Vertices.Count;

                    var intoLambda = (0 - depths[intoIndex]) / (depths[intoIndex2] - depths[intoIndex]);
                    var outoLambda = (0 - depths[outoIndex]) / (depths[outoIndex2] - depths[outoIndex]);

                    var intoVec = new Vector2(
                        polygonShape.Vertices[intoIndex].x * (1 - intoLambda) +
                        polygonShape.Vertices[intoIndex2].x * intoLambda,
                        polygonShape.Vertices[intoIndex].y * (1 - intoLambda) +
                        polygonShape.Vertices[intoIndex2].y * intoLambda);
                    var outoVec = new Vector2(
                        polygonShape.Vertices[outoIndex].x * (1 - outoLambda) +
                        polygonShape.Vertices[outoIndex2].x * outoLambda,
                        polygonShape.Vertices[outoIndex].y * (1 - outoLambda) +
                        polygonShape.Vertices[outoIndex2].y * outoLambda);

                    //Initialize accumulator
                    float area = 0;
                    var center = new Vector2(0, 0);
                    var p2 = polygonShape.Vertices[intoIndex2];

                    const float k_inv3 = 1.0f / 3.0f;

                    //An awkward loop from intoIndex2+1 to outIndex2
                    i = intoIndex2;
                    while (i != outoIndex2)
                    {
                        i = (i + 1) % polygonShape.Vertices.Count;
                        Vector2 p3;
                        if (i == outoIndex2)
                            p3 = outoVec;
                        else
                            p3 = polygonShape.Vertices[i];

                        //Add the triangle formed by intoVec,p2,p3
                        {
                            var e1 = p2 - intoVec;
                            var e2 = p3 - intoVec;

                            var D = MathUtils.Cross(e1, e2);

                            var triangleArea = 0.5f * D;

                            area += triangleArea;

                            // Area weighted centroid
                            center += triangleArea * k_inv3 * (intoVec + p2 + p3);
                        }

                        p2 = p3;
                    }

                    //Normalize and transform centroid
                    center *= 1.0f / area;

                    sc = MathUtils.Mul(ref xf, center);

                    return area;
                }
                case ShapeType.Chain:
                    sc = Vector2.zero;
                    return 0;
                case ShapeType.Unknown:
                case ShapeType.TypeCount:
                    throw new NotSupportedException();
                default:
                    throw new ArgumentOutOfRangeException();
            }
        }
    }
}