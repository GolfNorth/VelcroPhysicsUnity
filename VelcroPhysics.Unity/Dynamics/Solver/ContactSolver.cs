//#define B2_DEBUG_SOLVER
/*
* Velcro Physics:
* Copyright (c) 2017 Ian Qvist
* 
* Original source Box2D:
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org 
* 
* This software is provided 'as-is', without any express or implied 
* warranty.  In no event will the authors be held liable for any damages 
* arising from the use of this software. 
* Permission is granted to anyone to use this software for any purpose, 
* including commercial applications, and to alter it and redistribute it 
* freely, subject to the following restrictions: 
* 1. The origin of this software must not be misrepresented; you must not 
* claim that you wrote the original software. If you use this software 
* in a product, an acknowledgment in the product documentation would be 
* appreciated but is not required. 
* 2. Altered source versions must be plainly marked as such, and must not be 
* misrepresented as being the original software. 
* 3. This notice may not be removed or altered from any source distribution. 
*/

using UnityEngine;
using VelcroPhysics.Collision.ContactSystem;
using VelcroPhysics.Collision.Narrowphase;
using VelcroPhysics.Collision.Shapes;
using VelcroPhysics.Shared;
using VelcroPhysics.Shared.Optimization;
using VelcroPhysics.Utilities;
using Transform = VelcroPhysics.Shared.Transform;

namespace VelcroPhysics.Dynamics.Solver
{
    public class ContactSolver
    {
        private Contact[] _contacts;
        private int _count;
        private ContactPositionConstraint[] _positionConstraints;
        private Position[] _positions;
        private TimeStep _step;
        private Velocity[] _velocities;
        public ContactVelocityConstraint[] VelocityConstraints;

        public void Reset(TimeStep step, int count, Contact[] contacts, Position[] positions, Velocity[] velocities)
        {
            _step = step;
            _count = count;
            _positions = positions;
            _velocities = velocities;
            _contacts = contacts;

            // grow the array
            if (VelocityConstraints == null || VelocityConstraints.Length < count)
            {
                VelocityConstraints = new ContactVelocityConstraint[count * 2];
                _positionConstraints = new ContactPositionConstraint[count * 2];

                for (var i = 0; i < VelocityConstraints.Length; i++)
                    VelocityConstraints[i] = new ContactVelocityConstraint();

                for (var i = 0; i < _positionConstraints.Length; i++)
                    _positionConstraints[i] = new ContactPositionConstraint();
            }

            // Initialize position independent portions of the constraints.
            for (var i = 0; i < _count; ++i)
            {
                var contact = contacts[i];

                var fixtureA = contact.FixtureA;
                var fixtureB = contact.FixtureB;
                var shapeA = fixtureA.Shape;
                var shapeB = fixtureB.Shape;
                var radiusA = shapeA.Radius;
                var radiusB = shapeB.Radius;
                var bodyA = fixtureA.Body;
                var bodyB = fixtureB.Body;
                var manifold = contact.Manifold;

                var pointCount = manifold.PointCount;
                Debug.Assert(pointCount > 0);

                var vc = VelocityConstraints[i];
                vc.Friction = contact.Friction;
                vc.Restitution = contact.Restitution;
                vc.TangentSpeed = contact.TangentSpeed;
                vc.IndexA = bodyA.IslandIndex;
                vc.IndexB = bodyB.IslandIndex;
                vc.InvMassA = bodyA._invMass;
                vc.InvMassB = bodyB._invMass;
                vc.InvIA = bodyA._invI;
                vc.InvIB = bodyB._invI;
                vc.ContactIndex = i;
                vc.PointCount = pointCount;
                vc.K.SetZero();
                vc.NormalMass.SetZero();

                var pc = _positionConstraints[i];
                pc.IndexA = bodyA.IslandIndex;
                pc.IndexB = bodyB.IslandIndex;
                pc.InvMassA = bodyA._invMass;
                pc.InvMassB = bodyB._invMass;
                pc.LocalCenterA = bodyA._sweep.LocalCenter;
                pc.LocalCenterB = bodyB._sweep.LocalCenter;
                pc.InvIA = bodyA._invI;
                pc.InvIB = bodyB._invI;
                pc.LocalNormal = manifold.LocalNormal;
                pc.LocalPoint = manifold.LocalPoint;
                pc.PointCount = pointCount;
                pc.RadiusA = radiusA;
                pc.RadiusB = radiusB;
                pc.Type = manifold.Type;

                for (var j = 0; j < pointCount; ++j)
                {
                    var cp = manifold.Points[j];
                    var vcp = vc.Points[j];

                    if (Settings.EnableWarmstarting)
                    {
                        vcp.NormalImpulse = _step.dtRatio * cp.NormalImpulse;
                        vcp.TangentImpulse = _step.dtRatio * cp.TangentImpulse;
                    }
                    else
                    {
                        vcp.NormalImpulse = 0.0f;
                        vcp.TangentImpulse = 0.0f;
                    }

                    vcp.rA = Vector2.zero;
                    vcp.rB = Vector2.zero;
                    vcp.NormalMass = 0.0f;
                    vcp.TangentMass = 0.0f;
                    vcp.VelocityBias = 0.0f;

                    pc.LocalPoints[j] = cp.LocalPoint;
                }
            }
        }

        /// <summary>
        /// Initialize position dependent portions of the velocity constraints.
        /// </summary>
        public void InitializeVelocityConstraints()
        {
            for (var i = 0; i < _count; ++i)
            {
                var vc = VelocityConstraints[i];
                var pc = _positionConstraints[i];

                var radiusA = pc.RadiusA;
                var radiusB = pc.RadiusB;
                var manifold = _contacts[vc.ContactIndex].Manifold;

                var indexA = vc.IndexA;
                var indexB = vc.IndexB;

                var mA = vc.InvMassA;
                var mB = vc.InvMassB;
                var iA = vc.InvIA;
                var iB = vc.InvIB;
                var localCenterA = pc.LocalCenterA;
                var localCenterB = pc.LocalCenterB;

                var cA = _positions[indexA].C;
                var aA = _positions[indexA].A;
                var vA = _velocities[indexA].V;
                var wA = _velocities[indexA].W;

                var cB = _positions[indexB].C;
                var aB = _positions[indexB].A;
                var vB = _velocities[indexB].V;
                var wB = _velocities[indexB].W;

                Debug.Assert(manifold.PointCount > 0);

                var xfA = new Transform();
                var xfB = new Transform();
                xfA.q.Set(aA);
                xfB.q.Set(aB);
                xfA.p = cA - MathUtils.Mul(xfA.q, localCenterA);
                xfB.p = cB - MathUtils.Mul(xfB.q, localCenterB);

                WorldManifold.Initialize(ref manifold, ref xfA, radiusA, ref xfB, radiusB, out var normal,
                    out var points, out _);

                vc.Normal = normal;

                var pointCount = vc.PointCount;
                for (var j = 0; j < pointCount; ++j)
                {
                    var vcp = vc.Points[j];

                    vcp.rA = points[j] - cA;
                    vcp.rB = points[j] - cB;

                    var rnA = MathUtils.Cross(vcp.rA, vc.Normal);
                    var rnB = MathUtils.Cross(vcp.rB, vc.Normal);

                    var kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

                    vcp.NormalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

                    var tangent = MathUtils.Cross(vc.Normal, 1.0f);

                    var rtA = MathUtils.Cross(vcp.rA, tangent);
                    var rtB = MathUtils.Cross(vcp.rB, tangent);

                    var kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

                    vcp.TangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

                    // Setup a velocity bias for restitution.
                    vcp.VelocityBias = 0.0f;
                    var vRel = Vector2.Dot(vc.Normal,
                        vB + MathUtils.Cross(wB, vcp.rB) - vA - MathUtils.Cross(wA, vcp.rA));
                    if (vRel < -Settings.VelocityThreshold) vcp.VelocityBias = -vc.Restitution * vRel;
                }

                // If we have two points, then prepare the block solver.
                if (vc.PointCount == 2 && Settings.BlockSolve)
                {
                    var vcp1 = vc.Points[0];
                    var vcp2 = vc.Points[1];

                    var rn1A = MathUtils.Cross(vcp1.rA, vc.Normal);
                    var rn1B = MathUtils.Cross(vcp1.rB, vc.Normal);
                    var rn2A = MathUtils.Cross(vcp2.rA, vc.Normal);
                    var rn2B = MathUtils.Cross(vcp2.rB, vc.Normal);

                    var k11 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
                    var k22 = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
                    var k12 = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;

                    // Ensure a reasonable condition number.
                    const float k_maxConditionNumber = 1000.0f;
                    if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12))
                    {
                        // K is safe to invert.
                        vc.K.ex = new Vector2(k11, k12);
                        vc.K.ey = new Vector2(k12, k22);
                        vc.NormalMass = vc.K.Inverse;
                    }
                    else
                    {
                        // The constraints are redundant, just use one.
                        // TODO_ERIN use deepest?
                        vc.PointCount = 1;
                    }
                }
            }
        }

        public void WarmStart()
        {
            // Warm start.
            for (var i = 0; i < _count; ++i)
            {
                var vc = VelocityConstraints[i];

                var indexA = vc.IndexA;
                var indexB = vc.IndexB;
                var mA = vc.InvMassA;
                var iA = vc.InvIA;
                var mB = vc.InvMassB;
                var iB = vc.InvIB;
                var pointCount = vc.PointCount;

                var vA = _velocities[indexA].V;
                var wA = _velocities[indexA].W;
                var vB = _velocities[indexB].V;
                var wB = _velocities[indexB].W;

                var normal = vc.Normal;
                var tangent = MathUtils.Cross(normal, 1.0f);

                for (var j = 0; j < pointCount; ++j)
                {
                    var vcp = vc.Points[j];
                    var P = vcp.NormalImpulse * normal + vcp.TangentImpulse * tangent;
                    wA -= iA * MathUtils.Cross(vcp.rA, P);
                    vA -= mA * P;
                    wB += iB * MathUtils.Cross(vcp.rB, P);
                    vB += mB * P;
                }

                _velocities[indexA].V = vA;
                _velocities[indexA].W = wA;
                _velocities[indexB].V = vB;
                _velocities[indexB].W = wB;
            }
        }

        public void SolveVelocityConstraints()
        {
            for (var i = 0; i < _count; ++i)
            {
                var vc = VelocityConstraints[i];

                var indexA = vc.IndexA;
                var indexB = vc.IndexB;
                var mA = vc.InvMassA;
                var iA = vc.InvIA;
                var mB = vc.InvMassB;
                var iB = vc.InvIB;
                var pointCount = vc.PointCount;

                var vA = _velocities[indexA].V;
                var wA = _velocities[indexA].W;
                var vB = _velocities[indexB].V;
                var wB = _velocities[indexB].W;

                var normal = vc.Normal;
                var tangent = MathUtils.Cross(normal, 1.0f);
                var friction = vc.Friction;

                Debug.Assert(pointCount == 1 || pointCount == 2);

                // Solve tangent constraints first because non-penetration is more important
                // than friction.
                for (var j = 0; j < pointCount; ++j)
                {
                    var vcp = vc.Points[j];

                    // Relative velocity at contact
                    var dv = vB + MathUtils.Cross(wB, vcp.rB) - vA - MathUtils.Cross(wA, vcp.rA);

                    // Compute tangent force
                    var vt = Vector2.Dot(dv, tangent) - vc.TangentSpeed;
                    var lambda = vcp.TangentMass * -vt;

                    // b2Clamp the accumulated force
                    var maxFriction = friction * vcp.NormalImpulse;
                    var newImpulse = MathUtils.Clamp(vcp.TangentImpulse + lambda, -maxFriction, maxFriction);
                    lambda = newImpulse - vcp.TangentImpulse;
                    vcp.TangentImpulse = newImpulse;

                    // Apply contact impulse
                    var P = lambda * tangent;

                    vA -= mA * P;
                    wA -= iA * MathUtils.Cross(vcp.rA, P);

                    vB += mB * P;
                    wB += iB * MathUtils.Cross(vcp.rB, P);
                }

                // Solve normal constraints
                if (pointCount == 1 || Settings.BlockSolve == false)
                {
                    for (var j = 0; j < pointCount; ++j)
                    {
                        var vcp = vc.Points[j];

                        // Relative velocity at contact
                        var dv = vB + MathUtils.Cross(wB, vcp.rB) - vA - MathUtils.Cross(wA, vcp.rA);

                        // Compute normal impulse
                        var vn = Vector2.Dot(dv, normal);
                        var lambda = -vcp.NormalMass * (vn - vcp.VelocityBias);

                        // b2Clamp the accumulated impulse
                        var newImpulse = Mathf.Max(vcp.NormalImpulse + lambda, 0.0f);
                        lambda = newImpulse - vcp.NormalImpulse;
                        vcp.NormalImpulse = newImpulse;

                        // Apply contact impulse
                        var P = lambda * normal;
                        vA -= mA * P;
                        wA -= iA * MathUtils.Cross(vcp.rA, P);

                        vB += mB * P;
                        wB += iB * MathUtils.Cross(vcp.rB, P);
                    }
                }
                else
                {
                    // Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
                    // Build the mini LCP for this contact patch
                    //
                    // vn = A * x + b, vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
                    //
                    // A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
                    // b = vn0 - velocityBias
                    //
                    // The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
                    // implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
                    // vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
                    // solution that satisfies the problem is chosen.
                    // 
                    // In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
                    // that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
                    //
                    // Substitute:
                    // 
                    // x = a + d
                    // 
                    // a := old total impulse
                    // x := new total impulse
                    // d := incremental impulse 
                    //
                    // For the current iteration we extend the formula for the incremental impulse
                    // to compute the new total impulse:
                    //
                    // vn = A * d + b
                    //    = A * (x - a) + b
                    //    = A * x + b - A * a
                    //    = A * x + b'
                    // b' = b - A * a;

                    var cp1 = vc.Points[0];
                    var cp2 = vc.Points[1];

                    var a = new Vector2(cp1.NormalImpulse, cp2.NormalImpulse);
                    Debug.Assert(a.x >= 0.0f && a.y >= 0.0f);

                    // Relative velocity at contact
                    var dv1 = vB + MathUtils.Cross(wB, cp1.rB) - vA - MathUtils.Cross(wA, cp1.rA);
                    var dv2 = vB + MathUtils.Cross(wB, cp2.rB) - vA - MathUtils.Cross(wA, cp2.rA);

                    // Compute normal velocity
                    var vn1 = Vector2.Dot(dv1, normal);
                    var vn2 = Vector2.Dot(dv2, normal);

                    var b = Vector2.zero;
                    b.x = vn1 - cp1.VelocityBias;
                    b.y = vn2 - cp2.VelocityBias;

                    //const float k_errorTol = 1e-3f;

                    // Compute b'
                    b -= MathUtils.Mul(ref vc.K, a);

                    for (;;)
                    {
                        //
                        // Case 1: vn = 0
                        //
                        // 0 = A * x + b'
                        //
                        // Solve for x:
                        //
                        // x = - inv(A) * b'
                        //
                        var x = -MathUtils.Mul(ref vc.NormalMass, b);

                        if (x.x >= 0.0f && x.y >= 0.0f)
                        {
                            // Get the incremental impulse
                            var d = x - a;

                            // Apply incremental impulse
                            var P1 = d.x * normal;
                            var P2 = d.y * normal;
                            vA -= mA * (P1 + P2);
                            wA -= iA * (MathUtils.Cross(cp1.rA, P1) + MathUtils.Cross(cp2.rA, P2));

                            vB += mB * (P1 + P2);
                            wB += iB * (MathUtils.Cross(cp1.rB, P1) + MathUtils.Cross(cp2.rB, P2));

                            // Accumulate
                            cp1.NormalImpulse = x.x;
                            cp2.NormalImpulse = x.y;

#if B2_DEBUG_SOLVER
                            // Postconditions
                            dv1 = vB + MathUtils.Cross(wB, cp1.rB) - vA - MathUtils.Cross(wA, cp1.rA);
                            dv2 = vB + MathUtils.Cross(wB, cp2.rB) - vA - MathUtils.Cross(wA, cp2.rA);

                            // Compute normal velocity
                            vn1 = Vector2.Dot(dv1, normal);
                            vn2 = Vector2.Dot(dv2, normal);

                            Debug.Assert(Mathf.Abs(vn1 - cp1.VelocityBias) < k_errorTol);
                            Debug.Assert(Mathf.Abs(vn2 - cp2.VelocityBias) < k_errorTol);
#endif
                            break;
                        }

                        //
                        // Case 2: vn1 = 0 and x2 = 0
                        //
                        //   0 = a11 * x1 + a12 * 0 + b1' 
                        // vn2 = a21 * x1 + a22 * 0 + b2'
                        //
                        x.x = -cp1.NormalMass * b.x;
                        x.y = 0.0f;
                        vn1 = 0.0f;
                        vn2 = vc.K.ex.y * x.x + b.y;

                        if (x.x >= 0.0f && vn2 >= 0.0f)
                        {
                            // Get the incremental impulse
                            var d = x - a;

                            // Apply incremental impulse
                            var P1 = d.x * normal;
                            var P2 = d.y * normal;
                            vA -= mA * (P1 + P2);
                            wA -= iA * (MathUtils.Cross(cp1.rA, P1) + MathUtils.Cross(cp2.rA, P2));

                            vB += mB * (P1 + P2);
                            wB += iB * (MathUtils.Cross(cp1.rB, P1) + MathUtils.Cross(cp2.rB, P2));

                            // Accumulate
                            cp1.NormalImpulse = x.x;
                            cp2.NormalImpulse = x.y;

#if B2_DEBUG_SOLVER
                            // Postconditions
                            dv1 = vB + MathUtils.Cross(wB, cp1.rB) - vA - MathUtils.Cross(wA, cp1.rA);

                            // Compute normal velocity
                            vn1 = Vector2.Dot(dv1, normal);

                            Debug.Assert(Mathf.Abs(vn1 - cp1.VelocityBias) < k_errorTol);
#endif
                            break;
                        }

                        //
                        // Case 3: vn2 = 0 and x1 = 0
                        //
                        // vn1 = a11 * 0 + a12 * x2 + b1' 
                        //   0 = a21 * 0 + a22 * x2 + b2'
                        //
                        x.x = 0.0f;
                        x.y = -cp2.NormalMass * b.y;
                        vn1 = vc.K.ey.x * x.y + b.x;
                        vn2 = 0.0f;

                        if (x.y >= 0.0f && vn1 >= 0.0f)
                        {
                            // Resubstitute for the incremental impulse
                            var d = x - a;

                            // Apply incremental impulse
                            var P1 = d.x * normal;
                            var P2 = d.y * normal;
                            vA -= mA * (P1 + P2);
                            wA -= iA * (MathUtils.Cross(cp1.rA, P1) + MathUtils.Cross(cp2.rA, P2));

                            vB += mB * (P1 + P2);
                            wB += iB * (MathUtils.Cross(cp1.rB, P1) + MathUtils.Cross(cp2.rB, P2));

                            // Accumulate
                            cp1.NormalImpulse = x.x;
                            cp2.NormalImpulse = x.y;

#if B2_DEBUG_SOLVER
                            // Postconditions
                            dv2 = vB + MathUtils.Cross(wB, cp2.rB) - vA - MathUtils.Cross(wA, cp2.rA);

                            // Compute normal velocity
                            vn2 = Vector2.Dot(dv2, normal);

                            Debug.Assert(Mathf.Abs(vn2 - cp2.VelocityBias) < k_errorTol);
#endif
                            break;
                        }

                        //
                        // Case 4: x1 = 0 and x2 = 0
                        // 
                        // vn1 = b1
                        // vn2 = b2;
                        x.x = 0.0f;
                        x.y = 0.0f;
                        vn1 = b.x;
                        vn2 = b.y;

                        if (vn1 >= 0.0f && vn2 >= 0.0f)
                        {
                            // Resubstitute for the incremental impulse
                            var d = x - a;

                            // Apply incremental impulse
                            var P1 = d.x * normal;
                            var P2 = d.y * normal;
                            vA -= mA * (P1 + P2);
                            wA -= iA * (MathUtils.Cross(cp1.rA, P1) + MathUtils.Cross(cp2.rA, P2));

                            vB += mB * (P1 + P2);
                            wB += iB * (MathUtils.Cross(cp1.rB, P1) + MathUtils.Cross(cp2.rB, P2));

                            // Accumulate
                            cp1.NormalImpulse = x.x;
                            cp2.NormalImpulse = x.y;

                            break;
                        }

                        // No solution, give up. This is hit sometimes, but it doesn't seem to matter.
                        break;
                    }
                }

                _velocities[indexA].V = vA;
                _velocities[indexA].W = wA;
                _velocities[indexB].V = vB;
                _velocities[indexB].W = wB;
            }
        }

        public void StoreImpulses()
        {
            for (var i = 0; i < _count; ++i)
            {
                var vc = VelocityConstraints[i];
                var manifold = _contacts[vc.ContactIndex].Manifold;

                for (var j = 0; j < vc.PointCount; ++j)
                {
                    var point = manifold.Points[j];
                    point.NormalImpulse = vc.Points[j].NormalImpulse;
                    point.TangentImpulse = vc.Points[j].TangentImpulse;
                    manifold.Points[j] = point;
                }

                _contacts[vc.ContactIndex].Manifold = manifold;
            }
        }

        public bool SolvePositionConstraints()
        {
            var minSeparation = 0.0f;

            for (var i = 0; i < _count; ++i)
            {
                var pc = _positionConstraints[i];

                var indexA = pc.IndexA;
                var indexB = pc.IndexB;
                var localCenterA = pc.LocalCenterA;
                var mA = pc.InvMassA;
                var iA = pc.InvIA;
                var localCenterB = pc.LocalCenterB;
                var mB = pc.InvMassB;
                var iB = pc.InvIB;
                var pointCount = pc.PointCount;

                var cA = _positions[indexA].C;
                var aA = _positions[indexA].A;

                var cB = _positions[indexB].C;
                var aB = _positions[indexB].A;

                // Solve normal constraints
                for (var j = 0; j < pointCount; ++j)
                {
                    var xfA = new Transform();
                    var xfB = new Transform();
                    xfA.q.Set(aA);
                    xfB.q.Set(aB);
                    xfA.p = cA - MathUtils.Mul(xfA.q, localCenterA);
                    xfB.p = cB - MathUtils.Mul(xfB.q, localCenterB);

                    PositionSolverManifold.Initialize(pc, xfA, xfB, j, out var normal, out var point,
                        out var separation);

                    var rA = point - cA;
                    var rB = point - cB;

                    // Track max constraint error.
                    minSeparation = Mathf.Min(minSeparation, separation);

                    // Prevent large corrections and allow slop.
                    var C = MathUtils.Clamp(Settings.Baumgarte * (separation + Settings.LinearSlop),
                        -Settings.MaxLinearCorrection, 0.0f);

                    // Compute the effective mass.
                    var rnA = MathUtils.Cross(rA, normal);
                    var rnB = MathUtils.Cross(rB, normal);
                    var K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

                    // Compute normal impulse
                    var impulse = K > 0.0f ? -C / K : 0.0f;

                    var P = impulse * normal;

                    cA -= mA * P;
                    aA -= iA * MathUtils.Cross(rA, P);

                    cB += mB * P;
                    aB += iB * MathUtils.Cross(rB, P);
                }

                _positions[indexA].C = cA;
                _positions[indexA].A = aA;

                _positions[indexB].C = cB;
                _positions[indexB].A = aB;
            }

            // We can't expect minSpeparation >= -b2_linearSlop because we don't
            // push the separation above -b2_linearSlop.
            return minSeparation >= -3.0f * Settings.LinearSlop;
        }

        // Sequential position solver for position constraints.
        public bool SolveTOIPositionConstraints(int toiIndexA, int toiIndexB)
        {
            var minSeparation = 0.0f;

            for (var i = 0; i < _count; ++i)
            {
                var pc = _positionConstraints[i];

                var indexA = pc.IndexA;
                var indexB = pc.IndexB;
                var localCenterA = pc.LocalCenterA;
                var localCenterB = pc.LocalCenterB;
                var pointCount = pc.PointCount;

                var mA = 0.0f;
                var iA = 0.0f;
                if (indexA == toiIndexA || indexA == toiIndexB)
                {
                    mA = pc.InvMassA;
                    iA = pc.InvIA;
                }

                var mB = 0.0f;
                var iB = 0.0f;
                if (indexB == toiIndexA || indexB == toiIndexB)
                {
                    mB = pc.InvMassB;
                    iB = pc.InvIB;
                }

                var cA = _positions[indexA].C;
                var aA = _positions[indexA].A;

                var cB = _positions[indexB].C;
                var aB = _positions[indexB].A;

                // Solve normal constraints
                for (var j = 0; j < pointCount; ++j)
                {
                    var xfA = new Transform();
                    var xfB = new Transform();
                    xfA.q.Set(aA);
                    xfB.q.Set(aB);
                    xfA.p = cA - MathUtils.Mul(xfA.q, localCenterA);
                    xfB.p = cB - MathUtils.Mul(xfB.q, localCenterB);

                    PositionSolverManifold.Initialize(pc, xfA, xfB, j, out var normal, out var point,
                        out var separation);

                    var rA = point - cA;
                    var rB = point - cB;

                    // Track max constraint error.
                    minSeparation = Mathf.Min(minSeparation, separation);

                    // Prevent large corrections and allow slop.
                    var C = MathUtils.Clamp(Settings.Baumgarte * (separation + Settings.LinearSlop),
                        -Settings.MaxLinearCorrection, 0.0f);

                    // Compute the effective mass.
                    var rnA = MathUtils.Cross(rA, normal);
                    var rnB = MathUtils.Cross(rB, normal);
                    var K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

                    // Compute normal impulse
                    var impulse = K > 0.0f ? -C / K : 0.0f;

                    var P = impulse * normal;

                    cA -= mA * P;
                    aA -= iA * MathUtils.Cross(rA, P);

                    cB += mB * P;
                    aB += iB * MathUtils.Cross(rB, P);
                }

                _positions[indexA].C = cA;
                _positions[indexA].A = aA;

                _positions[indexB].C = cB;
                _positions[indexB].A = aB;
            }

            // We can't expect minSpeparation >= -b2_linearSlop because we don't
            // push the separation above -b2_linearSlop.
            return minSeparation >= -1.5f * Settings.LinearSlop;
        }
    }
}