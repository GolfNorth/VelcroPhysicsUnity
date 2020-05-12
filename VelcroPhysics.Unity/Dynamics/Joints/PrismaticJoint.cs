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
using VelcroPhysics.Dynamics.Solver;
using VelcroPhysics.Shared;
using VelcroPhysics.Utilities;
using Transform = VelcroPhysics.Shared.Transform;

namespace VelcroPhysics.Dynamics.Joints
{
    // Linear constraint (point-to-line)
    // d = p2 - p1 = x2 + r2 - x1 - r1
    // C = dot(perp, d)
    // Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
    //      = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
    // J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
    //
    // Angular constraint
    // C = a2 - a1 + a_initial
    // Cdot = w2 - w1
    // J = [0 0 -1 0 0 1]
    //
    // K = J * invM * JT
    //
    // J = [-a -s1 a s2]
    //     [0  -1  0  1]
    // a = perp
    // s1 = cross(d + r1, a) = cross(p2 - x1, a)
    // s2 = cross(r2, a) = cross(p2 - x2, a)
    // Motor/Limit linear constraint
    // C = dot(ax1, d)
    // Cdot = = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
    // J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]
    // Block Solver
    // We develop a block solver that includes the joint limit. This makes the limit stiff (inelastic) even
    // when the mass has poor distribution (leading to large torques about the joint anchor points).
    //
    // The Jacobian has 3 rows:
    // J = [-uT -s1 uT s2] // linear
    //     [0   -1   0  1] // angular
    //     [-vT -a1 vT a2] // limit
    //
    // u = perp
    // v = axis
    // s1 = cross(d + r1, u), s2 = cross(r2, u)
    // a1 = cross(d + r1, v), a2 = cross(r2, v)
    // M * (v2 - v1) = JT * df
    // J * v2 = bias
    //
    // v2 = v1 + invM * JT * df
    // J * (v1 + invM * JT * df) = bias
    // K * df = bias - J * v1 = -Cdot
    // K = J * invM * JT
    // Cdot = J * v1 - bias
    //
    // Now solve for f2.
    // df = f2 - f1
    // K * (f2 - f1) = -Cdot
    // f2 = invK * (-Cdot) + f1
    //
    // Clamp accumulated limit impulse.
    // lower: f2(3) = max(f2(3), 0)
    // upper: f2(3) = min(f2(3), 0)
    //
    // Solve for correct f2(1:2)
    // K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:3) * f1
    //                       = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:2) * f1(1:2) + K(1:2,3) * f1(3)
    // K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3)) + K(1:2,1:2) * f1(1:2)
    // f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
    //
    // Now compute impulse to be applied:
    // df = f2 - f1

    /// <summary>
    /// A prismatic joint. This joint provides one degree of freedom: translation
    /// along an axis fixed in bodyA. Relative rotation is prevented. You can
    /// use a joint limit to restrict the range of motion and a joint motor to
    /// drive the motion or to model joint friction.
    /// </summary>
    public class PrismaticJoint : Joint
    {
        private float _a1, _a2;
        private Vector2 _axis, _perp;
        private Vector2 _axis1;
        private bool _enableLimit;
        private bool _enableMotor;
        private Vector3 _impulse;

        // Solver temp
        private int _indexA;

        private int _indexB;
        private float _invIA;
        private float _invIB;
        private float _invMassA;
        private float _invMassB;
        private Mat33 _K;
        private LimitState _limitState;
        private Vector2 _localCenterA;
        private Vector2 _localCenterB;
        private Vector2 _localYAxisA;
        private float _lowerTranslation;
        private float _maxMotorForce;
        private float _motorMass;
        private float _motorSpeed;
        private float _s1, _s2;
        private float _upperTranslation;

        internal PrismaticJoint()
        {
            JointType = JointType.Prismatic;
        }

        /// <summary>
        /// This requires defining a line of
        /// motion using an axis and an anchor point. The definition uses local
        /// anchor points and a local axis so that the initial configuration
        /// can violate the constraint slightly. The joint translation is zero
        /// when the local anchor points coincide in world space. Using local
        /// anchors and a local axis helps when saving and loading a game.
        /// </summary>
        /// <param name="bodyA">The first body.</param>
        /// <param name="bodyB">The second body.</param>
        /// <param name="anchorA">The first body anchor.</param>
        /// <param name="anchorB">The second body anchor.</param>
        /// <param name="axis">The axis.</param>
        /// <param name="useWorldCoordinates">Set to true if you are using world coordinates as anchors.</param>
        public PrismaticJoint(Body bodyA, Body bodyB, Vector2 anchorA, Vector2 anchorB, Vector2 axis,
            bool useWorldCoordinates = false)
            : base(bodyA, bodyB)
        {
            Initialize(anchorA, anchorB, axis, useWorldCoordinates);
        }

        public PrismaticJoint(Body bodyA, Body bodyB, Vector2 anchor, Vector2 axis, bool useWorldCoordinates = false)
            : base(bodyA, bodyB)
        {
            Initialize(anchor, anchor, axis, useWorldCoordinates);
        }

        /// <summary>
        /// The local anchor point on BodyA
        /// </summary>
        public Vector2 LocalAnchorA { get; set; }

        /// <summary>
        /// The local anchor point on BodyB
        /// </summary>
        public Vector2 LocalAnchorB { get; set; }

        public override Vector2 WorldAnchorA
        {
            get => BodyA.GetWorldPoint(LocalAnchorA);
            set => LocalAnchorA = BodyA.GetLocalPoint(value);
        }

        public override Vector2 WorldAnchorB
        {
            get => BodyB.GetWorldPoint(LocalAnchorB);
            set => LocalAnchorB = BodyB.GetLocalPoint(value);
        }

        /// <summary>
        /// Get the current joint translation, usually in meters.
        /// </summary>
        /// <value></value>
        public float JointTranslation
        {
            get
            {
                var d = BodyB.GetWorldPoint(LocalAnchorB) - BodyA.GetWorldPoint(LocalAnchorA);
                var axis = BodyA.GetWorldVector(LocalXAxis);

                return Vector2.Dot(d, axis);
            }
        }

        /// <summary>
        /// Get the current joint translation speed, usually in meters per second.
        /// </summary>
        /// <value></value>
        public float JointSpeed
        {
            get
            {
                Transform xf1, xf2;
                BodyA.GetTransform(out xf1);
                BodyB.GetTransform(out xf2);

                var r1 = MathUtils.Mul(ref xf1.q, LocalAnchorA - BodyA.LocalCenter);
                var r2 = MathUtils.Mul(ref xf2.q, LocalAnchorB - BodyB.LocalCenter);
                var p1 = BodyA._sweep.C + r1;
                var p2 = BodyB._sweep.C + r2;
                var d = p2 - p1;
                var axis = BodyA.GetWorldVector(LocalXAxis);

                var v1 = BodyA._linearVelocity;
                var v2 = BodyB._linearVelocity;
                var w1 = BodyA._angularVelocity;
                var w2 = BodyB._angularVelocity;

                var speed = Vector2.Dot(d, MathUtils.Cross(w1, axis)) + Vector2.Dot(axis,
                    v2 + MathUtils.Cross(w2, r2) - v1 - MathUtils.Cross(w1, r1));
                return speed;
            }
        }

        /// <summary>
        /// Is the joint limit enabled?
        /// </summary>
        /// <value><c>true</c> if [limit enabled]; otherwise, <c>false</c>.</value>
        public bool LimitEnabled
        {
            get => _enableLimit;
            set
            {
                Debug.Assert(BodyA.FixedRotation == false || BodyB.FixedRotation == false,
                    "Warning: limits does currently not work with fixed rotation");

                if (value == _enableLimit)
                    return;

                WakeBodies();
                _enableLimit = value;
                _impulse.z = 0;
            }
        }

        /// <summary>
        /// Get the lower joint limit, usually in meters.
        /// </summary>
        /// <value></value>
        public float LowerLimit
        {
            get => _lowerTranslation;
            set
            {
                if (value == _lowerTranslation)
                    return;

                WakeBodies();
                _lowerTranslation = value;
                _impulse.z = 0.0f;
            }
        }

        /// <summary>
        /// Get the upper joint limit, usually in meters.
        /// </summary>
        /// <value></value>
        public float UpperLimit
        {
            get => _upperTranslation;
            set
            {
                if (value == _upperTranslation)
                    return;

                WakeBodies();
                _upperTranslation = value;
                _impulse.z = 0.0f;
            }
        }

        /// <summary>
        /// Is the joint motor enabled?
        /// </summary>
        /// <value><c>true</c> if [motor enabled]; otherwise, <c>false</c>.</value>
        public bool MotorEnabled
        {
            get => _enableMotor;
            set
            {
                if (value == _enableMotor)
                    return;

                WakeBodies();
                _enableMotor = value;
            }
        }

        /// <summary>
        /// Set the motor speed, usually in meters per second.
        /// </summary>
        /// <value>The speed.</value>
        public float MotorSpeed
        {
            set
            {
                if (value == _motorSpeed)
                    return;

                WakeBodies();
                _motorSpeed = value;
            }
            get => _motorSpeed;
        }

        /// <summary>
        /// Set the maximum motor force, usually in N.
        /// </summary>
        /// <value>The force.</value>
        public float MaxMotorForce
        {
            get => _maxMotorForce;
            set
            {
                if (value == _maxMotorForce)
                    return;

                WakeBodies();
                _maxMotorForce = value;
            }
        }

        /// <summary>
        /// Get the current motor impulse, usually in N.
        /// </summary>
        /// <value></value>
        public float MotorImpulse { get; set; }

        /// <summary>
        /// The axis at which the joint moves.
        /// </summary>
        public Vector2 Axis
        {
            get => _axis1;
            set
            {
                _axis1 = value;
                LocalXAxis = BodyA.GetLocalVector(_axis1);
                LocalXAxis.Normalize();
                _localYAxisA = MathUtils.Cross(1.0f, LocalXAxis);
            }
        }

        /// <summary>
        /// The axis in local coordinates relative to BodyA
        /// </summary>
        public Vector2 LocalXAxis { get; private set; }

        /// <summary>
        /// The reference angle.
        /// </summary>
        public float ReferenceAngle { get; set; }

        private void Initialize(Vector2 localAnchorA, Vector2 localAnchorB, Vector2 axis, bool useWorldCoordinates)
        {
            JointType = JointType.Prismatic;

            if (useWorldCoordinates)
            {
                LocalAnchorA = BodyA.GetLocalPoint(localAnchorA);
                LocalAnchorB = BodyB.GetLocalPoint(localAnchorB);
            }
            else
            {
                LocalAnchorA = localAnchorA;
                LocalAnchorB = localAnchorB;
            }

            Axis = axis; //Velcro only: store the orignal value for use in Serialization
            ReferenceAngle = BodyB.Rotation - BodyA.Rotation;

            _limitState = LimitState.Inactive;
        }

        /// <summary>
        /// Set the joint limits, usually in meters.
        /// </summary>
        /// <param name="lower">The lower limit</param>
        /// <param name="upper">The upper limit</param>
        public void SetLimits(float lower, float upper)
        {
            if (upper == _upperTranslation && lower == _lowerTranslation)
                return;

            WakeBodies();
            _upperTranslation = upper;
            _lowerTranslation = lower;
            _impulse.z = 0.0f;
        }

        /// <summary>
        /// Gets the motor force.
        /// </summary>
        /// <param name="invDt">The inverse delta time</param>
        public float GetMotorForce(float invDt)
        {
            return invDt * MotorImpulse;
        }

        public override Vector2 GetReactionForce(float invDt)
        {
            return invDt * (_impulse.x * _perp + (MotorImpulse + _impulse.z) * _axis);
        }

        public override float GetReactionTorque(float invDt)
        {
            return invDt * _impulse.y;
        }

        internal override void InitVelocityConstraints(ref SolverData data)
        {
            _indexA = BodyA.IslandIndex;
            _indexB = BodyB.IslandIndex;
            _localCenterA = BodyA._sweep.LocalCenter;
            _localCenterB = BodyB._sweep.LocalCenter;
            _invMassA = BodyA._invMass;
            _invMassB = BodyB._invMass;
            _invIA = BodyA._invI;
            _invIB = BodyB._invI;

            var cA = data.Positions[_indexA].C;
            var aA = data.Positions[_indexA].A;
            var vA = data.Velocities[_indexA].V;
            var wA = data.Velocities[_indexA].W;

            var cB = data.Positions[_indexB].C;
            var aB = data.Positions[_indexB].A;
            var vB = data.Velocities[_indexB].V;
            var wB = data.Velocities[_indexB].W;

            Rot qA = new Rot(aA), qB = new Rot(aB);

            // Compute the effective masses.
            var rA = MathUtils.Mul(qA, LocalAnchorA - _localCenterA);
            var rB = MathUtils.Mul(qB, LocalAnchorB - _localCenterB);
            var d = cB - cA + rB - rA;

            float mA = _invMassA, mB = _invMassB;
            float iA = _invIA, iB = _invIB;

            // Compute motor Jacobian and effective mass.
            {
                _axis = MathUtils.Mul(qA, LocalXAxis);
                _a1 = MathUtils.Cross(d + rA, _axis);
                _a2 = MathUtils.Cross(rB, _axis);

                _motorMass = mA + mB + iA * _a1 * _a1 + iB * _a2 * _a2;
                if (_motorMass > 0.0f) _motorMass = 1.0f / _motorMass;
            }

            // Prismatic constraint.
            {
                _perp = MathUtils.Mul(qA, _localYAxisA);

                _s1 = MathUtils.Cross(d + rA, _perp);
                _s2 = MathUtils.Cross(rB, _perp);

                var k11 = mA + mB + iA * _s1 * _s1 + iB * _s2 * _s2;
                var k12 = iA * _s1 + iB * _s2;
                var k13 = iA * _s1 * _a1 + iB * _s2 * _a2;
                var k22 = iA + iB;
                if (k22 == 0.0f)
                    // For bodies with fixed rotation.
                    k22 = 1.0f;
                var k23 = iA * _a1 + iB * _a2;
                var k33 = mA + mB + iA * _a1 * _a1 + iB * _a2 * _a2;

                _K.ex = new Vector3(k11, k12, k13);
                _K.ey = new Vector3(k12, k22, k23);
                _K.ez = new Vector3(k13, k23, k33);
            }

            // Compute motor and limit terms.
            if (_enableLimit)
            {
                var jointTranslation = Vector2.Dot(_axis, d);
                if (Mathf.Abs(_upperTranslation - _lowerTranslation) < 2.0f * Settings.LinearSlop)
                {
                    _limitState = LimitState.Equal;
                }
                else if (jointTranslation <= _lowerTranslation)
                {
                    if (_limitState != LimitState.AtLower)
                    {
                        _limitState = LimitState.AtLower;
                        _impulse.z = 0.0f;
                    }
                }
                else if (jointTranslation >= _upperTranslation)
                {
                    if (_limitState != LimitState.AtUpper)
                    {
                        _limitState = LimitState.AtUpper;
                        _impulse.z = 0.0f;
                    }
                }
                else
                {
                    _limitState = LimitState.Inactive;
                    _impulse.z = 0.0f;
                }
            }
            else
            {
                _limitState = LimitState.Inactive;
                _impulse.z = 0.0f;
            }

            if (_enableMotor == false) MotorImpulse = 0.0f;

            if (Settings.EnableWarmstarting)
            {
                // Account for variable time step.
                _impulse *= data.Step.dtRatio;
                MotorImpulse *= data.Step.dtRatio;

                var P = _impulse.x * _perp + (MotorImpulse + _impulse.z) * _axis;
                var LA = _impulse.x * _s1 + _impulse.y + (MotorImpulse + _impulse.z) * _a1;
                var LB = _impulse.x * _s2 + _impulse.y + (MotorImpulse + _impulse.z) * _a2;

                vA -= mA * P;
                wA -= iA * LA;

                vB += mB * P;
                wB += iB * LB;
            }
            else
            {
                _impulse = Vector3.zero;
                MotorImpulse = 0.0f;
            }

            data.Velocities[_indexA].V = vA;
            data.Velocities[_indexA].W = wA;
            data.Velocities[_indexB].V = vB;
            data.Velocities[_indexB].W = wB;
        }

        internal override void SolveVelocityConstraints(ref SolverData data)
        {
            var vA = data.Velocities[_indexA].V;
            var wA = data.Velocities[_indexA].W;
            var vB = data.Velocities[_indexB].V;
            var wB = data.Velocities[_indexB].W;

            float mA = _invMassA, mB = _invMassB;
            float iA = _invIA, iB = _invIB;

            // Solve linear motor constraint.
            if (_enableMotor && _limitState != LimitState.Equal)
            {
                var Cdot = Vector2.Dot(_axis, vB - vA) + _a2 * wB - _a1 * wA;
                var impulse = _motorMass * (_motorSpeed - Cdot);
                var oldImpulse = MotorImpulse;
                var maxImpulse = data.Step.dt * _maxMotorForce;
                MotorImpulse = MathUtils.Clamp(MotorImpulse + impulse, -maxImpulse, maxImpulse);
                impulse = MotorImpulse - oldImpulse;

                var P = impulse * _axis;
                var LA = impulse * _a1;
                var LB = impulse * _a2;

                vA -= mA * P;
                wA -= iA * LA;

                vB += mB * P;
                wB += iB * LB;
            }

            var Cdot1 = new Vector2();
            Cdot1.x = Vector2.Dot(_perp, vB - vA) + _s2 * wB - _s1 * wA;
            Cdot1.y = wB - wA;

            if (_enableLimit && _limitState != LimitState.Inactive)
            {
                // Solve prismatic and limit constraint in block form.
                float Cdot2;
                Cdot2 = Vector2.Dot(_axis, vB - vA) + _a2 * wB - _a1 * wA;
                var Cdot = new Vector3(Cdot1.x, Cdot1.y, Cdot2);

                var f1 = _impulse;
                var df = _K.Solve33(-Cdot);
                _impulse += df;

                if (_limitState == LimitState.AtLower)
                    _impulse.z = Mathf.Max(_impulse.z, 0.0f);
                else if (_limitState == LimitState.AtUpper) _impulse.z = Mathf.Min(_impulse.z, 0.0f);

                // f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
                var b = -Cdot1 - (_impulse.z - f1.z) * new Vector2(_K.ez.x, _K.ez.y);
                var f2r = _K.Solve22(b) + new Vector2(f1.x, f1.y);
                _impulse.x = f2r.x;
                _impulse.y = f2r.y;

                df = _impulse - f1;

                var P = df.x * _perp + df.z * _axis;
                var LA = df.x * _s1 + df.y + df.z * _a1;
                var LB = df.x * _s2 + df.y + df.z * _a2;

                vA -= mA * P;
                wA -= iA * LA;

                vB += mB * P;
                wB += iB * LB;
            }
            else
            {
                // Limit is inactive, just solve the prismatic constraint in block form.
                var df = _K.Solve22(-Cdot1);
                _impulse.x += df.x;
                _impulse.y += df.y;

                var P = df.x * _perp;
                var LA = df.x * _s1 + df.y;
                var LB = df.x * _s2 + df.y;

                vA -= mA * P;
                wA -= iA * LA;

                vB += mB * P;
                wB += iB * LB;
            }

            data.Velocities[_indexA].V = vA;
            data.Velocities[_indexA].W = wA;
            data.Velocities[_indexB].V = vB;
            data.Velocities[_indexB].W = wB;
        }

        internal override bool SolvePositionConstraints(ref SolverData data)
        {
            var cA = data.Positions[_indexA].C;
            var aA = data.Positions[_indexA].A;
            var cB = data.Positions[_indexB].C;
            var aB = data.Positions[_indexB].A;

            Rot qA = new Rot(aA), qB = new Rot(aB);

            float mA = _invMassA, mB = _invMassB;
            float iA = _invIA, iB = _invIB;

            // Compute fresh Jacobians
            var rA = MathUtils.Mul(qA, LocalAnchorA - _localCenterA);
            var rB = MathUtils.Mul(qB, LocalAnchorB - _localCenterB);
            var d = cB + rB - cA - rA;

            var axis = MathUtils.Mul(qA, LocalXAxis);
            var a1 = MathUtils.Cross(d + rA, axis);
            var a2 = MathUtils.Cross(rB, axis);
            var perp = MathUtils.Mul(qA, _localYAxisA);

            var s1 = MathUtils.Cross(d + rA, perp);
            var s2 = MathUtils.Cross(rB, perp);

            Vector3 impulse;
            var C1 = new Vector2();
            C1.x = Vector2.Dot(perp, d);
            C1.y = aB - aA - ReferenceAngle;

            var linearError = Mathf.Abs(C1.x);
            var angularError = Mathf.Abs(C1.y);

            var active = false;
            var C2 = 0.0f;
            if (_enableLimit)
            {
                var translation = Vector2.Dot(axis, d);
                if (Mathf.Abs(_upperTranslation - _lowerTranslation) < 2.0f * Settings.LinearSlop)
                {
                    // Prevent large angular corrections
                    C2 = MathUtils.Clamp(translation, -Settings.MaxLinearCorrection, Settings.MaxLinearCorrection);
                    linearError = Mathf.Max(linearError, Mathf.Abs(translation));
                    active = true;
                }
                else if (translation <= _lowerTranslation)
                {
                    // Prevent large linear corrections and allow some slop.
                    C2 = MathUtils.Clamp(translation - _lowerTranslation + Settings.LinearSlop,
                        -Settings.MaxLinearCorrection, 0.0f);
                    linearError = Mathf.Max(linearError, _lowerTranslation - translation);
                    active = true;
                }
                else if (translation >= _upperTranslation)
                {
                    // Prevent large linear corrections and allow some slop.
                    C2 = MathUtils.Clamp(translation - _upperTranslation - Settings.LinearSlop, 0.0f,
                        Settings.MaxLinearCorrection);
                    linearError = Mathf.Max(linearError, translation - _upperTranslation);
                    active = true;
                }
            }

            if (active)
            {
                var k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
                var k12 = iA * s1 + iB * s2;
                var k13 = iA * s1 * a1 + iB * s2 * a2;
                var k22 = iA + iB;
                if (k22 == 0.0f)
                    // For fixed rotation
                    k22 = 1.0f;
                var k23 = iA * a1 + iB * a2;
                var k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;

                var K = new Mat33();
                K.ex = new Vector3(k11, k12, k13);
                K.ey = new Vector3(k12, k22, k23);
                K.ez = new Vector3(k13, k23, k33);

                var C = new Vector3();
                C.x = C1.x;
                C.y = C1.y;
                C.z = C2;

                impulse = K.Solve33(-C);
            }
            else
            {
                var k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
                var k12 = iA * s1 + iB * s2;
                var k22 = iA + iB;
                if (k22 == 0.0f) k22 = 1.0f;

                var K = new Mat22();
                K.ex = new Vector2(k11, k12);
                K.ey = new Vector2(k12, k22);

                var impulse1 = K.Solve(-C1);
                impulse = new Vector3();
                impulse.x = impulse1.x;
                impulse.y = impulse1.y;
                impulse.z = 0.0f;
            }

            var P = impulse.x * perp + impulse.z * axis;
            var LA = impulse.x * s1 + impulse.y + impulse.z * a1;
            var LB = impulse.x * s2 + impulse.y + impulse.z * a2;

            cA -= mA * P;
            aA -= iA * LA;
            cB += mB * P;
            aB += iB * LB;

            data.Positions[_indexA].C = cA;
            data.Positions[_indexA].A = aA;
            data.Positions[_indexB].C = cB;
            data.Positions[_indexB].A = aB;

            return linearError <= Settings.LinearSlop && angularError <= Settings.AngularSlop;
        }
    }
}