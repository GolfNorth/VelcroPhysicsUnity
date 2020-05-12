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

namespace VelcroPhysics.Dynamics.Joints
{
    // p = attached point, m = mouse point
    // C = p - m
    // Cdot = v
    //      = v + cross(w, r)
    // J = [I r_skew]
    // Identity used:
    // w k % (rx i + ry j) = w * (-ry i + rx j)

    /// <summary>
    /// A mouse joint is used to make a point on a body track a
    /// specified world point. This a soft constraint with a maximum
    /// force. This allows the constraint to stretch and without
    /// applying huge forces.
    /// NOTE: this joint is not documented in the manual because it was
    /// developed to be used in the testbed. If you want to learn how to
    /// use the mouse joint, look at the testbed.
    /// </summary>
    public class FixedMouseJoint : Joint
    {
        private float _beta;
        private Vector2 _C;
        private float _dampingRatio;
        private float _frequency;
        private float _gamma;

        // Solver shared
        private Vector2 _impulse;

        // Solver temp
        private int _indexA;

        private float _invIA;
        private float _invMassA;
        private Vector2 _localCenterA;
        private Mat22 _mass;
        private float _maxForce;
        private Vector2 _rA;
        private Vector2 _worldAnchor;

        /// <summary>
        /// This requires a world target point,
        /// tuning parameters, and the time step.
        /// </summary>
        /// <param name="body">The body.</param>
        /// <param name="worldAnchor">The target.</param>
        public FixedMouseJoint(Body body, Vector2 worldAnchor)
            : base(body)
        {
            JointType = JointType.FixedMouse;
            Frequency = 5.0f;
            DampingRatio = 0.7f;
            MaxForce = 1000 * body.Mass;

            Debug.Assert(worldAnchor.IsValid());

            _worldAnchor = worldAnchor;
            LocalAnchorA = MathUtils.MulT(BodyA._xf, worldAnchor);
        }

        /// <summary>
        /// The local anchor point on BodyA
        /// </summary>
        public Vector2 LocalAnchorA { get; set; }

        public override Vector2 WorldAnchorA
        {
            get => BodyA.GetWorldPoint(LocalAnchorA);
            set => LocalAnchorA = BodyA.GetLocalPoint(value);
        }

        public override Vector2 WorldAnchorB
        {
            get => _worldAnchor;
            set
            {
                WakeBodies();
                _worldAnchor = value;
            }
        }

        /// <summary>
        /// The maximum constraint force that can be exerted
        /// to move the candidate body. Usually you will express
        /// as some multiple of the weight (multiplier * mass * gravity).
        /// </summary>
        public float MaxForce
        {
            get => _maxForce;
            set
            {
                Debug.Assert(MathUtils.IsValid(value) && value >= 0.0f);
                _maxForce = value;
            }
        }

        /// <summary>
        /// The response speed.
        /// </summary>
        public float Frequency
        {
            get => _frequency;
            set
            {
                Debug.Assert(MathUtils.IsValid(value) && value >= 0.0f);
                _frequency = value;
            }
        }

        /// <summary>
        /// The damping ratio. 0 = no damping, 1 = critical damping.
        /// </summary>
        public float DampingRatio
        {
            get => _dampingRatio;
            set
            {
                Debug.Assert(MathUtils.IsValid(value) && value >= 0.0f);
                _dampingRatio = value;
            }
        }

        public override Vector2 GetReactionForce(float invDt)
        {
            return invDt * _impulse;
        }

        public override float GetReactionTorque(float invDt)
        {
            return invDt * 0.0f;
        }

        internal override void InitVelocityConstraints(ref SolverData data)
        {
            _indexA = BodyA.IslandIndex;
            _localCenterA = BodyA._sweep.LocalCenter;
            _invMassA = BodyA._invMass;
            _invIA = BodyA._invI;

            var cA = data.Positions[_indexA].C;
            var aA = data.Positions[_indexA].A;
            var vA = data.Velocities[_indexA].V;
            var wA = data.Velocities[_indexA].W;

            var qA = new Rot(aA);

            var mass = BodyA.Mass;

            // Frequency
            var omega = 2.0f * Settings.Pi * Frequency;

            // Damping coefficient
            var d = 2.0f * mass * DampingRatio * omega;

            // Spring stiffness
            var k = mass * (omega * omega);

            // magic formulas
            // gamma has units of inverse mass.
            // beta has units of inverse time.
            var h = data.Step.dt;
            Debug.Assert(d + h * k > Settings.Epsilon);
            _gamma = h * (d + h * k);
            if (_gamma != 0.0f) _gamma = 1.0f / _gamma;

            _beta = h * k * _gamma;

            // Compute the effective mass matrix.
            _rA = MathUtils.Mul(qA, LocalAnchorA - _localCenterA);

            // K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
            //      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
            //        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
            var K = new Mat22();
            K.ex.x = _invMassA + _invIA * _rA.y * _rA.y + _gamma;
            K.ex.y = -_invIA * _rA.x * _rA.y;
            K.ey.x = K.ex.y;
            K.ey.y = _invMassA + _invIA * _rA.x * _rA.x + _gamma;

            _mass = K.Inverse;

            _C = cA + _rA - _worldAnchor;
            _C *= _beta;

            // Cheat with some damping
            wA *= 0.98f;

            if (Settings.EnableWarmstarting)
            {
                _impulse *= data.Step.dtRatio;
                vA += _invMassA * _impulse;
                wA += _invIA * MathUtils.Cross(_rA, _impulse);
            }
            else
            {
                _impulse = Vector2.zero;
            }

            data.Velocities[_indexA].V = vA;
            data.Velocities[_indexA].W = wA;
        }

        internal override void SolveVelocityConstraints(ref SolverData data)
        {
            var vA = data.Velocities[_indexA].V;
            var wA = data.Velocities[_indexA].W;

            // Cdot = v + cross(w, r)
            var Cdot = vA + MathUtils.Cross(wA, _rA);
            var impulse = MathUtils.Mul(ref _mass, -(Cdot + _C + _gamma * _impulse));

            var oldImpulse = _impulse;
            _impulse += impulse;
            var maxImpulse = data.Step.dt * MaxForce;
            if (_impulse.sqrMagnitude > maxImpulse * maxImpulse) _impulse *= maxImpulse / _impulse.magnitude;
            impulse = _impulse - oldImpulse;

            vA += _invMassA * impulse;
            wA += _invIA * MathUtils.Cross(_rA, impulse);

            data.Velocities[_indexA].V = vA;
            data.Velocities[_indexA].W = wA;
        }

        internal override bool SolvePositionConstraints(ref SolverData data)
        {
            return true;
        }
    }
}