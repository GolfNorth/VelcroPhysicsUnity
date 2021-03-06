﻿/*
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

using System.Diagnostics;
using UnityEngine;
using VelcroPhysics.Collision.ContactSystem;
using VelcroPhysics.Dynamics.Joints;
using VelcroPhysics.Utilities;
using Debug = UnityEngine.Debug;

namespace VelcroPhysics.Dynamics.Solver
{
    /// <summary>
    /// This is an internal class.
    /// </summary>
    public class Island
    {
        private const float LinTolSqr = Settings.LinearSleepTolerance * Settings.LinearSleepTolerance;
        private const float AngTolSqr = Settings.AngularSleepTolerance * Settings.AngularSleepTolerance;
        private ContactManager _contactManager;
        private Contact[] _contacts;
        private ContactSolver _contactSolver = new ContactSolver();
        private Joint[] _joints;
        private Stopwatch _watch = new Stopwatch();

        public Body[] Bodies;
        public Position[] Positions;
        public Velocity[] Velocities;
        public int BodyCapacity;
        public int BodyCount;
        public int ContactCapacity;
        public int ContactCount;
        public int JointCapacity;
        public int JointCount;
        public float JointUpdateTime;

        public void Reset(int bodyCapacity, int contactCapacity, int jointCapacity, ContactManager contactManager)
        {
            BodyCapacity = bodyCapacity;
            ContactCapacity = contactCapacity;
            JointCapacity = jointCapacity;
            BodyCount = 0;
            ContactCount = 0;
            JointCount = 0;

            _contactManager = contactManager;

            if (Bodies == null || Bodies.Length < bodyCapacity)
            {
                Bodies = new Body[bodyCapacity];
                Velocities = new Velocity[bodyCapacity];
                Positions = new Position[bodyCapacity];
            }

            if (_contacts == null || _contacts.Length < contactCapacity) _contacts = new Contact[contactCapacity * 2];

            if (_joints == null || _joints.Length < jointCapacity) _joints = new Joint[jointCapacity * 2];
        }

        public void Clear()
        {
            BodyCount = 0;
            ContactCount = 0;
            JointCount = 0;
        }

        public void Solve(ref TimeStep step, ref Vector2 gravity)
        {
            var h = step.dt;

            // Integrate velocities and apply damping. Initialize the body state.
            for (var i = 0; i < BodyCount; ++i)
            {
                var b = Bodies[i];

                var c = b._sweep.C;
                var a = b._sweep.A;
                var v = b._linearVelocity;
                var w = b._angularVelocity;

                // Store positions for continuous collision.
                b._sweep.C0 = b._sweep.C;
                b._sweep.A0 = b._sweep.A;

                if (b.BodyType == BodyType.Dynamic)
                {
                    // Integrate velocities.

                    //Velcro: Only apply gravity if the body wants it.
                    if (b.IgnoreGravity)
                        v += h * (b._invMass * b._force);
                    else
                        v += h * (b.GravityScale * gravity + b._invMass * b._force);

                    w += h * b._invI * b._torque;

                    // Apply damping.
                    // ODE: dv/dt + c * v = 0
                    // Solution: v(t) = v0 * exp(-c * t)
                    // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
                    // v2 = exp(-c * dt) * v1
                    // Taylor expansion:
                    // v2 = (1.0f - c * dt) * v1
                    v *= MathUtils.Clamp(1.0f - h * b.LinearDamping, 0.0f, 1.0f);
                    w *= MathUtils.Clamp(1.0f - h * b.AngularDamping, 0.0f, 1.0f);
                }

                Positions[i].C = c;
                Positions[i].A = a;
                Velocities[i].V = v;
                Velocities[i].W = w;
            }

            // Solver data
            var solverData = new SolverData();
            solverData.Step = step;
            solverData.Positions = Positions;
            solverData.Velocities = Velocities;

            _contactSolver.Reset(step, ContactCount, _contacts, Positions, Velocities);
            _contactSolver.InitializeVelocityConstraints();

            if (Settings.EnableWarmstarting) _contactSolver.WarmStart();

            if (Settings.EnableDiagnostics)
                _watch.Start();

            for (var i = 0; i < JointCount; ++i)
                if (_joints[i].Enabled)
                    _joints[i].InitVelocityConstraints(ref solverData);

            if (Settings.EnableDiagnostics)
                _watch.Stop();

            // Solve velocity constraints.
            for (var i = 0; i < Settings.VelocityIterations; ++i)
            {
                for (var j = 0; j < JointCount; ++j)
                {
                    var joint = _joints[j];

                    if (!joint.Enabled)
                        continue;

                    if (Settings.EnableDiagnostics)
                        _watch.Start();

                    joint.SolveVelocityConstraints(ref solverData);
                    joint.Validate(step.inv_dt);

                    if (Settings.EnableDiagnostics)
                        _watch.Stop();
                }

                _contactSolver.SolveVelocityConstraints();
            }

            // Store impulses for warm starting.
            _contactSolver.StoreImpulses();

            // Integrate positions
            for (var i = 0; i < BodyCount; ++i)
            {
                var c = Positions[i].C;
                var a = Positions[i].A;
                var v = Velocities[i].V;
                var w = Velocities[i].W;

                // Check for large velocities
                var translation = h * v;
                if (Vector2.Dot(translation, translation) > Settings.MaxTranslationSquared)
                {
                    var ratio = Settings.MaxTranslation / translation.magnitude;
                    v *= ratio;
                }

                var rotation = h * w;
                if (rotation * rotation > Settings.MaxRotationSquared)
                {
                    var ratio = Settings.MaxRotation / Mathf.Abs(rotation);
                    w *= ratio;
                }

                // Integrate
                c += h * v;
                a += h * w;

                Positions[i].C = c;
                Positions[i].A = a;
                Velocities[i].V = v;
                Velocities[i].W = w;
            }

            // Solve position constraints
            var positionSolved = false;
            for (var i = 0; i < Settings.PositionIterations; ++i)
            {
                var contactsOkay = _contactSolver.SolvePositionConstraints();

                var jointsOkay = true;
                for (var j = 0; j < JointCount; ++j)
                {
                    var joint = _joints[j];

                    if (!joint.Enabled)
                        continue;

                    if (Settings.EnableDiagnostics)
                        _watch.Start();

                    var jointOkay = joint.SolvePositionConstraints(ref solverData);

                    if (Settings.EnableDiagnostics)
                        _watch.Stop();

                    jointsOkay = jointsOkay && jointOkay;
                }

                if (contactsOkay && jointsOkay)
                {
                    // Exit early if the position errors are small.
                    positionSolved = true;
                    break;
                }
            }

            if (Settings.EnableDiagnostics)
            {
                JointUpdateTime = _watch.ElapsedTicks;
                _watch.Reset();
            }

            // Copy state buffers back to the bodies
            for (var i = 0; i < BodyCount; ++i)
            {
                var body = Bodies[i];
                body._sweep.C = Positions[i].C;
                body._sweep.A = Positions[i].A;
                body._linearVelocity = Velocities[i].V;
                body._angularVelocity = Velocities[i].W;
                body.SynchronizeTransform();
            }

            Report(_contactSolver.VelocityConstraints);

            if (Settings.AllowSleep)
            {
                var minSleepTime = Settings.MaxFloat;

                for (var i = 0; i < BodyCount; ++i)
                {
                    var b = Bodies[i];

                    if (b.BodyType == BodyType.Static)
                        continue;

                    if (!b.SleepingAllowed || b._angularVelocity * b._angularVelocity > AngTolSqr ||
                        Vector2.Dot(b._linearVelocity, b._linearVelocity) > LinTolSqr)
                    {
                        b.SleepTime = 0.0f;
                        minSleepTime = 0.0f;
                    }
                    else
                    {
                        b.SleepTime += h;
                        minSleepTime = Mathf.Min(minSleepTime, b.SleepTime);
                    }
                }

                if (minSleepTime >= Settings.TimeToSleep && positionSolved)
                    for (var i = 0; i < BodyCount; ++i)
                    {
                        var b = Bodies[i];
                        b.Awake = false;
                    }
            }
        }

        internal void SolveTOI(ref TimeStep subStep, int toiIndexA, int toiIndexB)
        {
            Debug.Assert(toiIndexA < BodyCount);
            Debug.Assert(toiIndexB < BodyCount);

            // Initialize the body state.
            for (var i = 0; i < BodyCount; ++i)
            {
                var b = Bodies[i];
                Positions[i].C = b._sweep.C;
                Positions[i].A = b._sweep.A;
                Velocities[i].V = b._linearVelocity;
                Velocities[i].W = b._angularVelocity;
            }

            _contactSolver.Reset(subStep, ContactCount, _contacts, Positions, Velocities);

            // Solve position constraints.
            for (var i = 0; i < Settings.TOIPositionIterations; ++i)
            {
                var contactsOkay = _contactSolver.SolveTOIPositionConstraints(toiIndexA, toiIndexB);
                if (contactsOkay) break;
            }

            // Leap of faith to new safe state.
            Bodies[toiIndexA]._sweep.C0 = Positions[toiIndexA].C;
            Bodies[toiIndexA]._sweep.A0 = Positions[toiIndexA].A;
            Bodies[toiIndexB]._sweep.C0 = Positions[toiIndexB].C;
            Bodies[toiIndexB]._sweep.A0 = Positions[toiIndexB].A;

            // No warm starting is needed for TOI events because warm
            // starting impulses were applied in the discrete solver.
            _contactSolver.InitializeVelocityConstraints();

            // Solve velocity constraints.
            for (var i = 0; i < Settings.TOIVelocityIterations; ++i) _contactSolver.SolveVelocityConstraints();

            // Don't store the TOI contact forces for warm starting
            // because they can be quite large.

            var h = subStep.dt;

            // Integrate positions.
            for (var i = 0; i < BodyCount; ++i)
            {
                var c = Positions[i].C;
                var a = Positions[i].A;
                var v = Velocities[i].V;
                var w = Velocities[i].W;

                // Check for large velocities
                var translation = h * v;
                if (Vector2.Dot(translation, translation) > Settings.MaxTranslationSquared)
                {
                    var ratio = Settings.MaxTranslation / translation.magnitude;
                    v *= ratio;
                }

                var rotation = h * w;
                if (rotation * rotation > Settings.MaxRotationSquared)
                {
                    var ratio = Settings.MaxRotation / Mathf.Abs(rotation);
                    w *= ratio;
                }

                // Integrate
                c += h * v;
                a += h * w;

                Positions[i].C = c;
                Positions[i].A = a;
                Velocities[i].V = v;
                Velocities[i].W = w;

                // Sync bodies
                var body = Bodies[i];
                body._sweep.C = c;
                body._sweep.A = a;
                body._linearVelocity = v;
                body._angularVelocity = w;
                body.SynchronizeTransform();
            }

            Report(_contactSolver.VelocityConstraints);
        }

        public void Add(Body body)
        {
            Debug.Assert(BodyCount < BodyCapacity);
            body.IslandIndex = BodyCount;
            Bodies[BodyCount++] = body;
        }

        public void Add(Contact contact)
        {
            Debug.Assert(ContactCount < ContactCapacity);
            _contacts[ContactCount++] = contact;
        }

        public void Add(Joint joint)
        {
            Debug.Assert(JointCount < JointCapacity);
            _joints[JointCount++] = joint;
        }

        private void Report(ContactVelocityConstraint[] constraints)
        {
            if (_contactManager == null)
                return;

            for (var i = 0; i < ContactCount; ++i)
            {
                var c = _contacts[i];

                //Velcro optimization: We don't store the impulses and send it to the delegate. We just send the whole contact.
                //Velcro feature: added after collision
                c.FixtureA.AfterCollision?.Invoke(c.FixtureA, c.FixtureB, c, constraints[i]);

                c.FixtureB.AfterCollision?.Invoke(c.FixtureB, c.FixtureA, c, constraints[i]);

                _contactManager.PostSolve?.Invoke(c, constraints[i]);
            }
        }
    }
}