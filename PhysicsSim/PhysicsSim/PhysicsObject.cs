using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace PhysicsSim
{
    internal class PhysicsObject
    {
        private const float CEILING = 5.0f;
        private const float WALLS = 20.0f;
        private const float FLOOR = 0.2f;

        private static readonly Vector3 gravity = new Vector3(0.0f, -9.8f, 0.0f);
        //private static readonly Vector3 gravity = new Vector3(0.0f, -1.63f, 0.0f);    // moon gravity

        private bool locked = false;
        public bool Locked { get { return locked; } set { locked = value; } }

        private float mass = 1.0f;
        //private Vector3 massCenter = Vector3.Zero;     //relative to you object's origin

        //Trajectory:
        private Vector3 initialPos = Vector3.Zero;      //DEBUG for Reset() function only
        private Vector3 position = Vector3.Zero;
        private Vector3 new_position = Vector3.Zero;
        public Vector3 Position { get { return position; } set { position = value; } }
        public Vector3 NewPosition { get { return new_position; } set { new_position = value; } }
        private Vector3 velocity = Vector3.Zero;
        private Vector3 new_velocity = Vector3.Zero;
        public Vector3 Velocity { get { return velocity; } set { velocity = value; } }
        private Vector3 external_accel = Vector3.Zero;
        public Vector3 Acceleration { get { return gravity + external_accel; } }
        public Vector3 NetForce { get { return mass * Acceleration; } }

        //Collisions:
        private float restitution = 0.5f;
        private Vector3 Momentum { get { return mass * velocity; } }
        //private Vector3 AngularMomentum;

        private Vector3 KineticEnergy { get { return TranslationalEnergy + body.RotationalEnergy; } }
        private Vector3 TranslationalEnergy { get { return mass * velocity * velocity / 2; } }

        private RigidBody body = new RigidBody(Quaternion.Identity);
        public Quaternion Orientation { get { return body.Orientation; } }

        /// <summary>
        /// PhysicsObject Constructor (assumes moveable object)
        /// </summary>
        /// <param name="pos">The object's initial position in world coordinates</param>
        internal PhysicsObject(Vector3 pos, float scale, Vector3[] points)   //TODO: Implement points
        {
            initialPos = pos;
            position = pos;
            new_position = pos;
        }

        /// <summary>
        /// PhysicsObject Constructor
        /// </summary>
        /// <param name="pos">The object's initial position in world coordinates</param>
        /// <param name="moveable"></param>
        internal PhysicsObject(Vector3 pos, bool locked)
        {
            position = pos;
            this.locked = locked;
        }

        internal void Update(TimeSpan ElapsedTime)
        {
            if (locked == false)
                UpdatePosition(ElapsedTime);
        }

        private void UpdatePosition(TimeSpan ElapsedTime)
        {
            float time = (float)ElapsedTime.TotalSeconds;

            position = new_position;
            velocity = new_velocity;

            // check for floor collision
            if (position.Y <= FLOOR)
            {
                new_position.Y = FLOOR;
                new_velocity.Y *= -1 * restitution;

                if (velocity == Vector3.Zero)
                {
                    external_accel.Y = 0;
                    locked = true;
                }
            }
            else if (position.Y > CEILING)
            {
                //new_position.Y = CEILING;
                //new_velocity.Y *= -1 * restitution;
            }

            // check X collisions
            if (position.X >= WALLS)
            {
                new_position.X = WALLS;
                new_velocity.X *= -1 * restitution;
                external_accel.X = 0;
            }
            else if (position.X <= -WALLS)
            {
                new_position.X = -WALLS;
                new_velocity.X *= -1 * restitution;
                external_accel.X = 0;
            }

            // check Z collisions
            if (position.Z >= WALLS)
            {
                new_position.Z = WALLS;
                new_velocity.Z *= -1 * restitution;
                external_accel.Z = 0;
            }
            else if (position.Z <= -WALLS)
            {
                new_position.Z = -WALLS;
                new_velocity.Z *= -1 * restitution;
                external_accel.Z = 0;
            }

            if (velocity.Length() <= 0.000000001f)
                velocity = Vector3.Zero;

            // Keep moving
            new_velocity += Acceleration * time;
            new_position += new_velocity * time + 1 / 2 * Acceleration * time * time;

            body.UpdateOrientation(ElapsedTime);
        }

        internal void ApplyVelocity(Vector3 vel)
        {
            if (locked == false)
                velocity += vel;
        }

        internal void ApplyImpulse(Vector3 impulse)
        {
            if (locked == false)
                velocity += impulse/mass;
        }

        internal void CollidesWith(PhysicsObject obj, Vector3 contactPoint, TimeSpan ElapsedTime, float time)
        {
            this.locked = false;
            obj.locked = false;

            float new_time = (float)ElapsedTime.TotalSeconds - time;

            if (new_time < 0f)
                throw new Exception("new_time was negative");


            float m1 = this.mass;
            float m2 = obj.mass;
            
            Vector3 v1 = this.velocity;
            Vector3 v2 = obj.velocity;
            
            v1.Normalize();
            v2.Normalize();

            if (this.velocity == Vector3.Zero || obj.velocity == Vector3.Zero || v1 == v2 || v1 == -v2)
            {
                //Linear momentum
                v1 = this.velocity;
                v2 = obj.velocity;

                float e = this.restitution * obj.restitution;

                this.new_velocity = (m2 * e * (v2 - v1) + m1 * v1) / (m1 + m2);
                obj.new_velocity = e * (v1 - v2) + v1;
            }
            else
            {
                //3D momentum
                Vector3 n1 = v1 + v2;
                Vector3 n2 = Vector3.Cross(v1, v2);
                Vector3 tan = Vector3.Cross(n1, n2);

                Vector3 v1_tan = tan * Vector3.Dot(this.velocity, tan);
                Vector3 v2_tan = tan * Vector3.Dot(obj.velocity, tan);

                Vector3 v1_norm = this.velocity - v1_tan;
                Vector3 v2_norm = obj.velocity - v2_tan;

                float e = this.restitution * obj.restitution;

                v1_norm = (m2 * e * (v2_norm - v1_norm) + m1 * v1_norm) / (m1 + m2);
                v2_norm = e * (v1_norm - v2_norm) + v1_norm;

                this.new_velocity = v1_tan + v1_norm;
                obj.new_velocity = v2_tan + v2_norm;
            }

            if (this.velocity == Vector3.Zero && obj.velocity == Vector3.Zero)
            {
                this.locked = true;
                obj.locked = true;
            }

            this.new_position = this.position + (this.velocity * time + 1 / 2 * this.Acceleration * time * time) + (this.new_velocity * new_time + 1 / 2 * this.Acceleration * new_time * new_time);
            obj.new_position = obj.position + (obj.velocity * time + 1 / 2 * obj.Acceleration * time * time) + (obj.new_velocity * new_time + 1 / 2 * obj.Acceleration * new_time * new_time);

            //this.new_position += new_velocity * time + 1 / 2 * Acceleration * time * time;
            //obj.new_position += obj.new_velocity * time + 1 / 2 * obj.Acceleration * time * time;

            this.body.ApplyRotationalVelocity(Vector3.Cross(contactPoint - this.position, this.position) * (this.new_velocity + obj.new_velocity));
            obj.body.ApplyRotationalVelocity(Vector3.Cross(contactPoint - obj.position, obj.position) * -(this.new_velocity + obj.new_velocity));
        }

        private void CollidesWithPlane(Vector3 normal)
        {
            float m1 = this.mass;

            Vector3 v1 = this.velocity;
            Vector3 v2 = Vector3.Zero;

            float e = this.restitution;

            this.velocity = -1*(v2 - (e * (v1 - v2)));
        }

        internal void Reset()
        {
            external_accel = Vector3.Zero;
            velocity = Vector3.Zero;
            position = initialPos;
            new_position = initialPos;
            body.Reset();
        }

        internal void ApplyRandomForce()
        {
            //Random num = new Random();

            //Vector3 force = Vector3.Zero;

            //force.X = (float)(2 * (max_vel - num.Next()) % max_vel);
            //force.Y = (float)(num.Next() % max_vel);
            //force.Z = (float)(2 * (max_vel - num.Next()) % max_vel);

            //ApplyForce(force);
            throw new NotImplementedException();
        }

        internal void TestFunction()
        {
        }
    }
}
