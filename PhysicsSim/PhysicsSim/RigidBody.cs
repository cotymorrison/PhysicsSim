using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace PhysicsSim
{
    public class RigidBody
    {
        //Angular Rotation:
        private float momentOfInertia = 0.0f;
        private Quaternion base_orientation = Quaternion.Identity;
        private Quaternion orientation = Quaternion.Identity;
        public Quaternion Orientation { get { return orientation; } }
        //private Vector3 angularVelocity = new Vector3((float)Math.PI * 4, (float)Math.PI / 4, (float)Math.PI / 4);
        private Vector3 angularVelocity = Vector3.Zero; 
        private Vector3 angularAcceleration = Vector3.Zero;

        internal Vector3 RotationalEnergy { get { return momentOfInertia * angularVelocity * angularVelocity / 2; } }

        internal RigidBody(Quaternion orientation)
        {
            this.base_orientation = orientation;
            this.orientation = orientation;
        }

        internal void UpdateOrientation(TimeSpan ElapsedTime)
        {
            Quaternion temp = new Quaternion(angularVelocity * (float)ElapsedTime.TotalSeconds, 0);

            orientation += temp * orientation;

            orientation.Normalize();
        }

        internal void ApplyRotationalVelocity(Vector3 new_velocity)
        {
            //angularVelocity += new_velocity;
        }

        internal void Reset()
        {
            orientation = base_orientation;
            angularVelocity = Vector3.Zero;
        }
    }
}
