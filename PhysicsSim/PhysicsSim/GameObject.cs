using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework;
using PhysicsGame;

namespace PhysicsSim
{
    internal class GameObject
    {
        public bool Locked { get { return physics.Locked; } }
        public void Lock() {physics.Locked = true;}        
        public void Unlock() {physics.Locked = false;}

        private float scale;
        private Model model;
        private PhysicsObject physics;

        private BoundingBox totalBoundingBox;
        public BoundingBox BoundingBox
        {
            get
            {
                Vector3 min = Vector3.Transform(totalBoundingBox.Min, Matrix.CreateScale(scale) * Matrix.CreateFromQuaternion(physics.Orientation) * Matrix.CreateTranslation(physics.NewPosition));
                Vector3 max = Vector3.Transform(totalBoundingBox.Max, Matrix.CreateScale(scale) * Matrix.CreateFromQuaternion(physics.Orientation) * Matrix.CreateTranslation(physics.NewPosition));

                return new BoundingBox(min, max);
            }
        }

        public Vector3 Position { get { return physics.Position; } }
        public Vector3 Velocity { get { return physics.Velocity; } }
        public Vector3 Acceleration { get { return physics.Acceleration; } }

        private BoundingSphere totalBoundingSphere;
        //public BoundingSphere BoundingSphere { get { return totalBoundingSphere; } }


        internal GameObject(Vector3 initialPosition)
        {
            physics = new PhysicsObject(initialPosition, scale, totalBoundingBox.GetCorners());
        }

        internal GameObject(GameObject copy, Vector3 initialPosition)
        {
            this.scale = copy.scale;
            this.model = copy.model;
            this.totalBoundingBox = copy.totalBoundingBox;
            this.totalBoundingSphere = copy.totalBoundingSphere;
            physics = new PhysicsObject(initialPosition, scale, totalBoundingBox.GetCorners());
        }

        /// <summary>
        /// Load the model into the GameObject
        /// </summary>
        /// <param name="m"></param>
        internal void LoadModel(Model m, float scale)
        {
            model = m;
            this.scale = scale;

            BoundingSphere totalBoundingSphere = m.Meshes[0].BoundingSphere;

            foreach (ModelMesh mesh in model.Meshes)
            {
                BoundingSphere newSphere = mesh.BoundingSphere;
                BoundingSphere.CreateMerged(ref totalBoundingSphere, ref newSphere, out totalBoundingSphere);
            }

            totalBoundingBox = GetBoundingBoxFromModel(m);
            totalBoundingSphere = BoundingSphere.CreateFromBoundingBox(totalBoundingBox);
        }

        internal void CopyModel(GameObject g)
        {
            model = g.model;
            scale = g.scale;
            totalBoundingBox = g.totalBoundingBox;
            totalBoundingSphere = g.totalBoundingSphere;
        }

        /// <summary>
        /// Create a bounding box from a specified model
        /// </summary>
        /// <param name="model"></param>
        /// <returns></returns>
        private BoundingBox GetBoundingBoxFromModel(Model model)
        {
            BoundingBox boundingBox = new BoundingBox();

            foreach (ModelMesh mesh in model.Meshes)
            {
                foreach (ModelMeshPart part in mesh.MeshParts)
                {
                    VertexPositionNormalTexture[] vertices =
                      new VertexPositionNormalTexture[part.VertexBuffer.VertexCount];

                    part.VertexBuffer.GetData<VertexPositionNormalTexture>(vertices);

                    Vector3[] vertexs = new Vector3[vertices.Length];

                    for (int index = 0; index < vertexs.Length; index++)
                    {
                        vertexs[index] = vertices[index].Position;
                    }
                    boundingBox = BoundingBox.CreateMerged(boundingBox,
                      BoundingBox.CreateFromPoints(vertexs));
                }
            }

            return boundingBox;
        }

        /// <summary>
        /// Updates the GameObject's position
        /// </summary>
        /// <param name="gameTime"></param>

        internal void Update(GameTime gameTime)
        {
            physics.Update(gameTime.ElapsedGameTime);
        }

        /// <summary>
        /// Test function
        /// </summary>
        internal void ApplyRandomForce()
        {
            physics.ApplyRandomForce();
        }

        internal void ApplyVelocity(Vector3 vel)
        {
            physics.ApplyVelocity(vel);
        }

        /// <summary>
        /// Renders the 3D model.
        /// </summary>
        internal void DrawCrate(Camera camera)
        {
            Matrix[] transforms = new Matrix[model.Bones.Count];
            model.CopyAbsoluteBoneTransformsTo(transforms);

            Matrix world = Matrix.CreateScale(scale) * Matrix.CreateFromQuaternion(physics.Orientation) * Matrix.CreateTranslation(physics.Position);

            foreach (ModelMesh mesh in model.Meshes)
            {
                foreach (BasicEffect effects in mesh.Effects)
                {
                    effects.EnableDefaultLighting();

                    effects.View = camera.ViewMatrix;
                    effects.Projection = camera.ProjectionMatrix;
                    effects.World = transforms[mesh.ParentBone.Index] * world;
                }
                mesh.Draw();
            }

            DrawBoundingBox(totalBoundingBox, world);
        }

        internal void Reset()
        {
            physics.Reset();
        }

        internal void CollidesWith(GameObject box_b, GameTime gameTime)
        {
            box_b.Unlock();

            Vector3 contactPoint;
            float time;

            GetContactPoint(this.BoundingBox, this.physics.Velocity, box_b.BoundingBox, box_b.physics.Velocity, out contactPoint, out time);

            physics.CollidesWith(box_b.physics, contactPoint, gameTime.ElapsedGameTime, time);  // So what if that "out" time was actually the difference between 
                                                                                                // the overall game time and when the collision happend...  just a 
                                                                                                // thought, but maybe it would work out correctly.  So collision time 
                                                                                                // would = (Total game time - "out" Time)
        }

        internal void GetContactPoint(BoundingBox a, Vector3 vel_a, BoundingBox b, Vector3 vel_b, out Vector3 contactPoint, out float Time)
        {
            Vector3[] aCorners = new Vector3[8];
            Vector3[] bCorners = new Vector3[8];

            Plane[] aPlanes = new Plane[6];
            Plane[] bPlanes = new Plane[6];

            //Box a
            aCorners[0] = a.Max;
            aCorners[1] = new Vector3(a.Min.X, a.Max.Y, a.Max.Z);
            aCorners[2] = new Vector3(a.Max.X, a.Max.Y, a.Min.Z);
            aCorners[3] = new Vector3(a.Max.X, a.Min.Y, a.Max.Z);

            aCorners[4] = a.Min;
            aCorners[5] = new Vector3(a.Max.X, a.Min.Y, a.Min.Z);
            aCorners[6] = new Vector3(a.Min.X, a.Max.Y, a.Min.Z);
            aCorners[7] = new Vector3(a.Min.X, a.Min.Y, a.Max.Z);

            aPlanes[0] = new Plane(aCorners[0], aCorners[1], aCorners[2]);
            aPlanes[1] = new Plane(aCorners[0], aCorners[2], aCorners[3]);
            aPlanes[2] = new Plane(aCorners[0], aCorners[3], aCorners[1]);

            aPlanes[3] = new Plane(aCorners[4], aCorners[5], aCorners[6]);
            aPlanes[4] = new Plane(aCorners[4], aCorners[6], aCorners[7]);
            aPlanes[5] = new Plane(aCorners[4], aCorners[7], aCorners[5]);

            // Box b
            bCorners[0] = b.Max;
            bCorners[1] = new Vector3(b.Min.X, b.Max.Y, b.Max.Z);
            bCorners[2] = new Vector3(b.Max.X, b.Max.Y, b.Min.Z);
            bCorners[3] = new Vector3(b.Max.X, b.Min.Y, b.Max.Z);

            bCorners[4] = b.Min;
            bCorners[5] = new Vector3(b.Max.X, b.Min.Y, b.Min.Z);
            bCorners[6] = new Vector3(b.Min.X, b.Max.Y, b.Min.Z);
            bCorners[7] = new Vector3(b.Min.X, b.Min.Y, b.Max.Z);

            bPlanes[0] = new Plane(bCorners[0], bCorners[1], bCorners[2]);
            bPlanes[1] = new Plane(bCorners[0], bCorners[2], bCorners[3]);
            bPlanes[2] = new Plane(bCorners[0], bCorners[3], bCorners[1]);

            bPlanes[3] = new Plane(bCorners[4], bCorners[5], bCorners[6]);
            bPlanes[4] = new Plane(bCorners[4], bCorners[6], bCorners[7]);
            bPlanes[5] = new Plane(bCorners[4], bCorners[7], bCorners[5]);


            Ray[] aRays = new Ray[aCorners.Length];
            Ray[] bRays = new Ray[bCorners.Length];

            Vector3 a_rel_b = vel_a + vel_b;
            Vector3 b_rel_a = -a_rel_b;

            Vector3 dir_a = a_rel_b;
            Vector3 dir_b = -a_rel_b;

            dir_a.Normalize();
            dir_b.Normalize();

            for (int i = 0; i < aRays.Length; i++)
                aRays[i] = new Ray(aCorners[i], dir_a);

            for (int i = 0; i < bRays.Length; i++)
                bRays[i] = new Ray(bCorners[i], dir_b);

            
            float?[] aDistances = new float?[aRays.Length * aPlanes.Length];
            float?[] bDistances = new float?[bRays.Length * bPlanes.Length];

            //for (int i = 0; i < aRays.Length; i++)
            //    for (int j = 0; j < aPlanes.Length; j++)
            //        aDistances[i * aPlanes.Length + j] = aRays[i].Intersects(bPlanes[j]);

            //for (int i = 0; i < bRays.Length; i++)
            //    for (int j = 0; j < bPlanes.Length; j++)
            //        bDistances[i * bPlanes.Length + j] = bRays[i].Intersects(aPlanes[j]);

            //Subsitution Code//
            for (int i = 0; i < aRays.Length; i++)
                aDistances[i] = aRays[i].Intersects(b);

            for (int i = 0; i < bRays.Length; i++)
                bDistances[i] = bRays[i].Intersects(a);
            //End//


            int aIndex = -1;
            int bIndex = -1;
            float aTime = float.PositiveInfinity;
            float bTime = float.PositiveInfinity;

            for (int i = 0; i < aDistances.Length; i++)
                if (aDistances[i].HasValue && Math.Abs(aDistances[i].Value) < aTime)
                {
                    aIndex = i;
                    aTime = aDistances[i].Value;
                }

            for (int i = 0; i < bDistances.Length; i++)
                if (bDistances[i].HasValue && Math.Abs(bDistances[i].Value) < bTime)  
                {
                    bIndex = i;
                    bTime = bDistances[i].Value;
                }


            // Find the lowest time for any point of contact
            if (aIndex != -1 && bIndex != -1)
            {
                if (aDistances[aIndex] < bDistances[bIndex])
                {
                    Time = aTime / a_rel_b.Length();
                    contactPoint = aCorners[aIndex];
                }
                else
                {
                    Time = bTime / a_rel_b.Length();
                    contactPoint = bCorners[bIndex];
                }

            }
            else if (aIndex != -1)
            {
                Time = aTime / a_rel_b.Length();
                contactPoint = aCorners[aIndex];
            }
            else if (bIndex != -1)
            {
                Time = bTime / a_rel_b.Length();
                contactPoint = bCorners[bIndex];
            }
            else
            {
                // wtf?
                //throw new Exception("they aren't colliding?");

                Time = 0;
                contactPoint = new Vector3(0, 0, 0);
            }


        }

        internal void DrawBoundingBox(BoundingBox box, Matrix world)
        {
            Vector3[] corners = box.GetCorners();
            VertexPositionNormalTexture[] verts = new VertexPositionNormalTexture[8];

            foreach (Vector3 corner in corners)
                Vector3.Transform(corner, world);

            for (int i = 0; i < 8; i++)
            {
                verts[i].Position = corners[i];
                verts[i].Normal = new Vector3(0, 0, -1);
                verts[i].TextureCoordinate = new Vector2(0, 0);
            }
            int[] indices = new int[]  
            {  
                0, 1,  
                1, 2,  
                2, 3,  
                3, 0,  
                0, 4,  
                1, 5,  
                2, 6,  
                3, 7,  
                4, 5,  
                5, 6,  
                6, 7,  
                7, 4,  
            };

            Demo.graphics.GraphicsDevice.DrawUserIndexedPrimitives(
                PrimitiveType.LineList,
                verts,
                0,
                8,
                indices,
                0,
                indices.Length / 2);

        }
    }
}
