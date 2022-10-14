//-----------------------------------------------------------------------------
// Copyright (c) 2007-2011 dhpoware. All Rights Reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
//-----------------------------------------------------------------------------

using System;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace PhysicsSim
{
    /// <summary>
    /// Custom vertex structure used for normal mapping.
    /// </summary>
    public struct NormalMappedVertex : IVertexType
    {
        public static readonly VertexDeclaration VertexDeclaration = new VertexDeclaration
        (
            new VertexElement(0, VertexElementFormat.Vector3, VertexElementUsage.Position, 0),
            new VertexElement(sizeof(float) * 3, VertexElementFormat.Vector2, VertexElementUsage.TextureCoordinate, 0),
            new VertexElement(sizeof(float) * 5, VertexElementFormat.Vector3, VertexElementUsage.Normal, 0),
            new VertexElement(sizeof(float) * 8, VertexElementFormat.Vector4, VertexElementUsage.Tangent, 0)
        );

        public Vector3 Position;
        public Vector2 TexCoord;
        public Vector3 Normal;
        public Vector4 Tangent;

        public NormalMappedVertex(Vector3 position, Vector2 texCoord, Vector3 normal, Vector4 tangent)
        {
            Position = position;
            TexCoord = texCoord;
            Normal = normal;
            Tangent = tangent;
        }

        VertexDeclaration IVertexType.VertexDeclaration
        {
            get { return VertexDeclaration; }
        }

        /// <summary>
        /// Given the 3 vertices (position and texture coordinate) and the
        /// face normal of a triangle calculate and return the triangle's
        /// tangent vector. This method is designed to work with XNA's default
        /// right handed coordinate system and clockwise triangle winding order.
        /// Undefined behavior will result if any other coordinate system
        /// and/or winding order is used. The handedness of the local tangent
        /// space coordinate system is stored in the tangent's w component.
        /// </summary>
        /// <param name="pos1">Triangle vertex 1 position</param>
        /// <param name="pos2">Triangle vertex 2 position</param>
        /// <param name="pos3">Triangle vertex 3 position</param>
        /// <param name="texCoord1">Triangle vertex 1 texture coordinate</param>
        /// <param name="texCoord2">Triangle vertex 2 texture coordinate</param>
        /// <param name="texCoord3">Triangle vertex 3 texture coordinate</param>
        /// <param name="normal">Triangle face normal</param>
        /// <param name="tangent">Calculated tangent vector</param>
        public static void CalcTangent(ref Vector3 pos1,
                                       ref Vector3 pos2,
                                       ref Vector3 pos3,
                                       ref Vector2 texCoord1,
                                       ref Vector2 texCoord2,
                                       ref Vector2 texCoord3,
                                       ref Vector3 normal,
                                       out Vector4 tangent)
        {
            // Create 2 vectors in object space.
            // edge1 is the vector from vertex positions pos1 to pos3.
            // edge2 is the vector from vertex positions pos1 to pos2.
            Vector3 edge1 = pos3 - pos1;
            Vector3 edge2 = pos2 - pos1;

            edge1.Normalize();
            edge2.Normalize();

            // Create 2 vectors in tangent (texture) space that point in the
            // same direction as edge1 and edge2 (in object space).
            // texEdge1 is the vector from texture coordinates texCoord1 to texCoord3.
            // texEdge2 is the vector from texture coordinates texCoord1 to texCoord2.
            Vector2 texEdge1 = texCoord3 - texCoord1;
            Vector2 texEdge2 = texCoord2 - texCoord1;

            texEdge1.Normalize();
            texEdge2.Normalize();

            // These 2 sets of vectors form the following system of equations:
            //
            //  edge1 = (texEdge1.x * tangent) + (texEdge1.y * bitangent)
            //  edge2 = (texEdge2.x * tangent) + (texEdge2.y * bitangent)
            //
            // Using matrix notation this system looks like:
            //
            //  [ edge1 ]     [ texEdge1.x  texEdge1.y ]  [ tangent   ]
            //  [       ]  =  [                        ]  [           ]
            //  [ edge2 ]     [ texEdge2.x  texEdge2.y ]  [ bitangent ]
            //
            // The solution is:
            //
            //  [ tangent   ]        1     [ texEdge2.y  -texEdge1.y ]  [ edge1 ]
            //  [           ]  =  -------  [                         ]  [       ]
            //  [ bitangent ]      det A   [-texEdge2.x   texEdge1.x ]  [ edge2 ]
            //
            //  where:
            //        [ texEdge1.x  texEdge1.y ]
            //    A = [                        ]
            //        [ texEdge2.x  texEdge2.y ]
            //
            //    det A = (texEdge1.x * texEdge2.y) - (texEdge1.y * texEdge2.x)
            //
            // From this solution the tangent space basis vectors are:
            //
            //    tangent = (1 / det A) * ( texEdge2.y * edge1 - texEdge1.y * edge2)
            //  bitangent = (1 / det A) * (-texEdge2.x * edge1 + texEdge1.x * edge2)
            //     normal = cross(tangent, bitangent)

            Vector3 t;
            Vector3 b;
            float det = (texEdge1.X * texEdge2.Y) - (texEdge1.Y * texEdge2.X);

            if ((float)Math.Abs(det) < 1e-6f)    // almost equal to zero
            {
                t = Vector3.UnitX;
                b = Vector3.UnitY;
            }
            else
            {
                det = 1.0f / det;

                t.X = (texEdge2.Y * edge1.X - texEdge1.Y * edge2.X) * det;
                t.Y = (texEdge2.Y * edge1.Y - texEdge1.Y * edge2.Y) * det;
                t.Z = (texEdge2.Y * edge1.Z - texEdge1.Y * edge2.Z) * det;

                b.X = (-texEdge2.X * edge1.X + texEdge1.X * edge2.X) * det;
                b.Y = (-texEdge2.X * edge1.Y + texEdge1.X * edge2.Y) * det;
                b.Z = (-texEdge2.X * edge1.Z + texEdge1.X * edge2.Z) * det;

                t.Normalize();
                b.Normalize();
            }

            // Calculate the handedness of the local tangent space.
            // The bitangent vector is the cross product between the triangle face
            // normal vector and the calculated tangent vector. The resulting bitangent
            // vector should be the same as the bitangent vector calculated from the
            // set of linear equations above. If they point in different directions
            // then we need to invert the cross product calculated bitangent vector. We
            // store this scalar multiplier in the tangent vector's 'w' component so
            // that the correct bitangent vector can be generated in the normal mapping
            // shader's vertex shader.

            Vector3 bitangent = Vector3.Cross(normal, t);
            float handedness = (Vector3.Dot(bitangent, b) < 0.0f) ? -1.0f : 1.0f;

            tangent.X = t.X;
            tangent.Y = t.Y;
            tangent.Z = t.Z;
            tangent.W = handedness;
        }
    }


    /// <summary>
    /// The NormalMappedRoom class is used to procedurally generate a room.
    /// A room is a cube with all face normals pointed inwards.
    /// This class generates geometry using the NormalMappedVertex structure.
    /// </summary>
    public class NormalMappedRoom
    {
        private VertexBuffer vertexBuffer;
        private NormalMappedVertex[] vertices;

        private int floorIndex;
        private int ceilingIndex;
        private int wallsIndex;

        public NormalMappedRoom(GraphicsDevice graphicsDevice,
                                float size,
                                float height,
                                float floorTileFactor,
                                float ceilingTileFactor,
                                float wallTileFactorX,
                                float wallTileFactorY)
        {
            GenerateRoom(graphicsDevice, size, height, floorTileFactor, ceilingTileFactor, wallTileFactorX, wallTileFactorY);
        }

        public void Draw(GraphicsDevice graphicsDevice,
                         Effect effect,
                         string colorMapParamName,
                         string normalMapParamName,
                         Texture2D wallColorMap,
                         Texture2D wallNormalMap,
                         Texture2D floorColorMap,
                         Texture2D floorNormalMap,
                         Texture2D ceilingColorMap,
                         Texture2D ceilingNormalMap)
        {
            // Draw the scene geometry.
            graphicsDevice.SetVertexBuffer(vertexBuffer);

            // Draw the walls.

            effect.Parameters[colorMapParamName].SetValue(wallColorMap);
            effect.Parameters[normalMapParamName].SetValue(wallNormalMap);

            foreach (EffectPass pass in effect.CurrentTechnique.Passes)
            {
                pass.Apply();
                graphicsDevice.DrawPrimitives(PrimitiveType.TriangleList, wallsIndex, 8);
            }

            // Draw the ceiling.

            effect.Parameters[colorMapParamName].SetValue(ceilingColorMap);
            effect.Parameters[normalMapParamName].SetValue(ceilingNormalMap);

            foreach (EffectPass pass in effect.CurrentTechnique.Passes)
            {
                pass.Apply();
                graphicsDevice.DrawPrimitives(PrimitiveType.TriangleList, ceilingIndex, 2);
            }

            // Draw the floor.

            effect.Parameters[colorMapParamName].SetValue(floorColorMap);
            effect.Parameters[normalMapParamName].SetValue(floorNormalMap);

            foreach (EffectPass pass in effect.CurrentTechnique.Passes)
            {
                pass.Apply();
                graphicsDevice.DrawPrimitives(PrimitiveType.TriangleList, floorIndex, 2);
            }
        }

        public void Draw(GraphicsDevice graphicsDevice,
                         Effect effect,
                         string colorMapParamName,
                         string normalMapParamName,
                         string heightMapParamName,
                         Texture2D wallColorMap,
                         Texture2D wallNormalMap,
                         Texture2D wallHeightMap,
                         Texture2D floorColorMap,
                         Texture2D floorNormalMap,
                         Texture2D floorHeightMap,
                         Texture2D ceilingColorMap,
                         Texture2D ceilingNormalMap,
                         Texture2D ceilingHeightMap)
        {
            // Draw the scene geometry. 

            graphicsDevice.SetVertexBuffer(vertexBuffer);

            // Draw the walls.               
            effect.Parameters[colorMapParamName].SetValue(wallColorMap);
            effect.Parameters[normalMapParamName].SetValue(wallNormalMap);
            effect.Parameters[heightMapParamName].SetValue(wallHeightMap);

            foreach (EffectPass pass in effect.CurrentTechnique.Passes)
            {
                pass.Apply();
                graphicsDevice.DrawPrimitives(PrimitiveType.TriangleList, wallsIndex, 8);
            }

            // Draw the ceiling.

            //effect.Parameters[colorMapParamName].SetValue(ceilingColorMap);
            //effect.Parameters[normalMapParamName].SetValue(ceilingNormalMap);
            //effect.Parameters[heightMapParamName].SetValue(ceilingHeightMap);

            //foreach (EffectPass pass in effect.CurrentTechnique.Passes)
            //{
            //    pass.Apply();
            //    graphicsDevice.DrawPrimitives(PrimitiveType.TriangleList, ceilingIndex, 2);
            //}

            // Draw the floor.

            effect.Parameters[colorMapParamName].SetValue(floorColorMap);
            effect.Parameters[normalMapParamName].SetValue(floorNormalMap);
            effect.Parameters[heightMapParamName].SetValue(floorHeightMap);

            foreach (EffectPass pass in effect.CurrentTechnique.Passes)
            {
                pass.Apply();
                graphicsDevice.DrawPrimitives(PrimitiveType.TriangleList, floorIndex, 2);
            }
        }

        private void GenerateRoomGeometry(float floorSize,
                                          float wallHeight,
                                          float floorTileFactor,
                                          float ceilingTileFactor,
                                          float wallTileFactorX,
                                          float wallTileFactorY)
        {
            /* 5--------6
             * |\      /|
             * | 4----7 |
             * | |    | |
             * | |    | |
             * | 0----3 |
             * |/      \|
             * 1--------2
             *
             * +z wall: 6512 tri1: 651 tri2: 126
             * -z wall: 4730 tri1: 473 tri2: 304
             * +x wall: 7623 tri1: 762 tri2: 237
             * -x wall: 5401 tri1: 540 tri2: 015
             * 
             * +y ceiling: 5674 tri1: 567 tri2: 745
             * -y floor:   0321 tri1: 032 tri2: 210
             */

            int offset = 0;
            float halfSize = floorSize * 0.5f;

            Vector3[] corners =
            {
                new Vector3(-halfSize, 0.0f, -halfSize),            // 0
                new Vector3(-halfSize, 0.0f,  halfSize),            // 1
                new Vector3( halfSize, 0.0f,  halfSize),            // 2
                new Vector3( halfSize, 0.0f, -halfSize),            // 3

                new Vector3(-halfSize, wallHeight, -halfSize),      // 4
                new Vector3(-halfSize, wallHeight,  halfSize),      // 5
                new Vector3( halfSize, wallHeight,  halfSize),      // 6
                new Vector3( halfSize, wallHeight, -halfSize),      // 7
            };

            Vector2[] wallTexCoords =
            {
                new Vector2(0.0f, 0.0f),                            // top left corner
                new Vector2(wallTileFactorX, 0.0f),                 // top right corner
                new Vector2(wallTileFactorX, wallTileFactorY),      // bottom right corner    
                new Vector2(0.0f, wallTileFactorY)                  // bottom left corner
            };

            Vector2[] floorTexCoords =
            {
                new Vector2(0.0f, 0.0f),                            // top left corner
                new Vector2(floorTileFactor, 0.0f),                 // top right corner
                new Vector2(floorTileFactor, floorTileFactor),      // bottom right corner    
                new Vector2(0.0f, floorTileFactor)                  // bottom left corner
            };

            Vector2[] ceilingTexCoords =
            {
                new Vector2(0.0f, 0.0f),                            // top left corner
                new Vector2(ceilingTileFactor, 0.0f),               // top right corner
                new Vector2(ceilingTileFactor, ceilingTileFactor),  // bottom right corner
                new Vector2(0.0f, ceilingTileFactor)                // bottom left corner
            };

            wallsIndex = offset;

            // -z wall: 4730 tri1: 473 tri2: 304
            vertices[offset++] = new NormalMappedVertex(corners[4], wallTexCoords[0], Vector3.Backward, Vector4.Zero);
            vertices[offset++] = new NormalMappedVertex(corners[7], wallTexCoords[1], Vector3.Backward, Vector4.Zero);
            vertices[offset++] = new NormalMappedVertex(corners[3], wallTexCoords[2], Vector3.Backward, Vector4.Zero);
            vertices[offset++] = new NormalMappedVertex(corners[3], wallTexCoords[2], Vector3.Backward, Vector4.Zero);
            vertices[offset++] = new NormalMappedVertex(corners[0], wallTexCoords[3], Vector3.Backward, Vector4.Zero);
            vertices[offset++] = new NormalMappedVertex(corners[4], wallTexCoords[0], Vector3.Backward, Vector4.Zero);

            // +z wall: 6512 tri1: 651 tri2: 126
            vertices[offset++] = new NormalMappedVertex(corners[6], wallTexCoords[0], Vector3.Forward, Vector4.Zero);
            vertices[offset++] = new NormalMappedVertex(corners[5], wallTexCoords[1], Vector3.Forward, Vector4.Zero);
            vertices[offset++] = new NormalMappedVertex(corners[1], wallTexCoords[2], Vector3.Forward, Vector4.Zero);
            vertices[offset++] = new NormalMappedVertex(corners[1], wallTexCoords[2], Vector3.Forward, Vector4.Zero);
            vertices[offset++] = new NormalMappedVertex(corners[2], wallTexCoords[3], Vector3.Forward, Vector4.Zero);
            vertices[offset++] = new NormalMappedVertex(corners[6], wallTexCoords[0], Vector3.Forward, Vector4.Zero);

            // -x wall: 5401 tri1: 540 tri2: 015
            vertices[offset++] = new NormalMappedVertex(corners[5], wallTexCoords[0], Vector3.Right, Vector4.Zero);
            vertices[offset++] = new NormalMappedVertex(corners[4], wallTexCoords[1], Vector3.Right, Vector4.Zero);
            vertices[offset++] = new NormalMappedVertex(corners[0], wallTexCoords[2], Vector3.Right, Vector4.Zero);
            vertices[offset++] = new NormalMappedVertex(corners[0], wallTexCoords[2], Vector3.Right, Vector4.Zero);
            vertices[offset++] = new NormalMappedVertex(corners[1], wallTexCoords[3], Vector3.Right, Vector4.Zero);
            vertices[offset++] = new NormalMappedVertex(corners[5], wallTexCoords[0], Vector3.Right, Vector4.Zero);

            // +x wall: 7623 tri1: 762 tri2: 237
            vertices[offset++] = new NormalMappedVertex(corners[7], wallTexCoords[0], Vector3.Left, Vector4.Zero);
            vertices[offset++] = new NormalMappedVertex(corners[6], wallTexCoords[1], Vector3.Left, Vector4.Zero);
            vertices[offset++] = new NormalMappedVertex(corners[2], wallTexCoords[2], Vector3.Left, Vector4.Zero);
            vertices[offset++] = new NormalMappedVertex(corners[2], wallTexCoords[2], Vector3.Left, Vector4.Zero);
            vertices[offset++] = new NormalMappedVertex(corners[3], wallTexCoords[3], Vector3.Left, Vector4.Zero);
            vertices[offset++] = new NormalMappedVertex(corners[7], wallTexCoords[0], Vector3.Left, Vector4.Zero);

            // +y ceiling: 5674 tri1: 567 tri2: 745
            ceilingIndex = offset;
            vertices[offset++] = new NormalMappedVertex(corners[5], ceilingTexCoords[0], Vector3.Down, Vector4.Zero);
            vertices[offset++] = new NormalMappedVertex(corners[6], ceilingTexCoords[1], Vector3.Down, Vector4.Zero);
            vertices[offset++] = new NormalMappedVertex(corners[7], ceilingTexCoords[2], Vector3.Down, Vector4.Zero);
            vertices[offset++] = new NormalMappedVertex(corners[7], ceilingTexCoords[2], Vector3.Down, Vector4.Zero);
            vertices[offset++] = new NormalMappedVertex(corners[4], ceilingTexCoords[3], Vector3.Down, Vector4.Zero);
            vertices[offset++] = new NormalMappedVertex(corners[5], ceilingTexCoords[0], Vector3.Down, Vector4.Zero);

            // -y floor: 0321 tri1: 032 tri2: 210
            floorIndex = offset;
            vertices[offset++] = new NormalMappedVertex(corners[0], floorTexCoords[0], Vector3.Up, Vector4.Zero);
            vertices[offset++] = new NormalMappedVertex(corners[3], floorTexCoords[1], Vector3.Up, Vector4.Zero);
            vertices[offset++] = new NormalMappedVertex(corners[2], floorTexCoords[2], Vector3.Up, Vector4.Zero);
            vertices[offset++] = new NormalMappedVertex(corners[2], floorTexCoords[2], Vector3.Up, Vector4.Zero);
            vertices[offset++] = new NormalMappedVertex(corners[1], floorTexCoords[3], Vector3.Up, Vector4.Zero);
            vertices[offset++] = new NormalMappedVertex(corners[0], floorTexCoords[0], Vector3.Up, Vector4.Zero);
        }

        private void GenerateRoom(GraphicsDevice graphicsDevice,
                                  float floorSize,
                                  float wallHeight,
                                  float floorTileFactor,
                                  float ceilingTileFactor,
                                  float wallTileFactorX,
                                  float wallTileFactorY)
        {
            // The room is a cube with all surface normal pointing inwards.
            // We need a total of 36 vertices in the vertex buffer.
            // 6 faces to the room.
            // Each face is made of 2 triangles requiring 6 vertices.

            vertices = new NormalMappedVertex[36];

            GenerateRoomGeometry(floorSize, wallHeight, floorTileFactor,
                ceilingTileFactor, wallTileFactorX, wallTileFactorY);

            // Calculate the tangent vectors for each triangle in the room.

            Vector4 tangent;

            for (int i = 0; i < vertices.Length; i += 3)
            {
                NormalMappedVertex.CalcTangent(
                    ref vertices[i].Position,
                    ref vertices[i + 1].Position,
                    ref vertices[i + 2].Position,
                    ref vertices[i].TexCoord,
                    ref vertices[i + 1].TexCoord,
                    ref vertices[i + 2].TexCoord,
                    ref vertices[i].Normal,
                    out tangent);

                vertices[i].Tangent = tangent;
                vertices[i + 1].Tangent = tangent;
                vertices[i + 2].Tangent = tangent;
            }

            // Create and fill the vertex buffer with the room geometry.

            vertexBuffer = new VertexBuffer(graphicsDevice, typeof(NormalMappedVertex), vertices.Length, BufferUsage.WriteOnly);
            vertexBuffer.SetData(vertices);
        }
    }
}