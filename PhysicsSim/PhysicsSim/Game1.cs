using System;
using System.Text;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Audio;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.GamerServices;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using Microsoft.Xna.Framework.Media;

using PhysicsSim;

namespace PhysicsGame
{
    /// <summary>
    /// This Windows XNA application implements a first person shooter style
    /// camera. The walls, floor, and ceiling are parallax normal mapped in
    /// tangent space. The weapon model is shaded using the XNA per pixel
    /// BasicEffect class.
    /// </summary>
    public class Demo : Microsoft.Xna.Framework.Game
    {
        const int NUM_BOXES = 5;

        /// <summary>
        /// A light. This light structure is the same as the one defined in
        /// the parallax_normal_mapping.fx file. The only difference is the
        /// LightType enum.
        /// </summary>
        private struct Light
        {
            public enum LightType
            {
                DirectionalLight,
                PointLight,
                SpotLight
            }

            public LightType Type;
            public Vector3 Direction;
            public Vector3 Position;
            public Color Ambient;
            public Color Diffuse;
            public Color Specular;
            public float SpotInnerConeRadians;
            public float SpotOuterConeRadians;
            public float Radius;
        }

        /// <summary>
        /// A material. This material structure is the same as the one defined
        /// in the parallax_normal_mapping.fx file. We use the Color type here
        /// instead of a four element floating point array.
        /// </summary>
        private struct Material
        {
            public Color Ambient;
            public Color Diffuse;
            public Color Emissive;
            public Color Specular;
            public float Shininess;
        }

        private const float CEILING_TILE_FACTOR = 6.0f;
        private const float FLOOR_PLANE_SIZE = 40.0f;
        private const float FLOOR_TILE_FACTOR = 8.0f;
        private const float FLOOR_CLIP_BOUNDS = FLOOR_PLANE_SIZE * 0.5f - 3.0f;
        private const float WALL_HEIGHT = 5.0f;
        private const float WALL_TILE_FACTOR_X = 12.0f;
        private const float WALL_TILE_FACTOR_Y = 1.5f;

        private const float CAMERA_FOVX = 85.0f;
        private const float CAMERA_ZNEAR = 0.01f;
        private const float CAMERA_ZFAR = FLOOR_PLANE_SIZE * 2.0f;
        private const float CAMERA_PLAYER_EYE_HEIGHT = 1.83f;
        private const float CAMERA_ACCELERATION_X = 40.0f;
        private const float CAMERA_ACCELERATION_Y = 9.8f;
        private const float CAMERA_ACCELERATION_Z = 40.0f;
        private const float CAMERA_VELOCITY_X = 8.0f;
        private const float CAMERA_VELOCITY_Y = 3.0f;
        private const float CAMERA_VELOCITY_Z = 8.0f;
        private const float CAMERA_RUNNING_MULTIPLIER = 2.0f;
        private const float CAMERA_RUNNING_JUMP_MULTIPLIER = 1.5f;
        private const float CAMERA_BOUNDS_PADDING = 0.25f;
        private const float CAMERA_BOUNDS_MIN_X = -FLOOR_PLANE_SIZE / 2.0f + CAMERA_BOUNDS_PADDING;
        private const float CAMERA_BOUNDS_MAX_X = FLOOR_PLANE_SIZE / 2.0f - CAMERA_BOUNDS_PADDING;
        private const float CAMERA_BOUNDS_MIN_Y = 0.0f;
        private const float CAMERA_BOUNDS_MAX_Y = WALL_HEIGHT;
        private const float CAMERA_BOUNDS_MIN_Z = -FLOOR_PLANE_SIZE / 2.0f + CAMERA_BOUNDS_PADDING;
        private const float CAMERA_BOUNDS_MAX_Z = FLOOR_PLANE_SIZE / 2.0f - CAMERA_BOUNDS_PADDING;

        private Texture2D nullTexture;
        private Texture2D brickColorMap;
        private Texture2D brickNormalMap;
        private Texture2D brickHeightMap;
        private Texture2D stoneColorMap;
        private Texture2D stoneNormalMap;
        private Texture2D stoneHeightMap;
        private Texture2D woodColorMap;
        private Texture2D woodNormalMap;
        private Texture2D woodHeightMap;

        public static GraphicsDeviceManager graphics;
        private SpriteBatch spriteBatch;
        private SpriteFont spriteFont;
        private Effect effect;
        private KeyboardState currentKeyboardState;
        private KeyboardState prevKeyboardState;
        private Camera camera;
        private NormalMappedRoom room;

        private Light light;
        private Material material;
        private Color globalAmbient;
        private Vector2 scaleBias;
        private Vector2 fontPos;
        private int windowWidth;
        private int windowHeight;
        private int framesPerSecond;
        private TimeSpan elapsedTime = TimeSpan.Zero;
        private bool enableColorMap;
        private bool enableParallax;
        private bool displayHelp;

        //private Dictionary<int, GameObject> gameObjects = new Dictionary<int, GameObject>();
        private GameObject[] boxes = new GameObject[NUM_BOXES];

        public Demo()
        {
            graphics = new GraphicsDeviceManager(this);
            Content.RootDirectory = "Content";

            camera = new Camera(this);
            Components.Add(camera);

            Window.Title = "XNA 4.0 First Person Camera";
            IsFixedTimeStep = false;
        }

        protected override void Initialize()
        {
            base.Initialize();

            // Setup the window to be a quarter the size of the desktop.
            windowWidth = GraphicsDevice.DisplayMode.Width / 2;
            windowHeight = GraphicsDevice.DisplayMode.Height / 2;

            // Setup frame buffer.
            graphics.SynchronizeWithVerticalRetrace = false;
            graphics.PreferredBackBufferWidth = windowWidth;
            graphics.PreferredBackBufferHeight = windowHeight;
            graphics.PreferMultiSampling = true;
            graphics.ApplyChanges();

            // Initially enable the diffuse color map texture.
            enableColorMap = true;

            // Initially enable parallax mapping.
            enableParallax = true;

            // Initial position for text rendering.
            fontPos = new Vector2(1.0f, 1.0f);

            // Parallax mapping height scale and bias values.
            scaleBias = new Vector2(0.04f, -0.03f);

            // Initialize point lighting for the scene.
            globalAmbient = new Color(new Vector4(0.2f, 0.2f, 0.2f, 1.0f));
            light.Type = Light.LightType.PointLight;
            light.Direction = Vector3.Zero;
            light.Position = new Vector3(0.0f, WALL_HEIGHT - (0.25f * WALL_HEIGHT), 0.0f);
            light.Ambient = Color.White;
            light.Diffuse = Color.White;
            light.Specular = Color.White;
            light.SpotInnerConeRadians = MathHelper.ToRadians(40.0f);
            light.SpotOuterConeRadians = MathHelper.ToRadians(70.0f);
            light.Radius = Math.Max(FLOOR_PLANE_SIZE, WALL_HEIGHT);

            // Initialize material settings. Just a plain lambert material.
            material.Ambient = new Color(new Vector4(0.2f, 0.2f, 0.2f, 1.0f));
            material.Diffuse = new Color(new Vector4(0.8f, 0.8f, 0.8f, 1.0f));
            material.Emissive = Color.Black;
            material.Specular = Color.Black;
            material.Shininess = 0.0f;

            // Create the room.
            room = new NormalMappedRoom(GraphicsDevice,
                    FLOOR_PLANE_SIZE, WALL_HEIGHT, FLOOR_TILE_FACTOR,
                    CEILING_TILE_FACTOR, WALL_TILE_FACTOR_X, WALL_TILE_FACTOR_Y);

            // Setup the camera.
            camera.EyeHeightStanding = CAMERA_PLAYER_EYE_HEIGHT;
            camera.Acceleration = new Vector3(
                CAMERA_ACCELERATION_X,
                CAMERA_ACCELERATION_Y,
                CAMERA_ACCELERATION_Z);
            camera.VelocityWalking = new Vector3(
                CAMERA_VELOCITY_X,
                CAMERA_VELOCITY_Y,
                CAMERA_VELOCITY_Z);
            camera.VelocityRunning = new Vector3(
                camera.VelocityWalking.X * CAMERA_RUNNING_MULTIPLIER,
                camera.VelocityWalking.Y * CAMERA_RUNNING_JUMP_MULTIPLIER,
                camera.VelocityWalking.Z * CAMERA_RUNNING_MULTIPLIER);
            camera.Perspective(
                CAMERA_FOVX,
                (float)windowWidth / (float)windowHeight,
                CAMERA_ZNEAR, CAMERA_ZFAR);

            // Get the initial keyboard state.
            currentKeyboardState = Keyboard.GetState();
        }

        protected override void LoadContent()
        {
            //Load models
            Model woodbox = Content.Load<Model>("Wooden Box");
            //Model woodbox = Content.Load<Model>("Wooden Box");

            Random num = new Random();

            for (int i = 0; i < boxes.Length; i++)
            {
                //boxes[i] = new GameObject(new Vector3(0, (float)num.NextDouble() * WALL_HEIGHT, -10));
                boxes[i] = new GameObject(new Vector3(0, i * 3f + 10, -10 + i*0.1f));
                boxes[i].LoadModel(woodbox, 0.25f);
            }

            //foreach (GameObject box in boxes)
            //    gameObjects.Add(gameObjects.Count, box);

            spriteBatch = new SpriteBatch(GraphicsDevice);
            spriteFont =  Content.Load<SpriteFont>(@"Fonts\DemoFont");

            effect = Content.Load<Effect>(@"Effects\parallax_normal_mapping");
            effect.CurrentTechnique = effect.Techniques["ParallaxNormalMappingPointLighting"];

            brickColorMap = Content.Load<Texture2D>(@"Textures\brick_color_map");
            brickNormalMap = Content.Load<Texture2D>(@"Textures\brick_normal_map");
            brickHeightMap = Content.Load<Texture2D>(@"Textures\brick_height_map");

            stoneColorMap = Content.Load<Texture2D>(@"Textures\stone_color_map");
            stoneNormalMap = Content.Load<Texture2D>(@"Textures\stone_normal_map");
            stoneHeightMap = Content.Load<Texture2D>(@"Textures\stone_height_map");

            woodColorMap = Content.Load<Texture2D>(@"Textures\wood_color_map");
            woodNormalMap = Content.Load<Texture2D>(@"Textures\wood_normal_map");
            woodHeightMap = Content.Load<Texture2D>(@"Textures\wood_height_map");

            // Create an empty white texture. This will be bound to the
            // colorMapTexture shader parameter when the user wants to
            // disable the color map texture. This trick will allow the
            // same shader to be used for when textures are enabled and
            // disabled.

            nullTexture = new Texture2D(GraphicsDevice, 1, 1, false, SurfaceFormat.Color);

            Color[] pixels = { Color.White };

            nullTexture.SetData(pixels);
        }

        protected override void UnloadContent()
        {
            Content.Unload();
        }

        private void ToggleFullScreen()
        {
            int newWidth = 0;
            int newHeight = 0;

            graphics.IsFullScreen = !graphics.IsFullScreen;

            if (graphics.IsFullScreen)
            {
                newWidth = GraphicsDevice.DisplayMode.Width;
                newHeight = GraphicsDevice.DisplayMode.Height;
            }
            else
            {
                newWidth = windowWidth;
                newHeight = windowHeight;
            }

            graphics.PreferredBackBufferWidth = newWidth;
            graphics.PreferredBackBufferHeight = newHeight;
            graphics.PreferMultiSampling = true;
            graphics.ApplyChanges();

            float aspectRatio = (float)newWidth / (float)newHeight;

            camera.Perspective(CAMERA_FOVX, aspectRatio, CAMERA_ZNEAR, CAMERA_ZFAR);
        }

        /// <summary>
        /// Very simple camera collision detection logic to prevent the camera
        /// from moving below the floor and from moving outside the bounds of
        /// the room.
        /// </summary>
        private void PerformCameraCollisionDetection()
        {
            Vector3 newPos = camera.Position;

            if (camera.Position.X > CAMERA_BOUNDS_MAX_X)
                newPos.X = CAMERA_BOUNDS_MAX_X;

            if (camera.Position.X < CAMERA_BOUNDS_MIN_X)
                newPos.X = CAMERA_BOUNDS_MIN_X;

            if (camera.Position.Y > CAMERA_BOUNDS_MAX_Y)
                newPos.Y = CAMERA_BOUNDS_MAX_Y;

            if (camera.Position.Y < CAMERA_BOUNDS_MIN_Y)
                newPos.Y = CAMERA_BOUNDS_MIN_Y;

            if (camera.Position.Z > CAMERA_BOUNDS_MAX_Z)
                newPos.Z = CAMERA_BOUNDS_MAX_Z;

            if (camera.Position.Z < CAMERA_BOUNDS_MIN_Z)
                newPos.Z = CAMERA_BOUNDS_MIN_Z;

            camera.Position = newPos;
        }

        private bool KeyJustPressed(Keys key)
        {
            return currentKeyboardState.IsKeyDown(key) && prevKeyboardState.IsKeyUp(key);
        }

        private void ProcessKeyboard()
        {
            prevKeyboardState = currentKeyboardState;
            currentKeyboardState = Keyboard.GetState();

            if (KeyJustPressed(Keys.Escape))
                this.Exit();

            if (KeyJustPressed(Keys.H))
                displayHelp = !displayHelp;

            if (KeyJustPressed(Keys.M))
                camera.EnableMouseSmoothing = !camera.EnableMouseSmoothing;

            if (KeyJustPressed(Keys.P))
                enableParallax = !enableParallax;

            if (KeyJustPressed(Keys.T))
                enableColorMap = !enableColorMap;

            if (currentKeyboardState.IsKeyDown(Keys.LeftAlt) || currentKeyboardState.IsKeyDown(Keys.RightAlt))
            {
                if (KeyJustPressed(Keys.Enter))
                    ToggleFullScreen();
            }

            if (KeyJustPressed(Keys.Add))
            {
                camera.RotationSpeed += 0.01f;

                if (camera.RotationSpeed > 1.0f)
                    camera.RotationSpeed = 1.0f;
            }

            if (KeyJustPressed(Keys.Subtract))
            {
                camera.RotationSpeed -= 0.01f;

                if (camera.RotationSpeed <= 0.0f)
                    camera.RotationSpeed = 0.01f;
            }


            if (KeyJustPressed(Keys.R))
                foreach (GameObject box in boxes)
                    box.Reset();

            if (KeyJustPressed(Keys.F))
                ForcePush(camera.Position);


            //Freeze objects
            if (KeyJustPressed(Keys.Z))
                foreach (GameObject box in boxes)
                    box.Lock();

            //Unfreeze objects
            if (KeyJustPressed(Keys.X))
                foreach (GameObject box in boxes)
                    box.Unlock();

            if (KeyJustPressed(Keys.N))
            {
                GameObject[] new_boxes = new GameObject[boxes.Length + 1];

                boxes.CopyTo(new_boxes, 0);

                boxes = new_boxes;

                boxes[boxes.Length-1] = new GameObject(boxes[0], camera.Position);
            }
        }

        private void ForcePush(Vector3 origin)
        {
            float max_distance = 5.0f;
            float strength = 100;

            BoundingSphere sphere = new BoundingSphere(origin, max_distance);

            foreach (GameObject obj in boxes)
                if (sphere.Intersects(obj.BoundingBox))
                    obj.ApplyVelocity(strength * (obj.Position - origin) * (max_distance / (obj.Position - origin).Length()));
        }

        private void UpdateEffect()
        {
            if (enableParallax)
                effect.CurrentTechnique = effect.Techniques["ParallaxNormalMappingPointLighting"];
            else
                effect.CurrentTechnique = effect.Techniques["NormalMappingPointLighting"];

            effect.Parameters["worldMatrix"].SetValue(Matrix.Identity);
            effect.Parameters["worldInverseTransposeMatrix"].SetValue(Matrix.Identity);
            effect.Parameters["worldViewProjectionMatrix"].SetValue(camera.ViewMatrix * camera.ProjectionMatrix);

            effect.Parameters["cameraPos"].SetValue(camera.Position);
            effect.Parameters["globalAmbient"].SetValue(globalAmbient.ToVector4());
            effect.Parameters["scaleBias"].SetValue(scaleBias);

            effect.Parameters["light"].StructureMembers["dir"].SetValue(light.Direction);
            effect.Parameters["light"].StructureMembers["pos"].SetValue(light.Position);
            effect.Parameters["light"].StructureMembers["ambient"].SetValue(light.Ambient.ToVector4());
            effect.Parameters["light"].StructureMembers["diffuse"].SetValue(light.Diffuse.ToVector4());
            effect.Parameters["light"].StructureMembers["specular"].SetValue(light.Specular.ToVector4());
            effect.Parameters["light"].StructureMembers["spotInnerCone"].SetValue(light.SpotInnerConeRadians);
            effect.Parameters["light"].StructureMembers["spotOuterCone"].SetValue(light.SpotOuterConeRadians);
            effect.Parameters["light"].StructureMembers["radius"].SetValue(light.Radius);

            effect.Parameters["material"].StructureMembers["ambient"].SetValue(material.Ambient.ToVector4());
            effect.Parameters["material"].StructureMembers["diffuse"].SetValue(material.Diffuse.ToVector4());
            effect.Parameters["material"].StructureMembers["emissive"].SetValue(material.Emissive.ToVector4());
            effect.Parameters["material"].StructureMembers["specular"].SetValue(material.Specular.ToVector4());
            effect.Parameters["material"].StructureMembers["shininess"].SetValue(material.Shininess);
        }

        protected override void Update(GameTime gameTime)
        {
            if (!this.IsActive)
                return;

            foreach (GameObject box in boxes)
                box.Update(gameTime);

            DetectCollisions(gameTime);

            base.Update(gameTime);

            ProcessKeyboard();
            PerformCameraCollisionDetection();
            UpdateEffect();
            UpdateFrameRate(gameTime);
        }

        protected void DetectCollisions(GameTime gameTime)
        {
            for (int i=0; i<boxes.Length; i++)
                for (int j = i+1; j < boxes.Length; j++)
                    if (boxes[i].BoundingBox.Intersects(boxes[j].BoundingBox))
                        boxes[i].CollidesWith(boxes[j], gameTime);
        }

        private void UpdateFrameRate(GameTime gameTime)
        {
            framesPerSecond = (int)(1.0 / gameTime.ElapsedGameTime.TotalSeconds);
        }

        private void DrawText()
        {
            StringBuilder buffer = new StringBuilder();

            buffer.AppendLine("FPS: " + framesPerSecond.ToString());
            buffer.AppendLine();

            for (int i = 0; i < boxes.Length; i++)
            {
                buffer.AppendLine("Box " + i.ToString() + ":");

                buffer.AppendLine("X " + boxes[i].Position.X.ToString(" 0.00;-0.00") + "     " + "Vx " + boxes[i].Velocity.X.ToString(" 0.00;-0.00") + "     " + "Ax " + boxes[i].Acceleration.X.ToString(" 0.00;-0.00"));
                buffer.AppendLine("Y " + boxes[i].Position.Y.ToString(" 0.00;-0.00") + "     " + "Vy " + boxes[i].Velocity.Y.ToString(" 0.00;-0.00") + "     " + "Ay " + boxes[i].Acceleration.Y.ToString(" 0.00;-0.00"));
                buffer.AppendLine("Z " + boxes[i].Position.Z.ToString(" 0.00;-0.00") + "     " + "Vz " + boxes[i].Velocity.Z.ToString(" 0.00;-0.00") + "     " + "Az " + boxes[i].Acceleration.Z.ToString(" 0.00;-0.00"));

                buffer.AppendLine();
            }

            if (displayHelp)
            {
                buffer.AppendLine("Move mouse to free look");
                buffer.AppendLine();
                buffer.AppendLine("Press W and S to move forwards and backwards");
                buffer.AppendLine("Press A and D to strafe left and right");
                buffer.AppendLine("Press SPACE to jump");
                buffer.AppendLine("Press and hold LEFT CTRL to crouch");
                buffer.AppendLine("Press and hold LEFT SHIFT to run");
                buffer.AppendLine();
                buffer.AppendLine("Press M to toggle mouse smoothing");
                buffer.AppendLine("Press P to toggle between parallax normal mapping and normal mapping");
                buffer.AppendLine("Press NUMPAD +/- to change camera rotation speed");
                buffer.AppendLine("Press ALT + ENTER to toggle full screen");
                buffer.AppendLine();
                buffer.AppendLine("Press H to hide help");
            }
            else
            {
                //buffer.Append("\nPress H to display help");
            }

            spriteBatch.Begin(SpriteSortMode.Deferred, BlendState.AlphaBlend);
            spriteBatch.DrawString(spriteFont, buffer.ToString(), fontPos, Color.Yellow);
            spriteBatch.End();
        }

        protected override void Draw(GameTime gameTime)
        {
            if (!this.IsActive)
                return;

            GraphicsDevice.Clear(Color.CornflowerBlue);

            GraphicsDevice.BlendState = BlendState.Opaque;
            GraphicsDevice.DepthStencilState = DepthStencilState.Default;
            GraphicsDevice.SamplerStates[0] = SamplerState.LinearWrap;
            GraphicsDevice.SamplerStates[1] = SamplerState.LinearWrap;
            GraphicsDevice.SamplerStates[2] = SamplerState.LinearWrap;

            // Draw the room.

            if (enableColorMap)
            {
                room.Draw(GraphicsDevice, effect,
                    "colorMapTexture", "normalMapTexture", "heightMapTexture",
                    brickColorMap, brickNormalMap, brickHeightMap,
                    stoneColorMap, stoneNormalMap, stoneHeightMap,
                    woodColorMap, woodNormalMap, woodHeightMap);
            }
            else
            {
                room.Draw(GraphicsDevice, effect,
                    "colorMapTexture", "normalMapTexture", "heightMapTexture",
                    nullTexture, brickNormalMap, brickHeightMap,
                    nullTexture, stoneNormalMap, stoneHeightMap,
                    nullTexture, woodNormalMap, woodHeightMap);
            }

            foreach(GameObject box in boxes)
                box.DrawCrate(camera);

            DrawText();            
 
            base.Draw(gameTime);
        }
    }
}
