//   Copyright (C) 2015  Marek Kowalski (M.Kowalski@ire.pw.edu.pl), Jacek Naruniec (J.Naruniec@ire.pw.edu.pl)
//   License: MIT Software License   See LICENSE.txt for the full license.

//   If you use this software in your research, then please use the following citation:

//    Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 3D Data
//    Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 2015 International Conference on, Lyon, France, 2015

//    @INPROCEEDINGS{Kowalski15,
//        author={Kowalski, M. and Naruniec, J. and Daniluk, M.},
//        booktitle={3D Vision (3DV), 2015 International Conference on},
//        title={LiveScan3D: A Fast and Inexpensive 3D Data Acquisition System for Multiple Kinect v2 Sensors},
//        year={2015},
//    }
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Globalization;
using System.Windows.Forms.Layout;

using System.Diagnostics;

using OpenTK;
using OpenTK.Graphics;
using OpenTK.Input;

enum ECameraMode
{
    CAMERA_NONE, CAMERA_TRACK, CAMERA_DOLLY, CAMERA_ORBIT
}

namespace KinectServer
{
    public class OpenGLWindow : GameWindow
    {
        int PointCount;
        int LineCount;
        int FaceCount;
        int ObjectCount;

        VertexC4ubV3f[] VBO;
        float PointSize = 0.0f;
        ECameraMode CameraMode = ECameraMode.CAMERA_NONE;

        static float KEYBOARD_MOVE_SPEED = 0.01f;

        bool IsFullscreen = false;

        static float MOUSE_ORBIT_SPEED = 0.30f;     // 0 = SLOWEST, 1 = FASTEST
        static float MOUSE_DOLLY_SPEED = 0.2f;     // same as above...but much more sensitive
        static float MOUSE_TRACK_SPEED = 0.003f;    // same as above...but much more sensitive

        float g_heading;
        float g_pitch;
        float dx = 0.0f;
        float dy = 0.0f;

        byte brightnessModifier = 0;

        Vector2 MousePrevious = new Vector2();
        Vector2 MouseCurrent = new Vector2();
        float[] cameraPosition = new float[3];
        float[] targetPosition = new float[3];

        public List<float> vertices = new List<float>();
        public List<byte> colors = new List<byte>();
        public List<AffineTransform> cameraPoses = new List<AffineTransform>();
        public List<Body> bodies = new List<Body>();
        public KinectSettings settings = new KinectSettings();
        public List<float> faces = new List<float>();
        public List<float> objects = new List<float>();

        DateTime tFPSUpdateTimer = DateTime.Now;
        int nTickCounter = 0;

        bool bDrawMarkings = true;

        // this struct is used for drawing
        struct VertexC4ubV3f
        {
            public byte R, G, B, A;
            public Vector3 Position;

            public static int SizeInBytes = 16;
        }

        uint VBOHandle;

        /// <summary>Creates a 800x600 window with the specified title.</summary>
        public OpenGLWindow()
            : base(800, 600, GraphicsMode.Default, "LiveScan")
        {
            this.VSync = VSyncMode.Off;
            MouseUp += new EventHandler<MouseButtonEventArgs>(OnMouseButtonUp);
            MouseDown += new EventHandler<MouseButtonEventArgs>(OnMouseButtonDown);
            MouseMove += new EventHandler<MouseMoveEventArgs>(OnMouseMove);
            MouseWheel += new EventHandler<MouseWheelEventArgs>(OnMouseWheelChanged);

            KeyDown += new EventHandler<KeyboardKeyEventArgs>(OnKeyDown);
            
            cameraPosition[0] = 0;
            cameraPosition[1] = 0;
            cameraPosition[2] = 1.0f;
            targetPosition[0] = 0;
            targetPosition[1] = 0;
            targetPosition[2] = 0;
        }

        public void CloudUpdateTick()
        {
            nTickCounter++;
        }

        public void ToggleFullscreen()
        {
            if (IsFullscreen)
            {
                WindowBorder = WindowBorder.Resizable;
                WindowState = WindowState.Normal;
                ClientSize = new System.Drawing.Size(800, 600);
                CursorVisible = true;
            }
            else
            {
                CursorVisible = false;
                WindowBorder = WindowBorder.Hidden;
                WindowState = WindowState.Fullscreen;
            }
            IsFullscreen = !IsFullscreen;
        }

        void OnKeyDown(object sender, KeyboardKeyEventArgs e)
        {

            var keyboard = e.Keyboard;
            if (keyboard[Key.Escape])
            {
                Exit();
            }
            if (keyboard[Key.Plus])
            {
                PointSize += 0.1f;
                GL.PointSize(PointSize);
            }
            if (keyboard[Key.Minus])
            {
                if (PointSize != 0)
                    PointSize -= 0.1f;
                GL.PointSize(PointSize);
            }
            if (keyboard[Key.W])
                cameraPosition[2] -= KEYBOARD_MOVE_SPEED;
            if (keyboard[Key.A])
                cameraPosition[0] -= KEYBOARD_MOVE_SPEED;
            if (keyboard[Key.S])
                cameraPosition[2] += KEYBOARD_MOVE_SPEED;
            if (keyboard[Key.D])
                cameraPosition[0] += KEYBOARD_MOVE_SPEED;
            if (keyboard[Key.F])
                ToggleFullscreen();
            if (keyboard[Key.M])
                bDrawMarkings = !bDrawMarkings;
            if (keyboard[Key.O])
                brightnessModifier = (byte)Math.Max(0, brightnessModifier - 10);
            if (keyboard[Key.P])
                brightnessModifier = (byte)Math.Min(255, brightnessModifier + 10);
        }

        /// <summary>Load resources here.</summary>
        /// <param name="e">Not used.</param>
        protected override void OnLoad(EventArgs e)
        {
            base.OnLoad(e);

            Version version = new Version(GL.GetString(StringName.Version).Substring(0, 3));
            Version target = new Version(1, 5);
            if (version < target)
            {
                throw new NotSupportedException(String.Format(
                    "OpenGL {0} is required (you only have {1}).", target, version));
            }

            GL.ClearColor(.1f, 0f, .1f, 0f);
            GL.Enable(EnableCap.DepthTest);

            // Setup parameters for Points
            GL.PointSize(PointSize);
            GL.Enable(EnableCap.PointSmooth);
            GL.Hint(HintTarget.PointSmoothHint, HintMode.Nicest);

            // Setup VBO state
            GL.EnableClientState(EnableCap.ColorArray);
            GL.EnableClientState(EnableCap.VertexArray);
            
            GL.GenBuffers(1, out VBOHandle);

            // Since there's only 1 VBO in the app, might aswell setup here.
            GL.BindBuffer(BufferTarget.ArrayBuffer, VBOHandle);
            GL.ColorPointer(4, ColorPointerType.UnsignedByte, VertexC4ubV3f.SizeInBytes, (IntPtr)0);
            GL.VertexPointer(3, VertexPointerType.Float, VertexC4ubV3f.SizeInBytes, (IntPtr)(4 * sizeof(byte)));


            PointCount = 0;
            LineCount = 12;
            //   VBO = new VertexC4ubV3f[PointCount + 2 * LineCount];
            //    VBO = new VertexC4ubV3f[PointCount + 2 * LineCount + 3 * FaceCount]; //opengl 공간할당
            //VBO = new VertexC4ubV3f[PointCount + 2 * FaceCount * LineCount + 3 * FaceCount]; 
            VBO = new VertexC4ubV3f[PointCount + 28 * FaceCount + 24 * ObjectCount];
        }

        protected override void OnUnload(EventArgs e)
        {
            GL.DeleteBuffers(1, ref VBOHandle);
        }

        /// <summary>
        /// Called when your window is resized. Set your viewport here. It is also
        /// a good place to set up your projection matrix (which probably changes
        /// along when the aspect ratio of your window).
        /// </summary>
        /// <param name="e">Contains information on the new Width and Size of the GameWindow.</param>
        protected override void OnResize(EventArgs e)
        {
            GL.Viewport(0, 0, Width, Height);

            GL.MatrixMode(MatrixMode.Projection);
            Matrix4 p = Matrix4.CreatePerspectiveFieldOfView(MathHelper.PiOver4, Width / (float)Height, 0.1f, 50.0f);
            GL.LoadMatrix(ref p);

            GL.MatrixMode(MatrixMode.Modelview);
            Matrix4 mv = Matrix4.LookAt(Vector3.UnitZ, Vector3.Zero, Vector3.UnitY);
            GL.LoadMatrix(ref mv);
        }

        void OnMouseWheelChanged(object sender, MouseWheelEventArgs e)
        {
            dy = e.Delta * MOUSE_DOLLY_SPEED;

            cameraPosition[2] -= dy;

            //if (cameraPosition[2] < 0)
            //    cameraPosition[2] = 0;
                    
        }

        void OnMouseMove(object sender, MouseMoveEventArgs e)
        {
            MouseCurrent.X = e.Mouse.X;
            MouseCurrent.Y = e.Mouse.Y;

            // Now use mouse_delta to move the camera

            switch (CameraMode)
            {
                case ECameraMode.CAMERA_TRACK:
                    dx = MouseCurrent.X - MousePrevious.X;
                    dx *= MOUSE_TRACK_SPEED;

                    dy = MouseCurrent.Y - MousePrevious.Y;
                    dy *= MOUSE_TRACK_SPEED;

                    cameraPosition[0] -= dx;
                    cameraPosition[1] += dy;

                    //targetPosition[0] -= dx;
                    //targetPosition[1] += dy;

                    break;

                case ECameraMode.CAMERA_DOLLY:
                    dy = MouseCurrent.Y - MousePrevious.Y;
                    dy *= MOUSE_DOLLY_SPEED;

                    cameraPosition[2] -= dy;

                //    if (cameraPosition[2] < 0)
                 //       cameraPosition[2] = 0;

                    break;

                case ECameraMode.CAMERA_ORBIT:
                    dx = MouseCurrent.X - MousePrevious.X;
                    dx *= MOUSE_ORBIT_SPEED;

                    dy = MouseCurrent.Y - MousePrevious.Y;
                    dy *= MOUSE_ORBIT_SPEED;

                    g_heading += dx;
                    g_pitch += dy;

                    break;
            }
            MousePrevious.X = MouseCurrent.X;
            MousePrevious.Y = MouseCurrent.Y;
        }

        void OnMouseButtonUp(object sender, MouseButtonEventArgs e)
        {
            CameraMode = ECameraMode.CAMERA_NONE;
        }

        void OnMouseButtonDown(object sender, MouseButtonEventArgs e)
        {
            switch (e.Button)
            {
                case MouseButton.Left:
                    CameraMode = ECameraMode.CAMERA_ORBIT;
                    break;
                case MouseButton.Middle:
                    CameraMode = ECameraMode.CAMERA_DOLLY;
                    break;
                case MouseButton.Right:
                    CameraMode = ECameraMode.CAMERA_TRACK;
                    break;
            }
            MousePrevious.X = Mouse.X;
            MousePrevious.Y = Mouse.Y;
        }

        protected override void OnUpdateFrame(FrameEventArgs e)
        {
            if ((DateTime.Now - tFPSUpdateTimer).Seconds >= 1)
            {
                double FPS = nTickCounter / (DateTime.Now - tFPSUpdateTimer).TotalSeconds;
                this.Title = "FPS: " + string.Format("{0:F}", FPS);

                tFPSUpdateTimer = DateTime.Now;
                nTickCounter = 0;
            }


            lock (vertices)
            {
                lock (settings)
                {
                    bool bShowSkeletons = settings.bShowSkeletons;

                    PointCount = vertices.Count / 3;
                    LineCount = 0;
                    if (bDrawMarkings)
                    {
                        //bounding box
                        LineCount += 12;
                        //markers
                        LineCount += settings.lMarkerPoses.Count * 3;
                        //cameras
                        LineCount += cameraPoses.Count * 3;
                        if (bShowSkeletons)
                            LineCount += 24 * bodies.Count;
                    }

                    FaceCount = faces.Count / 10;
                    ObjectCount = objects.Count / 11;


                    //VBO = new VertexC4ubV3f[PointCount + 2 * FaceCount];
                    // VBO = new VertexC4ubV3f[PointCount + 2 * LineCount + 3 * FaceCount]; //opengl 공간할당
                    VBO = new VertexC4ubV3f[PointCount + 28 * FaceCount + 24 * ObjectCount];

                    for (int i = 0; i < PointCount; i++)
                    {
                        VBO[i].R = (byte)Math.Max(0, Math.Min(255, (colors[i * 3] + brightnessModifier)));
                        VBO[i].G = (byte)Math.Max(0, Math.Min(255, (colors[i * 3 + 1] + brightnessModifier)));
                        VBO[i].B = (byte)Math.Max(0, Math.Min(255, (colors[i * 3 + 2] + brightnessModifier)));
                        VBO[i].A = 255;
                        VBO[i].Position.X = vertices[i * 3];
                        VBO[i].Position.Y = vertices[i * 3 + 1];
                        VBO[i].Position.Z = vertices[i * 3 + 2];
                    }

             
                    int n = 0;
                    // 지금은 야매로 한당
                    // M_bc 넘겨서 작은거 찾게하기??
                    //       if (FaceCount == 1) // 야매 
                    //       if (FaceCount != 0) // 야매 

                    //180604 : 사람 여러명으로 테스트필요 & 깜빡이는거 막고 싶으면 FaceCount나 ObjectCount가 0이면 이전값 기억하고 있다가 그거 출력


                    for (int ii = 0; ii < FaceCount; ii++) // 야매 아닌거
                    {

                        AddLine(PointCount + n, faces[(ii * 10) + 0], faces[(ii * 10) + 1], faces[(ii * 10) + 2],
                        faces[(ii * 10) + 3], faces[(ii * 10) + 4], faces[(ii * 10) + 5]);
                        n += 2;

                        AddLineg(PointCount + n, faces[(ii * 10) + 3], faces[(ii * 10) + 4], faces[(ii * 10) + 5],
                        faces[(ii * 10) + 6], faces[(ii * 10) + 7], faces[(ii * 10) + 8]);
                        n += 2;

                        float bounds = 0.2f;

                        //bottom vertices
                        //first vertex
                        AddLineg(PointCount + n, faces[(ii * 10) + 6] - bounds, faces[(ii * 10) + 7] - bounds, faces[(ii * 10) + 8] - bounds,
                        faces[(ii * 10) + 6] + bounds, faces[(ii * 10) + 7] - bounds, faces[(ii * 10) + 8] - bounds);
                        n += 2;
                        AddLineg(PointCount + n, faces[(ii * 10) + 6] - bounds, faces[(ii * 10) + 7] - bounds, faces[(ii * 10) + 8] - bounds,
                        faces[(ii * 10) + 6] - bounds, faces[(ii * 10) + 7] + bounds, faces[(ii * 10) + 8] - bounds);
                        n += 2;
                        AddLineg(PointCount + n, faces[(ii * 10) + 6] - bounds, faces[(ii * 10) + 7] - bounds, faces[(ii * 10) + 8] - bounds,
                        faces[(ii * 10) + 6] - bounds, faces[(ii * 10) + 7] - bounds, faces[(ii * 10) + 8] + bounds);
                        n += 2;
                        //second vertex
                        AddLineg(PointCount + n, faces[(ii * 10) + 6] + bounds, faces[(ii * 10) + 7] - bounds, faces[(ii * 10) + 8] - bounds,
                        faces[(ii * 10) + 6] + bounds, faces[(ii * 10) + 7] + bounds, faces[(ii * 10) + 8] - bounds);
                        n += 2;
                        AddLineg(PointCount + n, faces[(ii * 10) + 6] + bounds, faces[(ii * 10) + 7] - bounds, faces[(ii * 10) + 8] - bounds,
                        faces[(ii * 10) + 6] + bounds, faces[(ii * 10) + 7] - bounds, faces[(ii * 10) + 8] + bounds);
                        n += 2;
                        //third vertex
                        AddLineg(PointCount + n, faces[(ii * 10) + 6] + bounds, faces[(ii * 10) + 7] - bounds, faces[(ii * 10) + 8] + bounds,
                        faces[(ii * 10) + 6] + bounds, faces[(ii * 10) + 7] + bounds, faces[(ii * 10) + 8] + bounds);
                        n += 2;
                        AddLineg(PointCount + n, faces[(ii * 10) + 6] + bounds, faces[(ii * 10) + 7] - bounds, faces[(ii * 10) + 8] + bounds,
                        faces[(ii * 10) + 6] - bounds, faces[(ii * 10) + 7] - bounds, faces[(ii * 10) + 8] + bounds);
                        n += 2;
                        //fourth vertex                     
                        AddLineg(PointCount + n, faces[(ii * 10) + 6] - bounds, faces[(ii * 10) + 7] - bounds, faces[(ii * 10) + 8] + bounds,
                        faces[(ii * 10) + 6] - bounds, faces[(ii * 10) + 7] + bounds, faces[(ii * 10) + 8] + bounds);
                        n += 2;
                        //top vertices
                        //fifth vertex 
                        AddLineg(PointCount + n, faces[(ii * 10) + 6] - bounds, faces[(ii * 10) + 7] + bounds, faces[(ii * 10) + 8] - bounds,
                        faces[(ii * 10) + 6] + bounds, faces[(ii * 10) + 7] + bounds, faces[(ii * 10) + 8] - bounds);
                        n += 2;
                        AddLineg(PointCount + n, faces[(ii * 10) + 6] - bounds, faces[(ii * 10) + 7] + bounds, faces[(ii * 10) + 8] - bounds,
                        faces[(ii * 10) + 6] - bounds, faces[(ii * 10) + 7] + bounds, faces[(ii * 10) + 8] + bounds);
                        n += 2;
                        //sixth vertex
                        AddLineg(PointCount + n, faces[(ii * 10) + 6] + bounds, faces[(ii * 10) + 7] + bounds, faces[(ii * 10) + 8] + bounds,
                        faces[(ii * 10) + 6] + bounds, faces[(ii * 10) + 7] + bounds, faces[(ii * 10) + 8] - bounds);
                        n += 2;
                        AddLineg(PointCount + n, faces[(ii * 10) + 6] + bounds, faces[(ii * 10) + 7] + bounds, faces[(ii * 10) + 8] + bounds,
                        faces[(ii * 10) + 6] - bounds, faces[(ii * 10) + 7] + bounds, faces[(ii * 10) + 8] + bounds);
                        n += 2;
                    }

                    

                    for (int i = 0; i < ObjectCount; i++) // 야매 아닌거
                    { 

                      float bounds = 0.2f;



                            if (objects[(i * 11) + 8] == 1)
                            {
                                AddLine(PointCount + n, objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] - bounds,
                              objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] - bounds);
                                n += 2;
                                AddLine(PointCount + n, objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] - bounds,
                                objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] - bounds);
                                n += 2;
                                AddLine(PointCount + n, objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] - bounds,
                                objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] + bounds);
                                n += 2;
                                //second vertex
                                AddLine(PointCount + n, objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] - bounds,
                                objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] - bounds);
                                n += 2;
                                AddLine(PointCount + n, objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] - bounds,
                                objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] + bounds);
                                n += 2;
                                //third vertex
                                AddLine(PointCount + n, objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] + bounds,
                                objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] + bounds);
                                n += 2;
                                AddLine(PointCount + n, objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] + bounds,
                                objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] + bounds);
                                n += 2;
                                //fourth vertex                     
                                AddLine(PointCount + n, objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] + bounds,
                                objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] + bounds);
                                n += 2;
                                //top vertices
                                //fifth vertex 
                                AddLine(PointCount + n, objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] - bounds,
                                objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] - bounds);
                                n += 2;
                                AddLine(PointCount + n, objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] - bounds,
                                objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] + bounds);
                                n += 2;
                                //sixth vertex
                                AddLine(PointCount + n, objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] + bounds,
                                objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] - bounds);
                                n += 2;
                                AddLine(PointCount + n, objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] + bounds,
                                objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] + bounds);
                                n += 2;


                                this.Title = "intent object id: " + string.Format("{0:F}", objects[(i * 11) + 0]); //19/04/07 추후 people id objects[(i * 11) + 9]도 넣을 수 있음

                            }

                        /*   //190411 : 디스플레이 로스 감소
                        else
                        { 
                                if (objects[(i * 11) + 10] == 1)
                                {
                                    AddLineb1(PointCount + n, objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] - bounds,
                                    objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] - bounds);
                                    n += 2;
                                    AddLineb1(PointCount + n, objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] - bounds,
                                    objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] - bounds);
                                    n += 2;
                                    AddLineb1(PointCount + n, objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] - bounds,
                                    objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] + bounds);
                                    n += 2;
                                    //second vertex
                                    AddLineb1(PointCount + n, objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] - bounds,
                                    objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] - bounds);
                                    n += 2;
                                    AddLineb1(PointCount + n, objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] - bounds,
                                    objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] + bounds);
                                    n += 2;
                                    //third vertex
                                    AddLineb1(PointCount + n, objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] + bounds,
                                    objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] + bounds);
                                    n += 2;
                                    AddLineb1(PointCount + n, objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] + bounds,
                                    objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] + bounds);
                                    n += 2;
                                    //fourth vertex                     
                                    AddLineb1(PointCount + n, objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] + bounds,
                                    objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] + bounds);
                                    n += 2;
                                    //top vertices
                                    //fifth vertex 
                                    AddLineb1(PointCount + n, objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] - bounds,
                                    objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] - bounds);
                                    n += 2;
                                    AddLineb1(PointCount + n, objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] - bounds,
                                    objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] + bounds);
                                    n += 2;
                                    //sixth vertex
                                    AddLineb1(PointCount + n, objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] + bounds,
                                    objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] - bounds);
                                    n += 2;
                                    AddLineb1(PointCount + n, objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] + bounds,
                                    objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] + bounds);
                                    n += 2;
                                }

                                else if (objects[(i * 11) + 10] == 2)
                                {
                                    AddLineb2(PointCount + n, objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] - bounds,
                                    objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] - bounds);
                                    n += 2;
                                    AddLineb2(PointCount + n, objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] - bounds,
                                    objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] - bounds);
                                    n += 2;
                                    AddLineb2(PointCount + n, objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] - bounds,
                                    objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] + bounds);
                                    n += 2;
                                    //second vertex
                                    AddLineb2(PointCount + n, objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] - bounds,
                                    objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] - bounds);
                                    n += 2;
                                    AddLineb2(PointCount + n, objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] - bounds,
                                    objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] + bounds);
                                    n += 2;
                                    //third vertex
                                    AddLineb2(PointCount + n, objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] + bounds,
                                    objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] + bounds);
                                    n += 2;
                                    AddLineb2(PointCount + n, objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] + bounds,
                                    objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] + bounds);
                                    n += 2;
                                    //fourth vertex                     
                                    AddLineb2(PointCount + n, objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] + bounds,
                                    objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] + bounds);
                                    n += 2;
                                    //top vertices
                                    //fifth vertex 
                                    AddLineb2(PointCount + n, objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] - bounds,
                                    objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] - bounds);
                                    n += 2;
                                    AddLineb2(PointCount + n, objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] - bounds,
                                    objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] + bounds);
                                    n += 2;
                                    //sixth vertex
                                    AddLineb2(PointCount + n, objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] + bounds,
                                    objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] - bounds);
                                    n += 2;
                                    AddLineb2(PointCount + n, objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] + bounds,
                                    objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] + bounds);
                                    n += 2;
                                }

                                else if (objects[(i * 11) + 10] == 3)
                                {
                                    AddLineb3(PointCount + n, objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] - bounds,
                                    objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] - bounds);
                                    n += 2;
                                    AddLineb3(PointCount + n, objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] - bounds,
                                    objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] - bounds);
                                    n += 2;
                                    AddLineb3(PointCount + n, objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] - bounds,
                                    objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] + bounds);
                                    n += 2;
                                    //second vertex
                                    AddLineb3(PointCount + n, objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] - bounds,
                                    objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] - bounds);
                                    n += 2;
                                    AddLineb3(PointCount + n, objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] - bounds,
                                    objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] + bounds);
                                    n += 2;
                                    //third vertex
                                    AddLineb3(PointCount + n, objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] + bounds,
                                    objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] + bounds);
                                    n += 2;
                                    AddLineb3(PointCount + n, objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] + bounds,
                                    objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] + bounds);
                                    n += 2;
                                    //fourth vertex                     
                                    AddLineb3(PointCount + n, objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] + bounds,
                                    objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] + bounds);
                                    n += 2;
                                    //top vertices
                                    //fifth vertex 
                                    AddLineb3(PointCount + n, objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] - bounds,
                                    objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] - bounds);
                                    n += 2;
                                    AddLineb3(PointCount + n, objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] - bounds,
                                    objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] + bounds);
                                    n += 2;
                                    //sixth vertex
                                    AddLineb3(PointCount + n, objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] + bounds,
                                    objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] - bounds);
                                    n += 2;
                                    AddLineb3(PointCount + n, objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] + bounds,
                                    objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] + bounds);
                                    n += 2;
                                }

                                else if (objects[(i * 11) + 10] == 4)
                                {
                                    AddLineb4(PointCount + n, objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] - bounds,
                                    objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] - bounds);
                                    n += 2;
                                    AddLineb4(PointCount + n, objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] - bounds,
                                    objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] - bounds);
                                    n += 2;
                                    AddLineb4(PointCount + n, objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] - bounds,
                                    objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] + bounds);
                                    n += 2;
                                    //second vertex
                                    AddLineb4(PointCount + n, objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] - bounds,
                                    objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] - bounds);
                                    n += 2;
                                    AddLineb4(PointCount + n, objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] - bounds,
                                    objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] + bounds);
                                    n += 2;
                                    //third vertex
                                    AddLineb4(PointCount + n, objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] + bounds,
                                    objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] + bounds);
                                    n += 2;
                                    AddLineb4(PointCount + n, objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] + bounds,
                                    objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] + bounds);
                                    n += 2;
                                    //fourth vertex                     
                                    AddLineb4(PointCount + n, objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] - bounds, objects[(i * 11) + 3] + bounds,
                                    objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] + bounds);
                                    n += 2;
                                    //top vertices
                                    //fifth vertex 
                                    AddLineb4(PointCount + n, objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] - bounds,
                                    objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] - bounds);
                                    n += 2;
                                    AddLineb4(PointCount + n, objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] - bounds,
                                    objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] + bounds);
                                    n += 2;
                                    //sixth vertex
                                    AddLineb4(PointCount + n, objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] + bounds,
                                    objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] - bounds);
                                    n += 2;
                                    AddLineb4(PointCount + n, objects[(i * 11) + 1] + bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] + bounds,
                                    objects[(i * 11) + 1] - bounds, objects[(i * 11) + 2] + bounds, objects[(i * 11) + 3] + bounds);
                                    n += 2;
                                }


                            }

                        

                            */
                    }
                    

                    /*

                      if (bDrawMarkings)
                      {
                          int iCurLineCount = 0;
                          iCurLineCount += AddBoundingBox(PointCount + 2 * iCurLineCount);
                          for (int i = 0; i < settings.lMarkerPoses.Count; i++)
                          {
                              iCurLineCount += AddMarker(PointCount + 2 * iCurLineCount, settings.lMarkerPoses[i].pose);
                          }
                          for (int i = 0; i < cameraPoses.Count; i++)
                          {
                              iCurLineCount += AddCamera(PointCount + 2 * iCurLineCount, cameraPoses[i]);
                          }
                          if (bShowSkeletons)
                              iCurLineCount += AddBodies(PointCount + 2 * iCurLineCount);
                      }

                      */

                }
            }
        }

        /// <summary>
        /// Called when it is time to render the next frame. Add your rendering code here.
        /// </summary>
        /// <param name="e">Contains timing information.</param>
        protected override void OnRenderFrame(FrameEventArgs e)
        {        
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

            GL.PushMatrix();

            GL.MatrixMode(MatrixMode.Modelview);
            GL.Translate(-cameraPosition[0], -cameraPosition[1], -cameraPosition[2]);
            GL.Rotate(g_pitch, 1.0f, 0.0f, 0.0f);
            GL.Rotate(g_heading, 0.0f, 1.0f, 0.0f);

            // Tell OpenGL to discard old VBO when done drawing it and reserve memory _now_ for a new buffer.
            // without this, GL would wait until draw operations on old VBO are complete before writing to it
            //GL.BufferData(BufferTarget.ArrayBuffer, (IntPtr)(VertexC4ubV3f.SizeInBytes * (PointCount + 2 * LineCount)), IntPtr.Zero, BufferUsageHint.StreamDraw);
            //GL.BufferData(BufferTarget.ArrayBuffer, (IntPtr)(VertexC4ubV3f.SizeInBytes * (PointCount + 2 * LineCount + 3 * FaceCount)), IntPtr.Zero, BufferUsageHint.StreamDraw);
            GL.BufferData(BufferTarget.ArrayBuffer, (IntPtr)(VertexC4ubV3f.SizeInBytes * (PointCount + 28 * FaceCount + 24 * ObjectCount)), IntPtr.Zero, BufferUsageHint.StreamDraw);
            // Fill newly allocated buffer

            //GL.BufferData(BufferTarget.ArrayBuffer, (IntPtr)(VertexC4ubV3f.SizeInBytes * (PointCount + 2 * LineCount)), VBO, BufferUsageHint.StreamDraw);
            // GL.BufferData(BufferTarget.ArrayBuffer, (IntPtr)(VertexC4ubV3f.SizeInBytes * (PointCount + 2 * LineCount + 3 * FaceCount)), VBO, BufferUsageHint.StreamDraw);
            GL.BufferData(BufferTarget.ArrayBuffer, (IntPtr)(VertexC4ubV3f.SizeInBytes * (PointCount + 28 * FaceCount + 24 * ObjectCount)), VBO, BufferUsageHint.StreamDraw);
            // PointCount + 2 * LineCount 크기??

            GL.DrawArrays(BeginMode.Points, 0, PointCount);
            GL.LineWidth(3);
            //      GL.DrawArrays(BeginMode.Lines, PointCount, 2 * LineCount);
            GL.DrawArrays(BeginMode.Lines, PointCount, PointCount + 28 * FaceCount + 24 * ObjectCount);



            // 여기 수정하면 출력가능할듯 ㅎ
            GL.PopMatrix();

            SwapBuffers();
        }

        private int AddBoundingBox(int startIdx)
        {
            int nLinesBeingAdded = 12;
            //2 points per line
            int nPointsToAdd = 2 * nLinesBeingAdded;

            for (int i = startIdx; i < startIdx + nPointsToAdd; i++)
            {
                VBO[i].R = 255;
                VBO[i].G = 255;
                VBO[i].B = 0;
                VBO[i].A = 0;
            }

            int n = 0;

            //bottom vertices
            //first vertex
            AddLine(startIdx + n, settings.aMinBounds[0], settings.aMinBounds[1], settings.aMinBounds[2],
                settings.aMaxBounds[0], settings.aMinBounds[1], settings.aMinBounds[2]);
            n += 2;
            AddLine(startIdx + n, settings.aMinBounds[0], settings.aMinBounds[1], settings.aMinBounds[2],
                settings.aMinBounds[0], settings.aMaxBounds[1], settings.aMinBounds[2]);
            n += 2;
            AddLine(startIdx + n, settings.aMinBounds[0], settings.aMinBounds[1], settings.aMinBounds[2],
                settings.aMinBounds[0], settings.aMinBounds[1], settings.aMaxBounds[2]);
            n += 2;

            //second vertex
            AddLine(startIdx + n, settings.aMaxBounds[0], settings.aMinBounds[1], settings.aMinBounds[2],
                settings.aMaxBounds[0], settings.aMaxBounds[1], settings.aMinBounds[2]);
            n += 2;
            AddLine(startIdx + n, settings.aMaxBounds[0], settings.aMinBounds[1], settings.aMinBounds[2],
                settings.aMaxBounds[0], settings.aMinBounds[1], settings.aMaxBounds[2]);
            n += 2;

            //third vertex
            AddLine(startIdx + n, settings.aMaxBounds[0], settings.aMinBounds[1], settings.aMaxBounds[2],
                settings.aMaxBounds[0], settings.aMaxBounds[1], settings.aMaxBounds[2]);
            n += 2;
            AddLine(startIdx + n, settings.aMaxBounds[0], settings.aMinBounds[1], settings.aMaxBounds[2],
                settings.aMinBounds[0], settings.aMinBounds[1], settings.aMaxBounds[2]);
            n += 2;

            //fourth vertex
            AddLine(startIdx + n, settings.aMinBounds[0], settings.aMinBounds[1], settings.aMaxBounds[2],
                settings.aMinBounds[0], settings.aMaxBounds[1], settings.aMaxBounds[2]);
            n += 2;

            //top vertices
            //fifth vertex 
            AddLine(startIdx + n, settings.aMinBounds[0], settings.aMaxBounds[1], settings.aMinBounds[2],
                settings.aMaxBounds[0], settings.aMaxBounds[1], settings.aMinBounds[2]);
            n += 2;
            AddLine(startIdx + n, settings.aMinBounds[0], settings.aMaxBounds[1], settings.aMinBounds[2],
                settings.aMinBounds[0], settings.aMaxBounds[1], settings.aMaxBounds[2]);
            n += 2;

            //sixth vertex
            AddLine(startIdx + n, settings.aMaxBounds[0], settings.aMaxBounds[1], settings.aMaxBounds[2],
                settings.aMaxBounds[0], settings.aMaxBounds[1], settings.aMinBounds[2]);
            n += 2;
            AddLine(startIdx + n, settings.aMaxBounds[0], settings.aMaxBounds[1], settings.aMaxBounds[2],
                settings.aMinBounds[0], settings.aMaxBounds[1], settings.aMaxBounds[2]);
            n += 2;

            return nLinesBeingAdded;
        }

        private int AddMarker(int startIdx, AffineTransform pose)
        {
            int nLinesBeingAdded = 3;
            //2 points per line
            int nPointsToAdd = 2 * nLinesBeingAdded;

            for (int i = startIdx; i < startIdx + nPointsToAdd; i++)
            {
                VBO[i].R = 255;
                VBO[i].G = 0;
                VBO[i].B = 0;
                VBO[i].A = 0;
            }

            int n = 0;

            float x0 = pose.t[0];
            float y0 = pose.t[1];
            float z0 = pose.t[2];

            float x1 = 0.1f;
            float y1 = 0.1f;
            float z1 = 0.1f;

            float x2 = pose.R[0, 0] * x1;
            float y2 = pose.R[1, 0] * x1;
            float z2 = pose.R[2, 0] * x1;

            x2 += pose.t[0];
            y2 += pose.t[1];
            z2 += pose.t[2];

            AddLine(startIdx + n, x0, y0, z0, x2, y2, z2);
            n += 2;

            x2 = pose.R[0, 1] * y1;
            y2 = pose.R[1, 1] * y1;
            z2 = pose.R[2, 1] * y1;

            x2 += pose.t[0];
            y2 += pose.t[1];
            z2 += pose.t[2];

            AddLine(startIdx + n, x0, y0, z0, x2, y2, z2);
            n += 2;

            x2 = pose.R[0, 2] * z1;
            y2 = pose.R[1, 2] * z1;
            z2 = pose.R[2, 2] * z1;

            x2 += pose.t[0];
            y2 += pose.t[1];
            z2 += pose.t[2];

            AddLine(startIdx + n, x0, y0, z0, x2, y2, z2);
            n += 2;

            return nLinesBeingAdded;
        }

        private int AddCamera(int startIdx, AffineTransform pose)
        {
            int nLinesBeingAdded = 3;
            //2 points per line
            int nPointsToAdd = 2 * nLinesBeingAdded;

            for (int i = startIdx; i < startIdx + nPointsToAdd; i++)
            {
                VBO[i].R = 0;
                VBO[i].G = 255;
                VBO[i].B = 0;
                VBO[i].A = 0;
            }

            int n = 0;

            float x0 = pose.t[0];
            float y0 = pose.t[1];
            float z0 = pose.t[2];

            float x1 = 0.1f;
            float y1 = 0.1f;
            float z1 = 0.1f;

            float x2 = pose.R[0, 0] * x1;
            float y2 = pose.R[1, 0] * x1;
            float z2 = pose.R[2, 0] * x1;

            x2 += pose.t[0];
            y2 += pose.t[1];
            z2 += pose.t[2];

            AddLine(startIdx + n, x0, y0, z0, x2, y2, z2);
            n += 2;

            x2 = pose.R[0, 1] * y1;
            y2 = pose.R[1, 1] * y1;
            z2 = pose.R[2, 1] * y1;

            x2 += pose.t[0];
            y2 += pose.t[1];
            z2 += pose.t[2];

            AddLine(startIdx + n, x0, y0, z0, x2, y2, z2);
            n += 2;

            x2 = pose.R[0, 2] * z1;
            y2 = pose.R[1, 2] * z1;
            z2 = pose.R[2, 2] * z1;

            x2 += pose.t[0];
            y2 += pose.t[1];
            z2 += pose.t[2];

            AddLine(startIdx + n, x0, y0, z0, x2, y2, z2);
            n += 2;

            return nLinesBeingAdded;
        }

        private int AddBone(int bodyIdx, JointType jointType0, JointType jointType1, int startIdx)
        {
            Point3f joint0 = bodies[bodyIdx].lJoints[(int)jointType0].position;
            Point3f joint1 = bodies[bodyIdx].lJoints[(int)jointType1].position;
            AddLine(startIdx, joint0.X, joint0.Y, joint0.Z, joint1.X, joint1.Y, joint1.Z);
            return 2;
        }

        private int AddBodies(int startIdx)
        {
            int nLinesToAdd = 24 * bodies.Count;
            int nPointsToAdd = nLinesToAdd * 2;

            for (int i = startIdx; i < startIdx + nPointsToAdd; i++)
            {
                VBO[i].R = 0;
                VBO[i].G = 255;
                VBO[i].B = 0;
                VBO[i].A = 0;
            }

            int n = 0;

            for (int bodyIdx = 0; bodyIdx < bodies.Count; bodyIdx++)
            {
                if (bodies[bodyIdx].bTracked == false)
                    continue;

                //Torso
                n += AddBone(bodyIdx, JointType.JointType_Head, JointType.JointType_Neck, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_Neck, JointType.JointType_SpineShoulder, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_SpineShoulder, JointType.JointType_SpineMid, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_SpineMid, JointType.JointType_SpineBase, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_SpineShoulder, JointType.JointType_ShoulderRight, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_SpineShoulder, JointType.JointType_ShoulderLeft, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_SpineBase, JointType.JointType_HipRight, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_SpineBase, JointType.JointType_HipLeft, startIdx + n);

                // Right Arm    
                n += AddBone(bodyIdx, JointType.JointType_ShoulderRight, JointType.JointType_ElbowRight, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_ElbowRight, JointType.JointType_WristRight, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_WristRight, JointType.JointType_HandRight, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_HandRight, JointType.JointType_HandTipRight, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_WristRight, JointType.JointType_ThumbRight, startIdx + n);

                // Left Arm
                n += AddBone(bodyIdx, JointType.JointType_ShoulderLeft, JointType.JointType_ElbowLeft, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_ElbowLeft, JointType.JointType_WristLeft, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_WristLeft, JointType.JointType_HandLeft, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_HandLeft, JointType.JointType_HandTipLeft, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_WristLeft, JointType.JointType_ThumbLeft, startIdx + n);

                // Right Leg
                n += AddBone(bodyIdx, JointType.JointType_HipRight, JointType.JointType_KneeRight, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_KneeRight, JointType.JointType_AnkleRight, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_AnkleRight, JointType.JointType_FootRight, startIdx + n);

                // Left Leg
                n += AddBone(bodyIdx, JointType.JointType_HipLeft, JointType.JointType_KneeLeft, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_KneeLeft, JointType.JointType_AnkleLeft, startIdx + n);
                n += AddBone(bodyIdx, JointType.JointType_AnkleLeft, JointType.JointType_FootLeft, startIdx + n);
            }         

            return nLinesToAdd;
        }

        private void AddLine(int startIdx, float x0, float y0, float z0, 
            float x1, float y1, float z1)
        {

            VBO[startIdx].R = 255;
            VBO[startIdx].G = 0;
            VBO[startIdx].B = 0;
            VBO[startIdx].A = 0;



            VBO[startIdx].Position.X = x0;
            VBO[startIdx].Position.Y = y0;
            VBO[startIdx].Position.Z = z0;



            VBO[startIdx + 1].R = 255;
            VBO[startIdx + 1].G = 0;
            VBO[startIdx + 1].B = 0;
            VBO[startIdx + 1].A = 0;


            VBO[startIdx + 1].Position.X = x1;
            VBO[startIdx + 1].Position.Y = y1;
            VBO[startIdx + 1].Position.Z = z1;
        }

        private void AddLineg(int startIdx, float x0, float y0, float z0,
    float x1, float y1, float z1)
        {

            VBO[startIdx].R = 0;
            VBO[startIdx].G = 255;
            VBO[startIdx].B = 0;
            VBO[startIdx].A = 0;



            VBO[startIdx].Position.X = x0;
            VBO[startIdx].Position.Y = y0;
            VBO[startIdx].Position.Z = z0;

           

            VBO[startIdx + 1].R = 0;
            VBO[startIdx + 1].G = 255;
            VBO[startIdx + 1].B = 0;
            VBO[startIdx + 1].A = 0;


            VBO[startIdx + 1].Position.X = x1;
            VBO[startIdx + 1].Position.Y = y1;
            VBO[startIdx + 1].Position.Z = z1;
        }


        private void AddLineb1(int startIdx, float x0, float y0, float z0,
float x1, float y1, float z1) //노란
        {

            VBO[startIdx].R = 255;
            VBO[startIdx].G = 255;
            VBO[startIdx].B = 0;
            VBO[startIdx].A = 0;



            VBO[startIdx].Position.X = x0;
            VBO[startIdx].Position.Y = y0;
            VBO[startIdx].Position.Z = z0;



            VBO[startIdx + 1].R = 255;
            VBO[startIdx + 1].G = 255;
            VBO[startIdx + 1].B = 0;
            VBO[startIdx + 1].A = 0;


            VBO[startIdx + 1].Position.X = x1;
            VBO[startIdx + 1].Position.Y = y1;
            VBO[startIdx + 1].Position.Z = z1;
        }

        private void AddLineb2(int startIdx, float x0, float y0, float z0,
float x1, float y1, float z1) //청록
        {

            VBO[startIdx].R = 0;
            VBO[startIdx].G = 255;
            VBO[startIdx].B = 255;
            VBO[startIdx].A = 0;



            VBO[startIdx].Position.X = x0;
            VBO[startIdx].Position.Y = y0;
            VBO[startIdx].Position.Z = z0;



            VBO[startIdx + 1].R = 0;
            VBO[startIdx + 1].G = 255;
            VBO[startIdx + 1].B = 255;
            VBO[startIdx + 1].A = 0;


            VBO[startIdx + 1].Position.X = x1;
            VBO[startIdx + 1].Position.Y = y1;
            VBO[startIdx + 1].Position.Z = z1;
        }

        private void AddLineb3(int startIdx, float x0, float y0, float z0,
float x1, float y1, float z1) //보라색
        {

            VBO[startIdx].R = 148;
            VBO[startIdx].G = 0;
            VBO[startIdx].B = 211;
            VBO[startIdx].A = 0;



            VBO[startIdx].Position.X = x0;
            VBO[startIdx].Position.Y = y0;
            VBO[startIdx].Position.Z = z0;



            VBO[startIdx + 1].R = 148;
            VBO[startIdx + 1].G = 0;
            VBO[startIdx + 1].B = 211;
            VBO[startIdx + 1].A = 0;


            VBO[startIdx + 1].Position.X = x1;
            VBO[startIdx + 1].Position.Y = y1;
            VBO[startIdx + 1].Position.Z = z1;
        }

        private void AddLineb4(int startIdx, float x0, float y0, float z0,
float x1, float y1, float z1) //파랑
        {

            VBO[startIdx].R = 0;
            VBO[startIdx].G = 0;
            VBO[startIdx].B = 255;
            VBO[startIdx].A = 0;



            VBO[startIdx].Position.X = x0;
            VBO[startIdx].Position.Y = y0;
            VBO[startIdx].Position.Z = z0;



            VBO[startIdx + 1].R = 0;
            VBO[startIdx + 1].G = 0;
            VBO[startIdx + 1].B = 255;
            VBO[startIdx + 1].A = 0;


            VBO[startIdx + 1].Position.X = x1;
            VBO[startIdx + 1].Position.Y = y1;
            VBO[startIdx + 1].Position.Z = z1;
        }



    }
}

