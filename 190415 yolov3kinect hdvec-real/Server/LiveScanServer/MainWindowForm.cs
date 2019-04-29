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
using System.Runtime.InteropServices;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Threading;
using System.Windows.Forms;
using System.Globalization;
using System.Runtime.Serialization;

using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Timers;

using System.Diagnostics;


namespace KinectServer
{
    public partial class MainWindowForm : Form
    {
        [DllImport("ICP.dll")]
        static extern float ICP(IntPtr verts1, IntPtr verts2, int nVerts1, int nVerts2, float[] R, float[] t, int maxIter = 200);

        KinectServer oServer;
        TransferServer oTransferServer;


        //Those three variables are shared with the OpenGLWindow class and are used to exchange data with it.
        //Vertices from all of the sensors
        List<float> lAllVertices = new List<float>();
        //Color data from all of the sensors
        List<byte> lAllColors = new List<byte>();
        //Sensor poses from all of the sensors
        List<AffineTransform> lAllCameraPoses = new List<AffineTransform>();
        //Body data from all of the sensors
        List<Body> lAllBodies = new List<Body>();
        List<float> lAllFaces = new List<float>();
        List<float> lAllObject = new List<float>();
        List<float> objectbuffer = new List<float>(); //  비교 버퍼 저장

        long preuTimestamp = 0; //과거 시간 저장

        List<List<Single>> ListObject = new List<List<Single>>(); // 정렬된 오브젝트 리스트 저장 , 2차원 배열
        //
        List<float> lAllPoints = new List<float>();
        List<float> plAllPoints = new List<float>(); //과거값 저장

        float current_cellphone_place_index = -1; //첫번째 검출될 때도 SendChange call 가능
        float previous_cellphone_place_index = -1;
        int trann = 0; // 맨처음 잡힌 reomte 전송안하려고
        int trancn = 0; // 보팅후 보내려고


        List<float> lAllPoints1 = new List<float>(); // 각각 구하려고
//        List<float> lAllPoints2 = new List<float>();
//        List<float> lAllPoints3 = new List<float>();
//        List<float> lAllPoints4 = new List<float>();



        //칼만 필터 값들
        float[,] K_X = new float[18, 3]; // 18 -> 나중에 사람 수만큼 , 3 : x,y,z
        float[,] K_X_next = new float[18, 3]; // 18 -> 나중에 사람 수만큼 , 3 : x,y,z
        float[,] K_P = new float[18, 3]; // 18 -> 나중에 사람 수만큼 , 3 : x,y,z
        float[,] K_P_next = new float[18, 3]; // 18 -> 나중에 사람 수만큼 , 3 : x,y,z
        float[,] K_Q = new float[18, 3]; // 18 -> 나중에 사람 수만큼 , 3 : x,y,z
        float[,] K_R = new float[18, 3]; // 18 -> 나중에 사람 수만큼 , 3 : x,y,z
        float[,] K_K = new float[18, 3]; // 18 -> 나중에 사람 수만큼 , 3 : x,y,z
        float[,] K_Z = new float[18, 3]; // 18 -> 나중에 사람 수만큼 , 3 : x,y,z
        bool[] kalinit = new bool[18];
        


        bool bServerRunning = false;
        bool bRecording = false;
        bool bSaving = false;

        //Live view open or not
        bool bLiveViewRunning = false;

        System.Timers.Timer oStatusBarTimer = new System.Timers.Timer();

        KinectSettings oSettings = new KinectSettings();
        //The live view window class
        OpenGLWindow oOpenGLWindow;

        public MainWindowForm()
        {
            //This tries to read the settings from "settings.bin", if it failes the settings stay at default values.


            try
            {
                IFormatter formatter = new System.Runtime.Serialization.Formatters.Binary.BinaryFormatter();
                Stream stream = new FileStream("settings.bin", FileMode.Open, FileAccess.Read);
                oSettings = (KinectSettings)formatter.Deserialize(stream);
                stream.Close();
            }
            catch (Exception)
            {
            }

            oServer = new KinectServer(oSettings);
            oServer.eSocketListChanged += new SocketListChangedHandler(UpdateListView);
            oTransferServer = new TransferServer();
            oTransferServer.lVertices = lAllVertices;
            oTransferServer.lColors = lAllColors;

            InitializeComponent();
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            //The current settings are saved to a files.
            IFormatter formatter = new System.Runtime.Serialization.Formatters.Binary.BinaryFormatter();

            Stream stream = new FileStream("settings.bin", FileMode.Create, FileAccess.Write);
            formatter.Serialize(stream, oSettings);
            stream.Close();

            oServer.StopServer();
            oTransferServer.StopServer();
        }

        //Starts the server
        private void btStart_Click(object sender, EventArgs e)
        {
            bServerRunning = !bServerRunning;

            if (bServerRunning)
            {
                oServer.StartServer();
                oTransferServer.StartServer();
                btStart.Text = "Stop server";
            }
            else
            {
                oServer.StopServer();
                oTransferServer.StopServer();
                btStart.Text = "Start server";
            }
        }

        //Opens the settings form
        private void btSettings_Click(object sender, EventArgs e)
        {
            SettingsForm form = new SettingsForm();
            form.oSettings = oSettings;
            form.oServer = oServer;
            form.Show();
        }

        //Performs recording which is synchronized frame capture.
        //The frames are downloaded from the clients and saved once recording is finished.
        private void recordingWorker_DoWork(object sender, DoWorkEventArgs e)
        {
            oServer.ClearStoredFrames();

            int nCaptured = 0;
            BackgroundWorker worker = (BackgroundWorker)sender;
            while (!worker.CancellationPending)
            {
                oServer.CaptureSynchronizedFrame();

                nCaptured++;
                SetStatusBarOnTimer("Captured frame " + (nCaptured).ToString() + ".", 5000);
            }
        }

        private void recordingWorker_RunWorkerCompleted(object sender, RunWorkerCompletedEventArgs e)
        {
            //After recording has been terminated it is time to begin saving the frames.
            //Saving is downloading the frames from clients and saving them locally.
            bSaving = true;

            btRecord.Text = "Stop saving";
            btRecord.Enabled = true;

            savingWorker.RunWorkerAsync();
        }

        //Opens the live view window
        private void OpenGLWorker_DoWork(object sender, DoWorkEventArgs e)
        {
            bLiveViewRunning = true;
            oOpenGLWindow = new OpenGLWindow();

            //The variables below are shared between this class and the OpenGLWindow.
            lock (lAllVertices)
            {
          
                oOpenGLWindow.vertices = lAllVertices;
                oOpenGLWindow.colors = lAllColors;
                oOpenGLWindow.cameraPoses = lAllCameraPoses;
                oOpenGLWindow.bodies = lAllBodies;
                oOpenGLWindow.settings = oSettings;

                oOpenGLWindow.faces = lAllPoints1;
                //oOpenGLWindow.objects = lAllObject;
                oOpenGLWindow.objects = objectbuffer;

            }
            oOpenGLWindow.Run();
        }

        private void OpenGLWorker_RunWorkerCompleted(object sender, RunWorkerCompletedEventArgs e)
        {
            bLiveViewRunning = false;
            updateWorker.CancelAsync();
        }

        private void savingWorker_DoWork(object sender, DoWorkEventArgs e)
        {
            int nFrames = 0;

            string outDir = "out" + "\\" + txtSeqName.Text + "\\";
            DirectoryInfo di = Directory.CreateDirectory(outDir);

            BackgroundWorker worker = (BackgroundWorker)sender;
            //This loop is running till it is either cancelled (using the btRecord button), or till there are no more stored frames.
            while (!worker.CancellationPending)
            {
                List<List<byte>> lFrameRGBAllDevices = new List<List<byte>>();
                List<List<float>> lFrameVertsAllDevices = new List<List<float>>();

                bool success = oServer.GetStoredFrame(lFrameRGBAllDevices, lFrameVertsAllDevices);

                //This indicates that there are no more stored frames.
                if (!success)
                    break;

                nFrames++;
                int nVerticesTotal = 0;
                for (int i = 0; i < lFrameRGBAllDevices.Count; i++)
                {
                    nVerticesTotal += lFrameVertsAllDevices[i].Count;
                }

                List<byte> lFrameRGB = new List<byte>();
                List<Single> lFrameVerts = new List<Single>();

                SetStatusBarOnTimer("Saving frame " + (nFrames).ToString() + ".", 5000);
                for (int i = 0; i < lFrameRGBAllDevices.Count; i++)
                {
                    lFrameRGB.AddRange(lFrameRGBAllDevices[i]);
                    lFrameVerts.AddRange(lFrameVertsAllDevices[i]);

                    //This is ran if the frames from each client are to be placed in separate files.
                    if (!oSettings.bMergeScansForSave)
                    {
                        string outputFilename = outDir + "\\" + nFrames.ToString().PadLeft(5, '0') + i.ToString() + ".ply";
                        Utils.saveToPly(outputFilename, lFrameVertsAllDevices[i], lFrameRGBAllDevices[i], oSettings.bSaveAsBinaryPLY);
                    }
                }

                //This is ran if the frames from all clients are to be placed in a single file.
                if (oSettings.bMergeScansForSave)
                {
                    string outputFilename = outDir + "\\" + nFrames.ToString().PadLeft(5, '0') + ".ply";
                    Utils.saveToPly(outputFilename, lFrameVerts, lFrameRGB, oSettings.bSaveAsBinaryPLY);
                }
            }
        }

        private void savingWorker_RunWorkerCompleted(object sender, RunWorkerCompletedEventArgs e)
        {
            oServer.ClearStoredFrames();
            bSaving = false;

            //If the live view window was open, we need to restart the UpdateWorker.
            if (bLiveViewRunning)
                RestartUpdateWorker();

            btRecord.Enabled = true;
            btRecord.Text = "Start recording";
            btRefineCalib.Enabled = true;
            btCalibrate.Enabled = true;
        }

        //Continually requests frames that will be displayed in the live view window.
        private void updateWorker_DoWork(object sender, DoWorkEventArgs e)
        {
            List<List<byte>> lFramesRGB = new List<List<byte>>();
            List<List<Single>> lFramesVerts = new List<List<Single>>();
            List<List<Body>> lFramesBody = new List<List<Body>>();
            List<List<Single>> lFramesFace = new List<List<Single>>();
            List<List<Single>> lFramesObject = new List<List<Single>>();

            BackgroundWorker worker = (BackgroundWorker)sender;
            while (!worker.CancellationPending)
            {
                Thread.Sleep(1);
                oServer.GetLatestFrame(lFramesRGB, lFramesVerts, lFramesBody, lFramesFace, lFramesObject);

                //Update the vertex and color lists that are common between this class and the OpenGLWindow.
                lock (lAllVertices)
                {
                    lAllVertices.Clear();
                    lAllColors.Clear();
                    lAllBodies.Clear();
                    lAllCameraPoses.Clear();
                    lAllFaces.Clear();
                    lAllObject.Clear();


                    for (int i = 0; i < lFramesRGB.Count; i++)
                    {
                        lAllVertices.AddRange(lFramesVerts[i]);
                        lAllColors.AddRange(lFramesRGB[i]);
                        lAllBodies.AddRange(lFramesBody[i]);
                        lAllFaces.AddRange(lFramesFace[i]);
                        lAllObject.AddRange(lFramesObject[i]);                       
                    }

                    /////////////////////////
                    //여기에 오브젝트 리스트 관리함수
                    /*
                    for (int i = 0; i < lAllObject.Count; i++)
                    {
                        Console.Write(lAllObject[i]);
                        Console.Write(" ");
                        if ((i + 1) % 8 == 0) Console.Write("\n");
                    }
                    */

                    // 1초마다 object voting => 추후에



                    //180604 : 추가 되야할게 상태 플래그 : 0(걍 검출된 물체),1(변화된 물체),2(관심객체)
                    // 만약 관심 객체면 플래그 하나 더넣어서 누가 보고 있는 지도 (ex: people 1)

                    //180525 여기에 오브젝트 중복 제거?? 같은 오브젝트의 XYZ로 빼서 중복 제거?
                    //데이터 목록화?
                    //180604 : 이거보다 얼굴 처럼 버퍼에 따로 저장하는게 맞는거 같아

                    ///////////////////////////////////////////// 오브젝트 중복제거 -> 추후 바운딩박스도 넘겨 받으면 크기 고려 가능
                    int obc = lAllObject.Count / 8;  // 오브젝트 개수


                    float[,] oa = new float[obc, 8]; //object point


                    for (int cot = 0; cot < obc; cot++) //초기화 한번 해야하나??
                    {
                        oa[cot, 0] = lAllObject[(cot * 8) + 0]; //object id
                        oa[cot, 1] = lAllObject[(cot * 8) + 1]; //object x
                        oa[cot, 2] = lAllObject[(cot * 8) + 2]; //object y
                        oa[cot, 3] = lAllObject[(cot * 8) + 3]; //object z
                        oa[cot, 4] = lAllObject[(cot * 8) + 4]; //kinect id
                        oa[cot, 5] = lAllObject[(cot * 8) + 5]; //시
                        oa[cot, 6] = lAllObject[(cot * 8) + 6]; //분
                        oa[cot, 7] = lAllObject[(cot * 8) + 7]; //초
                    }


                    //List<float> objectbuffer = new List<float>();
                    objectbuffer.Clear();  //

                    // buffer 세팅 //

                    for (int ot = 0; ot < obc; ot++)
                    {

                        int ocnt = objectbuffer.Count / 11; // 상태 플래그, 사람 id, 3d 위치 추가
                        int dacnt = 0; //중복횟수 체크
                        for (int ott = 0; ott < ocnt; ott++) // 나중에 함수로 정리....
                        {
                            float dist = (objectbuffer[(ott * 11) + 1] - oa[ot, 1]) * (objectbuffer[(ott * 11) + 1] - oa[ot, 1]) + (objectbuffer[(ott * 11) + 2] - oa[ot, 2]) * (objectbuffer[(ott * 11) + 2] - oa[ot, 2]) + (objectbuffer[(ott * 11) + 3] - oa[ot, 3]) * (objectbuffer[(ott * 11) + 3] - oa[ot, 3]);

                            if (dist < 0.06) // 실험적으로 설정필요 => 180605 : 추후 바운딩 박스 크기받아서 선택하는 것도 넣으면 좋을듯 /0.06 => 약 24cm 
                            {


                                objectbuffer[(ott * 11) + 1] = (objectbuffer[(ott * 11) + 1] + oa[ot, 1]) / 2;
                                objectbuffer[(ott * 11) + 2] = (objectbuffer[(ott * 11) + 2] + oa[ot, 2]) / 2;
                                objectbuffer[(ott * 11) + 3] = (objectbuffer[(ott * 11) + 3] + oa[ot, 3]) / 2;
                                dacnt = dacnt + 1;
                                //지금은 단순 평균인데 추후에 평균필터 적용 및 카운트해서 변경 필요?
                            }


                        }

                        //190409 : 0,0,0 은 아닌데 0,0,0에 근사한 쓰레기 값 제거... => 임시방편 , 근본적인 원인 분석 필요.....
                       // float distze = (oa[ot, 1]) * (oa[ot, 1]) + (oa[ot, 2]) * (oa[ot, 2]) + (oa[ot, 3]) * (oa[ot, 3]);


                 //       if (distze > 0.01 * 0.01) // 1 cm 보다는 커야함...
        


                        if (dacnt == 0 && ocnt != 0) //중복되는거 하나도 없으면 채우기 && 버퍼가 안비워져 있을때
                        {
                            // 버퍼에 Add
                            objectbuffer.Add(oa[ot, 0]); //object id
                            objectbuffer.Add(oa[ot, 1]); //object x
                            objectbuffer.Add(oa[ot, 2]); //object y
                            objectbuffer.Add(oa[ot, 3]); //object z
                            objectbuffer.Add(oa[ot, 4]); //kinect id
                            objectbuffer.Add(oa[ot, 5]); //시
                            objectbuffer.Add(oa[ot, 6]); //분
                            objectbuffer.Add(oa[ot, 7]); //초
                            objectbuffer.Add(0); // 상태 플래그 => 0: 아무이상없음 1: 관심객체 2: 변화된객체
                            objectbuffer.Add(0); // 상태플래그 1일때 사람 id 

                            if (oa[ot, 1] > 0 && oa[ot, 3] > 0)  // 190407 : 0 >= 을 > 로 바꿈
                            {
                                objectbuffer.Add(4); // 3D xyz 좌표 기준 물체 위치
                            }
                            else if (oa[ot, 1] < 0 && oa[ot, 3] > 0)
                            {
                                objectbuffer.Add(3); // 3D xyz 좌표 기준 물체 위치
                            }
                            else if (oa[ot, 1] > 0 && oa[ot, 3] < 0)
                            {
                                objectbuffer.Add(2); // 3D xyz 좌표 기준 물체 위치
                            }
                            else if (oa[ot, 1] < 0 && oa[ot, 3] < 0)
                            {
                                objectbuffer.Add(1); // 3D xyz 좌표 기준 물체 위치
                            }
                            else
                            {
                                objectbuffer.Add(0); //혹시 모르니까 넣어두긴 하는데 별 의미 없을듯
                            }

                        }


                        if (ocnt == 0)
                        {
                            //버퍼 비었을 때 채우기
                            objectbuffer.Add(oa[ot, 0]); //object id
                            objectbuffer.Add(oa[ot, 1]); //object x
                            objectbuffer.Add(oa[ot, 2]); //object y
                            objectbuffer.Add(oa[ot, 3]); //object z
                            objectbuffer.Add(oa[ot, 4]); //kinect id
                            objectbuffer.Add(oa[ot, 5]); //시
                            objectbuffer.Add(oa[ot, 6]); //분
                            objectbuffer.Add(oa[ot, 7]); //초
                            objectbuffer.Add(0); // 상태 플래그 => 0: 아무이상없음 1: 관심객체 2: 변화된객체
                            objectbuffer.Add(0); // 상태플래그 1일때 사람 id 

                            if (oa[ot, 1] > 0 && oa[ot, 3] > 0) // 190407 : 0 >= 을 > 로 바꿈
                            {
                                objectbuffer.Add(4); // 3D xyz 좌표 기준 물체 위치
                            }
                            else if (oa[ot, 1] < 0 && oa[ot, 3] > 0)
                            {
                                objectbuffer.Add(3); // 3D xyz 좌표 기준 물체 위치
                            }
                            else if (oa[ot, 1] > 0 && oa[ot, 3] < 0)
                            {
                                objectbuffer.Add(2); // 3D xyz 좌표 기준 물체 위치
                            }
                            else if (oa[ot, 1] < 0 && oa[ot, 3] < 0)
                            {
                                objectbuffer.Add(1); // 3D xyz 좌표 기준 물체 위치
                            }
                            else
                            {
                                objectbuffer.Add(0); //혹시 모르니까 넣어두긴 하는데 별 의미 없을듯
                            }

                        }

            

                    }



                    /////////////////////////////////////////////


                    //180605 : objectbuffer의 상태 플래그 변화시키는놈 필요...
                    //상태 플래그 : 시선 또는 오브젝트 변화
                    //관심객체면 관심 사람 나오게
                    //관심위치 기준점 잡기 => 디버깅 필요..  


                    //변화된 오브젝트 잡는놈......
                    // => objectbuffer 써서??
                    List<float> oneObject = new List<float>();
                    oneObject.Clear();

                    for (int i = 0; i < objectbuffer.Count / 11; i++)
                    {

                        oneObject.Add(objectbuffer[(i * 11) + 0]);
                        oneObject.Add(objectbuffer[(i * 11) + 1]);
                        oneObject.Add(objectbuffer[(i * 11) + 2]);
                        oneObject.Add(objectbuffer[(i * 11) + 3]);
                        oneObject.Add(objectbuffer[(i * 11) + 4]);
                        oneObject.Add(objectbuffer[(i * 11) + 5]);
                        oneObject.Add(objectbuffer[(i * 11) + 6]);
                        oneObject.Add(objectbuffer[(i * 11) + 7]);
                        oneObject.Add(objectbuffer[(i * 11) + 8]);
                        oneObject.Add(objectbuffer[(i * 11) + 9]);
                        oneObject.Add(objectbuffer[(i * 11) + 10]);

                        // kinect ID : ListObject[i][4]
                        // object ID : ListObject[i][0]
                        // cellphone ID : 67 (확인 완료)

                        if (oneObject[10] == 1 || oneObject[10] == 2) //일단 place 1,2 일때만 체크한다.
                        { 

                        
                            if (oneObject[0] == 65 || oneObject[0] == 67) // 일단 리모트와 셀폰에 대해서만 검출한다. 
                            {
                            // 객체가 하나의 키넥트에서만 검출된다고 가정
                            // 중복되는거 제거는 나중에
                            current_cellphone_place_index = oneObject[10];
                            

                                if (current_cellphone_place_index != previous_cellphone_place_index)
                                {

                                    long unixTimestamp = (long)(DateTime.UtcNow.Subtract(new DateTime(1970, 1, 1))).TotalSeconds; //long time
                            //SendChange(oneObject[0], oneObject[4], unixTimestamp);


                                        if (trann == 0)
                                        {
                                             trann = 1; //맨처음 검출된 리모컨 안보냄
                                             previous_cellphone_place_index = current_cellphone_place_index;
                                         }
                                         else
                                        {
                                        /*
                                        oneObject[8] = 2;
                                        //SendChangeYame(oneObject[4], unixTimestamp); // 개야매
                                        SendChanget(oneObject[0], oneObject[4], oneObject[8], oneObject[9], oneObject[10], unixTimestamp);
                                        previous_cellphone_place_index = current_cellphone_place_index;
                                        */
                                
                                                 if (trancn < 30) //30 프레임 연속 변화 정보 있어야 보낸다 => 그래도 여전히 중복 체크는 피할수 없는듯 => 앞에서 오브젝트 전체 리스트 정렬 해야함
                                                 {
                                                        trancn++;
                                                  }
                                                  else if (trancn >= 30) // 변화가 30프레임이상 지속적으로 있으면 그친구 보낸다 => 잠깐 생기는 거는 무시가능
                                                  {
                                                        oneObject[8] = 2;
                                                       // SendChanget(oneObject[0], oneObject[4], oneObject[8], oneObject[9], oneObject[10], unixTimestamp);
                                                        previous_cellphone_place_index = current_cellphone_place_index;
                                                        trancn = 0;
                                                   }
                                
                                          }


                                        }
                                    }
                                }

                              oneObject.Clear();
                        


                            }

                    //ListObject
                    // res = comparelist();
                    // if res == 1 addElement();
                    // 1분마다 한번씩 
                    // res = checkList();
                    // if res != 0  SendChange(res);
                    // 1분마다 한번씩
                    // deleteIndex = checkList2();
                    // 있으면(=deleteIndex.Count > 0) deleteElement();


                    ////////////////////////


                    /* 180403 : 칼만이 및 안정화 및 3D 시각화를 위한 DetecP(lAllFaces 획득) 임시 제거
                    //칼만 필터 초기화

                    DetecP(lAllVertices, lAllFaces);

                    int Fnum = lAllPoints.Count / 9;  // 얼굴 개수

                    for (int i = 0; i < Fnum; i++)
                    {
                        if(!kalinit[i])
                        { 
                            K_P[i,0] = 1; K_P[i,1] = 1; K_P[i,2] = 1;
                            K_Q[i,0] = .2f; K_Q[i, 1] = .2f; K_Q[i, 2] = .2f;
                            K_R[i,0] = .1f; K_R[i,1] = .1f; K_R[i,2] = .1f;
                            K_X[i,0] = 0; K_X[i,1] = 0; K_X[i,2] = 0;

                            kalinit[i] = true;
                        }
                        for(int j=0; j<3; j++)
                        {
                            K_Z[i, j] = lAllPoints[9 * i + 6 + j]; 

                            K_X_next[i,j] = K_X[i,j];
                            K_P_next[i,j] = K_P[i,j] + K_Q[i, j];
                            K_K[i, j] = K_P_next[i, j] / (K_P_next[i, j] + K_R[i, j]);
                            K_X[i, j] = K_X_next[i, j] + K_K[i, j] * (K_Z[i, j] - K_X_next[i, j]);
                            K_P[i, j] = (1 - K_K[i, j]) * K_P_next[i, j];
                            lAllPoints[9 * i + 6 + j] = K_X[i, j];
                        }
                    }
                    */


                    /*
                    if (lAllPoints.Count != 0)  // 잘 안먹히는것 같다;; 원인 불명;; //위치 변경 필요??
                    {
                        plAllPoints = lAllPoints;
                    }
                    else
                    {
                        lAllPoints = plAllPoints;
                    }
                    */




                    // 18/04/03 : Detecpp안에 또는 앞에 lAllFaces의 중복되는 부분 제거 필요            ////////////////////////////////////////////////////////////////       
                    // 18/04/11 : 버퍼 이용해서 비교하는 방식 이용 => 평균 필터
                    // 18/05/04 : 여기 터짐 -3- 잡야아함


                    /*
                                    int afc = lAllFaces.Count / 6;  // 얼굴 개수


                                    float[,] fa = new float[afc, 3]; //head point
                                    float[,] ba = new float[afc, 3]; //head point

                                    for (int count = 0; count < afc; count++)
                                    {                                       
                                        fa[count, 0] = lAllFaces[(count * 6) + 0]; //012=3Dheadpoint
                                        fa[count, 1] = lAllFaces[(count * 6) + 1];
                                        fa[count, 2] = lAllFaces[(count * 6) + 2];
                                        ba[count, 0] = lAllFaces[(count * 6) + 3]; //345=3Dnosepoint
                                        ba[count, 1] = lAllFaces[(count * 6) + 4];
                                        ba[count, 2] = lAllFaces[(count * 6) + 5];
                                    }


                                    List<float> facebuffer = new List<float>(); //  비교 버퍼 저장
                                    List<int> facecount = new List<int>(); // 카운터 저장
                                    facebuffer.Clear();
                                    facecount.Clear();

                                    // buffer 세팅 //

                                    for (int ct = 0; ct < afc; ct++)
                                    {

                                     int bufcnt = facebuffer.Count / 6;

                                        for (int ctt = 0; ctt < bufcnt; ctt++) // 나중에 함수로 정리....
                                        {
                                            float dist = (facebuffer[(ctt * 6) + 0] - fa[ct, 0]) * (facebuffer[(ctt * 6) + 0] - fa[ct, 0]) + (facebuffer[(ctt * 6) + 1] - fa[ct, 1]) * (facebuffer[(ctt * 6) + 1] - fa[ct, 1]) + (facebuffer[(ctt * 6) + 2] - fa[ct, 2]) * (facebuffer[(ctt * 6) + 2] - fa[ct, 2]);

                                            if (dist < 0.03) // 실험적으로 설정필요 
                                            {
                                                //평균필터 적용 및 카운트
                                                
                                                //180604 : 여기서 터진다

                                                facebuffer[(ctt * 6) + 0] = facecount[ctt] / (facecount[ctt] + 1) * facebuffer[(ctt * 6) + 0] + fa[ct, 0] / (facecount[ctt] + 1);
                                                facebuffer[(ctt * 6) + 1] = facecount[ctt] / (facecount[ctt] + 1) * facebuffer[(ctt * 6) + 1] + fa[ct, 1] / (facecount[ctt] + 1);
                                                facebuffer[(ctt * 6) + 2] = facecount[ctt] / (facecount[ctt] + 1) * facebuffer[(ctt * 6) + 2] + fa[ct, 2] / (facecount[ctt] + 1);
                                                facebuffer[(ctt * 6) + 3] = facecount[ctt] / (facecount[ctt] + 1) * facebuffer[(ctt * 6) + 3] + ba[ct, 3] / (facecount[ctt] + 1);
                                                facebuffer[(ctt * 6) + 4] = facecount[ctt] / (facecount[ctt] + 1) * facebuffer[(ctt * 6) + 4] + ba[ct, 4] / (facecount[ctt] + 1);
                                                facebuffer[(ctt * 6) + 5] = facecount[ctt] / (facecount[ctt] + 1) * facebuffer[(ctt * 6) + 5] + ba[ct, 5] / (facecount[ctt] + 1);
                                                facecount[ctt]++;
                                                // 요거 그냥 평균으로 바꿀까?

                                                // 그냥 다 더한 다음에 평균 내는게 젤 좋지 않냐??
                                            }
                                            else
                                            {
                                                // 버퍼에 Add
                                                facebuffer.Add(fa[ct, 0]); //해당 얼굴중심 데이터
                                                facebuffer.Add(fa[ct, 1]);
                                                facebuffer.Add(fa[ct, 2]);
                                                facebuffer.Add(ba[ct, 0]); //해당 코데이터
                                                facebuffer.Add(ba[ct, 1]);
                                                facebuffer.Add(ba[ct, 2]);
                                                facecount.Add(1);
                                            }

                                        }

                                        if (bufcnt == 0)
                                        {
                                            //버퍼 비었을 때 채우기
                                            facebuffer.Add(fa[ct, 0]); //해당 얼굴중심 데이터
                                            facebuffer.Add(fa[ct, 1]);
                                            facebuffer.Add(fa[ct, 2]);
                                            facebuffer.Add(ba[ct, 0]); //해당 코데이터
                                            facebuffer.Add(ba[ct, 1]);
                                            facebuffer.Add(ba[ct, 2]);
                                            facecount.Add(1);
                                        }



                                    }

                    */

                    ///////////////////////////////////////////////////////////////////////////////////////////////



                    //150531 : 드디어 중복되는 얼굴 잡음 ㅠㅠ
                    //오브젝트도 이거 처럼하면 될듯요.....
                    


                    int afc = lAllFaces.Count / 7;  // 얼굴 개수


                    float[,] fa = new float[afc, 3]; //head point
                    float[,] ba = new float[afc, 3]; //nose point
                    float[,] cca = new float[afc, 1]; //people id

                    for (int count = 0; count < afc; count++)
                    {
                        fa[count, 0] = lAllFaces[(count * 7) + 0]; //012=3Dheadpoint
                        fa[count, 1] = lAllFaces[(count * 7) + 1];
                        fa[count, 2] = lAllFaces[(count * 7) + 2];
                        ba[count, 0] = lAllFaces[(count * 7) + 3]; //345=3Dnosepoint
                        ba[count, 1] = lAllFaces[(count * 7) + 4];
                        ba[count, 2] = lAllFaces[(count * 7) + 5];
                        cca[count, 0] = lAllFaces[(count * 7) + 6]; // id
                    }


                    List<float> facebuffer = new List<float>(); //  비교 버퍼 저장
                    facebuffer.Clear();

                    // buffer 세팅 //

                    for (int ct = 0; ct < afc; ct++)
                    {

                        int bufcnt = facebuffer.Count / 7;
                        int facnt = 0; //중복된 횟수 체크

                        for (int ctt = 0; ctt < bufcnt; ctt++) // 나중에 함수로 정리....
                        {
                            float dist = (facebuffer[(ctt * 6) + 0] - fa[ct, 0]) * (facebuffer[(ctt * 6) + 0] - fa[ct, 0]) + (facebuffer[(ctt * 6) + 1] - fa[ct, 1]) * (facebuffer[(ctt * 6) + 1] - fa[ct, 1]) + (facebuffer[(ctt * 6) + 2] - fa[ct, 2]) * (facebuffer[(ctt * 6) + 2] - fa[ct, 2]);

                            if (dist < 0.035) // 실험적으로 설정필요  0.03 => 약 17cm / 0.04 => 약 20cm /0.035 => 약 18.7cm
                            {

                                facebuffer[(ctt * 6) + 0] = (facebuffer[(ctt * 6) + 0] + fa[ct, 0]) / 2;
                                facebuffer[(ctt * 6) + 1] = (facebuffer[(ctt * 6) + 1] + fa[ct, 1]) / 2;
                                facebuffer[(ctt * 6) + 2] = (facebuffer[(ctt * 6) + 2] + fa[ct, 2]) / 2;
                                facebuffer[(ctt * 6) + 3] = (facebuffer[(ctt * 6) + 3] + ba[ct, 0]) / 2;
                                facebuffer[(ctt * 6) + 4] = (facebuffer[(ctt * 6) + 4] + ba[ct, 1]) / 2;
                                facebuffer[(ctt * 6) + 5] = (facebuffer[(ctt * 6) + 5] + ba[ct, 2]) / 2;
                                //지금은 단순 평균인데 추후에 평균필터 적용 및 카운트해서 변경 필요?
                                facnt = facnt + 1;
                            }

                        }

                        if (facnt == 0 && bufcnt != 0)  //중복된거 없으면 버퍼에 add => 요건 버퍼에 값있을 때 add해야하는거여
                        {
                            // 버퍼에 Add
                            facebuffer.Add(fa[ct, 0]); //해당 얼굴중심 데이터
                            facebuffer.Add(fa[ct, 1]);
                            facebuffer.Add(fa[ct, 2]);
                            facebuffer.Add(ba[ct, 0]); //해당 코데이터
                            facebuffer.Add(ba[ct, 1]);
                            facebuffer.Add(ba[ct, 2]);
                            facebuffer.Add(cca[ct, 0]); //id
                        }

                        if (bufcnt == 0)
                        {
                            //버퍼 비었을 때 채우기
                            facebuffer.Add(fa[ct, 0]); //해당 얼굴중심 데이터
                            facebuffer.Add(fa[ct, 1]);
                            facebuffer.Add(fa[ct, 2]);
                            facebuffer.Add(ba[ct, 0]); //해당 코데이터
                            facebuffer.Add(ba[ct, 1]);
                            facebuffer.Add(ba[ct, 2]);
                            facebuffer.Add(cca[ct, 0]); //id
                        }



                    }



                    /*
                                        // 방법 1 따로 따로보내기
                                        DetecPP(lFramesVerts[0], lAllFaces, lAllPoints1);
                                        DetecPP(lFramesVerts[1], lAllFaces, lAllPoints2);
                                        DetecPP(lFramesVerts[2], lAllFaces, lAllPoints3);
                                        DetecPP(lFramesVerts[3], lAllFaces, lAllPoints4);


                                        if (lAllPoints1.Count != 0) // 해당하는 포인트 있으면 보냄
                                        {
                                                oServer.sendim(0, lAllPoints1); 
                                        }
                                        if (lAllPoints1.Count != 0) // 해당하는 포인트 있으면 보냄
                                        {
                                            oServer.sendim(1, lAllPoints2);
                                        }
                                        if (lAllPoints1.Count != 0) // 해당하는 포인트 있으면 보냄
                                        {
                                            oServer.sendim(2, lAllPoints3);
                                        }
                                        if (lAllPoints1.Count != 0) // 해당하는 포인트 있으면 보냄
                                        {
                                            oServer.sendim(3, lAllPoints4);
                                        }
                    */

                    //180527 요앞에 얼굴 중복되는거 제거하는 부분 나와야함 ;; => 그럼 필터 쓰기위해 id 인식도 가능??


                    //방법 2 lAllPoints 랑 그냥 비교해서 해당하는 점만 따로보내기 ' ';;? =>  연결 개수마다 알아서 세팅되게



                    DetecPP(lAllVertices, facebuffer, lAllPoints1);
                    //DetecPP(lAllVertices, lAllFaces, lAllPoints1);

                    for (int si = 0; si < lFramesRGB.Count; si++)
                    {
                        SendPP(lFramesVerts[si], lAllFaces, lAllPoints1, si);
                    }

                    //필터링 끝내고 보내는게 좋겟지??
                    // 추정 비슷하게 되었을때 과거 값 갱신
                    // 추정 이상하게 되면 날리기


                    /////////////////////////////////////////////// 3D 시선 추정

                    for (int gi = 0; gi < lAllPoints1.Count/10; gi++)
                    {
                        float gpd = 5000; //최소값 저장
                        int gnm = 0; //매칭할 오브젝트 순서

                        for (int gtt = 0; gtt < objectbuffer.Count/11; gtt++) // 나중에 함수로 정리....
                        {

                            if(objectbuffer[(gtt * 11) + 1] !=0 && objectbuffer[(gtt * 11)] + 2 != 0 && objectbuffer[(gtt * 11) + 3] != 0) // 190407 0,0,0 객체는 비교안함
                            { 

                            float dist = (objectbuffer[(gtt * 11) + 1] - lAllPoints1[(gi * 10) + 6]) * (objectbuffer[(gtt * 11) + 1] - lAllPoints1[(gi * 10) + 6]) + (objectbuffer[(gtt * 11) + 2] - lAllPoints1[(gi * 10) + 7]) * (objectbuffer[(gtt * 11) + 2] - lAllPoints1[(gi * 10) + 7]) + (objectbuffer[(gtt * 11) + 3] - lAllPoints1[(gi * 10) + 8]) * (objectbuffer[(gtt * 11) + 3] - lAllPoints1[(gi * 10) + 8]);

                            if (dist < gpd) // 실험적으로 설정필요 => 180605 : 추후 바운딩 박스 크기받아서 선택하는 것도 넣으면 좋을듯 /0.06 => 약 24cm 
                            {
                                gpd = dist; //최소값
                                gnm = gtt; //매칭할 오브젝트 순서
                            }

                            }
                        }
                                                             
                        if(gpd < 0.3) // 0.2 = 44cm  0.3 = 54cm        %190320 : gpd 0.2 -> 0.3
                        {

                            

                            objectbuffer[(gnm * 11) + 8] = 1; //관심객체 여부
                            objectbuffer[(gnm * 11) + 9] = lAllPoints1[(gi * 10) + 9]; //관심 객체 보는 사람 id
                            //objectbuffer[(gnm * 11) + 9] = gi+1; //관심 객체 보는 사람 id



                            // 180605 너무 많이 보내니까 통신 터진다..... 천천히 보내는 방법 찾자.... 아마도 몇번 보팅 되면 보내는 식으로??
                            //long uTimestamp = (long)(DateTime.UtcNow.Subtract(new DateTime(1970, 1, 1))).TotalSeconds; //long time
                            //SendChanget(objectbuffer[(gnm * 11) + 0], objectbuffer[(gnm * 11) + 4], objectbuffer[(gnm * 11) + 8], objectbuffer[(gnm * 11) + 9], objectbuffer[(gnm * 11) + 10], uTimestamp);
                        }


                    }

                    //관심 객체 보내는거

                    long uTimestamp = (long)(DateTime.UtcNow.Subtract(new DateTime(1970, 1, 1))).TotalSeconds; //long time

                    if (uTimestamp - preuTimestamp >= 5) // 5초에 한번씩 통신
                    {

                        for (int ofc = 0; ofc < objectbuffer.Count / 11; ofc++)
                        {
                            if (objectbuffer[(ofc * 11) + 8] == 1)
                            {

                               // SendChanget(objectbuffer[(ofc * 11) + 0], objectbuffer[(ofc * 11) + 4], objectbuffer[(ofc * 11) + 8], objectbuffer[(ofc * 11) + 9], objectbuffer[(ofc * 11) + 10], uTimestamp);

                                preuTimestamp = uTimestamp;
                            }

                        }

                    }


                    /////////////190320 text파일 log 저장 /////////////////

                    
                    for (int ofc = 0; ofc < objectbuffer.Count / 11; ofc++)
                    {
                        if (objectbuffer[(ofc * 11) + 8] == 1)
                        {
                            if(objectbuffer[(ofc * 11) + 0] == 63)
                            {
                                objectbuffer[(ofc * 11) + 0] = 62;
                            }
      
//                            StreamWriter sw = new StreamWriter("C:\\Users\\IT2-123\\Desktop\\testp.txt", true);
//                            sw.WriteLine(string.Format("object id: " + "{0:F}", objectbuffer[(ofc * 11) + 0]) + " xyz: " + string.Format("{0:F}", objectbuffer[(ofc * 11) + 1]) + "/" + string.Format("{0:F}", objectbuffer[(ofc * 11) + 2]) + "/" + string.Format("{0:F}", objectbuffer[(ofc * 11) + 3]) + " time: " + string.Format("{0:F}", objectbuffer[(ofc * 11) + 5]) + "/" + string.Format("{0:F}", objectbuffer[(ofc * 11) + 6]) + "/" + string.Format("{0:F}", objectbuffer[(ofc * 11) + 7]));
//                            sw.Close();

                        }
                    }

                    

                    ///////////////////////////////////////////////////////////


                    ////////반드시 다 여기 들어와야한다



                    lAllCameraPoses.AddRange(oServer.lCameraPoses);
                }

                //Notes the fact that a new frame was downloaded, this is used to estimate the FPS.
                if (oOpenGLWindow != null)
                    oOpenGLWindow.CloudUpdateTick();
            }
        }

        //Performs the ICP based pose refinement.
        private void refineWorker_DoWork(object sender, DoWorkEventArgs e)
        {
            if (oServer.bAllCalibrated == false)
            {
                SetStatusBarOnTimer("Not all of the devices are calibrated.", 5000);
                return;
            }

            //Download a frame from each client.
            List<List<float>> lAllFrameVertices = new List<List<float>>();
            List<List<byte>> lAllFrameColors = new List<List<byte>>();
            List<List<Body>> lAllFrameBody = new List<List<Body>>();
            List<List<float>> lAllFrameFace = new List<List<float>>();
            List<List<float>> lAllFrameObject = new List<List<float>>();
            oServer.GetLatestFrame(lAllFrameColors, lAllFrameVertices, lAllFrameBody, lAllFrameFace, lAllFrameObject);

            //Initialize containers for the poses.
            List<float[]> Rs = new List<float[]>();
            List<float[]> Ts = new List<float[]>();
            for (int i = 0; i < lAllFrameVertices.Count; i++)
            {
                float[] tempR = new float[9];
                float[] tempT = new float[3];
                for (int j = 0; j < 3; j++)
                {
                    tempT[j] = 0;
                    tempR[j + j * 3] = 1;
                }

                Rs.Add(tempR);
                Ts.Add(tempT);
            }

            //Use ICP to refine the sensor poses.
            //This part is explained in more detail in our article (name on top of this file).

            for (int refineIter = 0; refineIter < oSettings.nNumRefineIters; refineIter++)
            {
                for (int i = 0; i < lAllFrameVertices.Count; i++)
                {
                    List<float> otherFramesVertices = new List<float>();
                    for (int j = 0; j < lAllFrameVertices.Count; j++)
                    {
                        if (j == i)
                            continue;
                        otherFramesVertices.AddRange(lAllFrameVertices[j]);
                    }

                    float[] verts1 = otherFramesVertices.ToArray();
                    float[] verts2 = lAllFrameVertices[i].ToArray();

                    IntPtr pVerts1 = Marshal.AllocHGlobal(otherFramesVertices.Count * sizeof(float));
                    IntPtr pVerts2 = Marshal.AllocHGlobal(lAllFrameVertices[i].Count * sizeof(float));

                    Marshal.Copy(verts1, 0, pVerts1, verts1.Length);
                    Marshal.Copy(verts2, 0, pVerts2, verts2.Length);

                    ICP(pVerts1, pVerts2, otherFramesVertices.Count / 3, lAllFrameVertices[i].Count / 3, Rs[i], Ts[i], oSettings.nNumICPIterations);

                    Marshal.Copy(pVerts2, verts2, 0, verts2.Length);
                    lAllFrameVertices[i].Clear();
                    lAllFrameVertices[i].AddRange(verts2);
                }
            }

            //Update the calibration data in client machines.
            List<AffineTransform> worldTransforms = oServer.lWorldTransforms;
            List<AffineTransform> cameraPoses = oServer.lCameraPoses;

            for (int i = 0; i < worldTransforms.Count; i++)
            {
                float[] tempT = new float[3];
                float[,] tempR = new float[3, 3];
                for (int j = 0; j < 3; j++)
                {
                    for (int k = 0; k < 3; k++)
                    {
                        tempT[j] += Ts[i][k] * worldTransforms[i].R[k, j];
                    }

                    worldTransforms[i].t[j] += tempT[j];
                    cameraPoses[i].t[j] += Ts[i][j];
                }

                for (int j = 0; j < 3; j++)
                {
                    for (int k = 0; k < 3; k++)
                    {
                        for (int l = 0; l < 3; l++)
                        {
                            tempR[j, k] += Rs[i][l * 3 + j] * worldTransforms[i].R[l, k];
                        }

                        worldTransforms[i].R[j, k] = tempR[j, k];
                        cameraPoses[i].R[j, k] = tempR[j, k];
                    }
                }
            }

            oServer.lWorldTransforms = worldTransforms;
            oServer.lCameraPoses = cameraPoses;

            oServer.SendCalibrationData();
        }

        private void refineWorker_RunWorkerCompleted(object sender, RunWorkerCompletedEventArgs e)
        {
            //Re-enable all of the buttons after refinement.
            btRefineCalib.Enabled = true;
            btCalibrate.Enabled = true;
            btRecord.Enabled = true;
        }

        //This is used for: starting/stopping the recording worker, stopping the saving worker
        private void btRecord_Click(object sender, EventArgs e)
        {
            if (oServer.nClientCount < 1)
            {
                SetStatusBarOnTimer("At least one client needs to be connected for recording.", 5000);
                return;
            }

            //If we are saving frames right now, this button stops saving.
            if (bSaving)
            {
                btRecord.Enabled = false;
                savingWorker.CancelAsync();
                return;
            }

            bRecording = !bRecording;
            if (bRecording)
            {
                //Stop the update worker to reduce the network usage (provides better synchronization).
                updateWorker.CancelAsync();

                recordingWorker.RunWorkerAsync();
                btRecord.Text = "Stop recording";
                btRefineCalib.Enabled = false;
                btCalibrate.Enabled = false;
            }
            else
            {
                btRecord.Enabled = false;
                recordingWorker.CancelAsync();
            }

        }

        private void btCalibrate_Click(object sender, EventArgs e)
        {
            oServer.Calibrate();
        }

        private void btRefineCalib_Click(object sender, EventArgs e)
        {
            if (oServer.nClientCount < 2)
            {
                SetStatusBarOnTimer("To refine calibration you need at least 2 connected devices.", 5000);
                return;
            }

            btRefineCalib.Enabled = false;
            btCalibrate.Enabled = false;
            btRecord.Enabled = false;

            refineWorker.RunWorkerAsync();
        }

        void RestartUpdateWorker()
        {
            if (!updateWorker.IsBusy)
                updateWorker.RunWorkerAsync();
        }

        private void btShowLive_Click(object sender, EventArgs e)
        {
            RestartUpdateWorker();

            //Opens the live view window if it is not open yet.
            if (!OpenGLWorker.IsBusy)
                OpenGLWorker.RunWorkerAsync();
        }

        private void SetStatusBarOnTimer(string message, int milliseconds)
        {
            statusLabel.Text = message;

            oStatusBarTimer.Stop();
            oStatusBarTimer = new System.Timers.Timer();

            oStatusBarTimer.Interval = milliseconds;
            oStatusBarTimer.Elapsed += delegate (object sender, System.Timers.ElapsedEventArgs e)
            {
                oStatusBarTimer.Stop();
                statusLabel.Text = "";
            };
            oStatusBarTimer.Start();
        }

        //Updates the ListBox contaning the connected clients, called by events inside KinectServer.
        private void UpdateListView(List<KinectSocket> socketList)
        {
            List<string> listBoxItems = new List<string>();

            for (int i = 0; i < socketList.Count; i++)
                listBoxItems.Add(socketList[i].sSocketState);

            lClientListBox.DataSource = listBoxItems;
        }
        /*
        private void UpdateListView_object(List<float> lAllObject)
        {
            List<string> listBoxItems = new List<string>();
            int count = 0;
            string s = "";
            for (int i = 0; i < lAllObject.Count; i++)
            {
                // element to string
                s += lAllObject[i].ToString();
                s += " ";
                if (count+1 % 4 == 0)
                {
                    // string to list item
                    listBoxItems.Add(s);
                    s = "";
                    count++;
                }
            }
            listBox1.DataSource = listBoxItems;

        }
        */
        /*
        private void addElement (List<Single> Object)
        {
            Object.Add(0);// 활성화 플러그
            ListObject.Add(new List<Single>(Object)); //single & float 혼용해서 사용하는거 정리하기

        }
        private int compareList(List<Single> Object, List<List<Single>> ListObject)
        {
            if (ListObject.Count == 0)
            {
                Console.WriteLine("first element");
                return 1;
            }            
            List<Single> ListObject_d = new List<Single>();
            int flag_same_object = 0;
            int flag_same_kinect = 0;
            int flag_diff_time = 0;
            for (int i = 0;i < ListObject.Count; i++)
            {
                ListObject_d = ListObject[i];
                Console.Write(i);
                Console.Write(" : ");
                //리스트의 객체 중 겹치는 것이 없는 경우
                if (ListObject_d[0] == Object[0]) flag_same_object = 1;
                if (ListObject_d[4] == Object[4]) flag_same_kinect = 1;
                if (ListObject_d[5] == Object[5] && ListObject_d[6] == Object[6] && ListObject_d[7] - Object[7] > 1.0) flag_diff_time = 1;

                if (flag_same_object == 1 && flag_same_kinect == 1 && flag_diff_time == 1) //같은 객체, 같은 키넥트, 다른 시간
                {
                    Console.WriteLine("111");
                    return 1; // 이 Object는 새로운 데이터니까 리스트에 추가해야함
                }
                else if (flag_same_object == 0 && flag_same_kinect == 1 && flag_diff_time == 1) // 다른 객체, 같은 키넥트, 다른 시간
                {
                    Console.WriteLine("011");
                    return 1; // 이 Object는 새로운 데이터니까 리스트에 추가해야함
                }
                else if (flag_same_object == 0 && flag_same_kinect == 0) // 다른 객체, 다른 키넥트 일단 등록
                {
                    Console.WriteLine("001");
                    return 1; // 이 Object는 새로운 데이터니까 리스트에 추가해야함
                }
                else
                {
                    Console.WriteLine("no match");
                    flag_same_kinect = 0;
                    flag_same_object = 0;
                    flag_diff_time = 0;                    
                }
            }
            return 0; // 겹치는 것이 있으므로 새로 리스트에 추가할 필요 없음
        }
        private int checkList(List<List<Single>> ListObject)
        {
            int flag_same_object = 0;
            int flag_same_kinect = 0;
            int flag_diff_time = 0;
            List<Single> ListObject_d = new List<Single>();
            List<Single> ListObject_d2 = new List<Single>();

            for (int i = 0; i < ListObject.Count; i++)
            {
                ListObject_d = ListObject[i];
                for (int j = i+1; j < ListObject.Count;j++)
                {
                    ListObject_d2 = ListObject[j];
                    if (ListObject_d[0] == ListObject_d2[0]) flag_same_object = 1;
                    if (ListObject_d[4] == ListObject_d2[4]) flag_same_kinect = 1;
                    //if (ListObject_d[5] == ListObject_d2[5] && ListObject_d[6] == ListObject_d2[6] && (ListObject_d[7] - ListObject_d2[7])>1) flag_diff_time = 1;
                    if (ListObject_d[5] == ListObject_d2[5] && ListObject_d[6] < ListObject_d2[6]) flag_diff_time = 1;

                    if (flag_same_object == 1 && flag_same_kinect == 0 && flag_diff_time == 1)
                    {
                        return j; //활성화 숫자 2
                    }

                    flag_same_kinect = 0;
                    flag_same_object = 0;
                    flag_diff_time = 0;
                }
            }
            return 0;
        }
        private List<int> checkList2(List<List<Single>> ListObject, int hour, int min)  //지금 시간 들어가야됨
        {
            int flag_diff_time = 0;
            List<Single> ListObject_d = new List<Single>();
            List<int> List_toDelete = new List<int>();

            for (int i = 0; i < ListObject.Count; i++)
            {
                ListObject_d = ListObject[i];
                //if (ListObject_d[5] == ListObject_d2[5] && ListObject_d[6] == ListObject_d2[6] && (ListObject_d[7] - ListObject_d2[7])>1) flag_diff_time = 1;
                if (ListObject_d[5] == hour && (min -ListObject_d[6]) > 1) flag_diff_time = 1;

                if (flag_diff_time == 1)
                {
                    List_toDelete.Add(i);
                }

                flag_diff_time = 0;
                
            }
            return List_toDelete;
        }
        */
        /*
        private void updateList(List<List<float>> ObjectList) //리스트 상태값 변경
        {
            //마지막 act값이 있다고 가정 [7]이라고 가정
            //act값 앞의 값이 초만 있다고 가정(1970-01-01부터 계산한 초) [6] 위치
            //time 여기서 불러옴 (localtime이든 뭐든)

            for (int i = 0; i < ObjectList.Count; i++)
            {
                if (ObjectList[6] - time > threshold) //초라고 가정
                    ObjectList[7] = 0;
                    //or ObjectList.removeAt(i);
            }
        }
            */

        /*
        private void deleteElement(List<List<Single>> ListObject, List<int> deleteIndex)
        {
            //pop 함수 필여            
            return;
        }
        */

        private void SendChange(float Objectid, float Kinectid, long time)
        {
             //POST 방식
            string url = "http://155.230.104.191:3001/api/v1/objects";

            StringBuilder dataParams = new StringBuilder();
            dataParams.Append("&createdAt=");
            dataParams.Append(time);
            dataParams.Append("&obj=");
            dataParams.Append(Objectid.ToString());
            dataParams.Append("&objId=");
            dataParams.Append(Objectid.ToString());
            dataParams.Append("&place=");
            dataParams.Append(Kinectid.ToString());
            byte[] byteDataParams = UTF8Encoding.UTF8.GetBytes(dataParams.ToString());

            
            HttpWebRequest request = (HttpWebRequest)WebRequest.Create(url);
            request.Method = "POST";
            request.ContentType = "application/x-www-form-urlencoded";
            request.Timeout = 30 * 1000;
            request.ContentLength = byteDataParams.Length; // 바이트수 지정

            Stream stDataParams = request.GetRequestStream();
            stDataParams.Write(byteDataParams, 0, byteDataParams.Length);
            stDataParams.Close();
            
            HttpWebResponse response = (HttpWebResponse)request.GetResponse();
           
        }


        private void SendChanget(float Objectid, float Kinectid, float Objstate, float Personid, float Place, long time)
        {
            //POST 방식
            string url = "http://155.230.104.191:3001/api/v1/objects";

            StringBuilder dataParams = new StringBuilder();
            dataParams.Append("&createdAt=");
            dataParams.Append(time);
            dataParams.Append("&obj=");
            dataParams.Append(Objectid.ToString());
            dataParams.Append("&objId=");
            dataParams.Append(Objectid.ToString());
            dataParams.Append("&kinectId=");
            dataParams.Append(Kinectid.ToString());
            dataParams.Append("&objState=");
            dataParams.Append(Objstate.ToString());
            dataParams.Append("&personId=");
            dataParams.Append(Personid.ToString());
            dataParams.Append("&place=");
            dataParams.Append(Place.ToString());
            byte[] byteDataParams = UTF8Encoding.UTF8.GetBytes(dataParams.ToString());


            HttpWebRequest request = (HttpWebRequest)WebRequest.Create(url);
            request.Method = "POST";
            request.ContentType = "application/x-www-form-urlencoded";
            request.Timeout = 30 * 1000;
            request.ContentLength = byteDataParams.Length; // 바이트수 지정

            Stream stDataParams = request.GetRequestStream();
            stDataParams.Write(byteDataParams, 0, byteDataParams.Length);
            stDataParams.Close();

            HttpWebResponse response = (HttpWebResponse)request.GetResponse();

        }



        private void SendChangeYame(float Kinectid, long time)
        {
            //POST 방식
            string url = "http://155.230.104.191:3001/api/v1/objects";

            StringBuilder dataParams = new StringBuilder();
            dataParams.Append("&createdAt=");
            dataParams.Append(time);
            dataParams.Append("&obj=remote");
            
            dataParams.Append("&objId=65");
            
            dataParams.Append("&place=");
            dataParams.Append(Kinectid.ToString());
            byte[] byteDataParams = UTF8Encoding.UTF8.GetBytes(dataParams.ToString());


            HttpWebRequest request = (HttpWebRequest)WebRequest.Create(url);
            request.Method = "POST";
            request.ContentType = "application/x-www-form-urlencoded";
            request.Timeout = 30 * 1000;
            request.ContentLength = byteDataParams.Length; // 바이트수 지정

            Stream stDataParams = request.GetRequestStream();
            stDataParams.Write(byteDataParams, 0, byteDataParams.Length);
            stDataParams.Close();

            HttpWebResponse response = (HttpWebResponse)request.GetResponse();

        }

        private void SendPP(List<float> lFramesVerts, List<float> lAllFaces, List<float> Allpoint, int sn)
        {
            List<float> sendpoint = new List<float>(); //포인트 보낼놈

            sendpoint.Clear();




            int espoint = Allpoint.Count / 10;
            int poi = lFramesVerts.Count / 3;


            for (int ei = 0; ei < espoint; ei++)
            {

                float mininum = 10;
                float mininumc = 0;
                int mnh = 0;
                int mnp = 0;


                for (int i1 = 0; i1 < poi; i1++) // 1번키넥트
                {

                    mininumc = (lFramesVerts[(i1 * 3) + 0] - Allpoint[(ei * 10) + 6]) * (lFramesVerts[(i1 * 3) + 0] - Allpoint[(ei * 10) + 6]) + (lFramesVerts[(i1 * 3) + 1] - Allpoint[(ei * 10) + 7]) * (lFramesVerts[(i1 * 3) + 1] - Allpoint[(ei * 10) + 7]) + (lFramesVerts[(i1 * 3) + 2] - Allpoint[(ei * 10) + 8]) * (lFramesVerts[(i1 * 3) + 2] - Allpoint[(ei * 10) + 8]);

                    if (mininum > mininumc) // 18/04/03 : 전체 맵에서 얻은 시선포인트랑 비슷한 로컬포인트만 사용 
                    {
                        mininum = mininumc;
                        mnh = ei;
                        mnp = i1;

                    }

                }

                if (mininum < 0.003) //190409: 0.07(26cm) => 0.003 (5.5cm)
                {
                    sendpoint.Add(Allpoint[(mnh * 10) + 0]);
                    sendpoint.Add(Allpoint[(mnh * 10) + 1]);
                    sendpoint.Add(Allpoint[(mnh * 10) + 2]);
                    sendpoint.Add(Allpoint[(mnh * 10) + 3]);
                    sendpoint.Add(Allpoint[(mnh * 10) + 4]);
                    sendpoint.Add(Allpoint[(mnh * 10) + 5]);
                    sendpoint.Add(lFramesVerts[(mnp * 3) + 0]);
                    sendpoint.Add(lFramesVerts[(mnp * 3) + 1]);
                    sendpoint.Add(lFramesVerts[(mnp * 3) + 2]);
                    sendpoint.Add(Allpoint[(mnh * 10) + 9]);
                }

            }

            if (sendpoint.Count != 0) // 해당하는 포인트 있으면 보냄
            {
                oServer.sendim(sn, sendpoint);
            }

        }


        private void DetecPP(List<float> lAllVertices, List<float> lAllFaces, List<float> lAllPointsz)
        {
            int VC = lAllVertices.Count / 3;  // 포인트 개수
            int FC = lAllFaces.Count / 7;  // 얼굴 개수
            lAllPointsz.Clear();

            float[,] a = new float[FC, 3]; //head point
            float[,] b = new float[FC, 3]; //nose point
            float[,] c = new float[FC, 3]; // 다른 점들
            float[,] ab = new float[FC, 3];
            float[,] bc = new float[FC, 3];
            float[,] pn = new float[FC, 3];
            float[,] cid = new float[FC, 1]; //id

            // float a[6][3], b[6][3], c[6][3], ab[6][3], bc[6][3];
            float t = 0;
            float O_dist = 0;
            // float buff = 0;
            // float O_dist2 = 0;
            // float M_dist = 1000;
            float bc_dist = 0;


            for (int count = 0; count < FC; count++)
            {
                // float M_bc=0.01F;   //10
                float M_bc = 10;   //10

                a[count, 0] = lAllFaces[(count * 7) + 0]; //012=3Dheadpoint
                a[count, 1] = lAllFaces[(count * 7) + 1];
                a[count, 2] = lAllFaces[(count * 7) + 2];
                b[count, 0] = lAllFaces[(count * 7) + 3]; //345=3Dnosepoint
                b[count, 1] = lAllFaces[(count * 7) + 4];
                b[count, 2] = lAllFaces[(count * 7) + 5];
                cid[count, 0] = lAllFaces[(count * 7) + 6]; //id

                ab[count, 0] = b[count, 0] - a[count, 0]; ab[count, 1] = b[count, 1] - a[count, 1]; ab[count, 2] = b[count, 2] - a[count, 2];

                for (int i = 0; i < VC; i++)
                {
                    c[count, 0] = lAllVertices[(i * 3) + 0]; c[count, 1] = lAllVertices[(i * 3) + 1]; c[count, 2] = lAllVertices[(i * 3) + 2];

                    //O_dist2 = sqrtf();

                    t = -((ab[count, +0] * (a[count, 0] - c[count, 0])) + (ab[count, 1] * (a[count, 1] - c[count, 1])) + (ab[count, 2] * (a[count, 2] - c[count, 2]))) / ((ab[count, 0] * ab[count, 0]) + (ab[count, 1] * ab[count, 1]) + (ab[count, 2] * ab[count, 2]));

                    //O_dist = (((ab[0] * t) + a[0] - c[0])*((ab[0] * t) + a[0] - c[0]) + ((ab[1] * t) + a[1] - c[1])*((ab[1] * t) + a[1] - c[1]) + ((ab[2] * t) + a[2] - c[2])*((ab[2] * t) + a[2] - c[2]));
                    //O_dist2 = (((ab[1] * bc[2]) - (ab[2] * bc[1]))*((ab[1] * bc[2]) - (ab[2] * bc[1])) + ((ab[2] * bc[0]) - (ab[0] * bc[2]))*((ab[2] * bc[0]) - (ab[0] * bc[2])) + ((ab[0] * bc[1]) - (ab[1] * bc[0]))*((ab[0] * bc[1]) - (ab[1] * bc[0]))) / ((ab[0] * ab[0]) + (ab[1] * ab[1]) + (ab[2] * ab[2]));
                    if (t > 1) //2.5    %190320 : 2.0 -> 1.0 
                    {
                        O_dist = (((ab[count, 0] * t) + a[count, 0] - c[count, 0]) * ((ab[count, 0] * t) + a[count, 0] - c[count, 0]) + ((ab[count, 1] * t) + a[count, 1] - c[count, 1]) * ((ab[count, 1] * t) + a[count, 1] - c[count, 1]) + ((ab[count, 2] * t) + a[count, 2] - c[count, 2]) * ((ab[count, 2] * t) + a[count, 2] - c[count, 2]));
                        bc[count, 0] = c[count, 0] - b[count, 0]; bc[count, 1] = c[count, 1] - b[count, 1]; bc[count, 2] = c[count, 2] - b[count, 2];
                        bc_dist = bc[count, 0] * bc[count, 0] + bc[count, 1] * bc[count, 1] + bc[count, 2] * bc[count, 2];
                        if ((O_dist < (0.06 * 0.04)) && bc_dist < M_bc) //0.04 * 0.04 or 0.05*0.04 //결국 이 알고리즘은 시선직선의 특정 거리를 만족하는 점들중에 코와 가장 짧은 거리의 값을 찾는다  %190320 : 0.045 * 0.04 -> 0.09 * 0.04 -> 0.06 * 0.04
                        {
                            M_bc = bc_dist;
                            // 여기는 최소 거리 포인트 갱신
                            pn[count, 0] = c[count, 0];
                            pn[count, 1] = c[count, 1];
                            pn[count, 2] = c[count, 2];
                        }
                    }
                }

                // t : 코 포인트와 벗어남 정도?/ O_dist : 시선 직선과 가장 근접한 포인트 / bc_dist : 코와 근접 포인트간의 거리
                // 너무 터무니 없는 값은 버려            
                // 19/04/07 시선 추정좌표가 0, 0, 0 안되게 && pn[count, 0] !=0 && pn[count, 1] != 0 && pn[count, 2] != 0 추가                                 
                if (M_bc < 12 && M_bc > 0.023 && pn[count, 0] != 0 && pn[count, 1] != 0 && pn[count, 2] != 0) //190320 : M_bc < 8 -> M_bc >0.023 (15cm) && M_bc < 12 (3.5m) 
                {
                    lAllPointsz.Add(a[count, 0]); //해당 얼굴중심 데이터
                    lAllPointsz.Add(a[count, 1]);
                    lAllPointsz.Add(a[count, 2]);
                    lAllPointsz.Add(b[count, 0]); //해당 코데이터
                    lAllPointsz.Add(b[count, 1]);
                    lAllPointsz.Add(b[count, 2]);
                    lAllPointsz.Add(pn[count, 0]);// 해당 매칭포인트
                    lAllPointsz.Add(pn[count, 1]);
                    lAllPointsz.Add(pn[count, 2]);
                    lAllPointsz.Add(cid[count, 0]); // 사람 id
                }
            }
            //여기서
        }

    }
}

