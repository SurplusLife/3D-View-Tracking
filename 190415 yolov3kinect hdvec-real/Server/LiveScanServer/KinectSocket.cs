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
using System.Net.Sockets;


namespace KinectServer
{
    public delegate void SocketChangedHandler();

    public class KinectSocket
    {
        Socket oSocket;
        byte[] byteToSend = new byte[1];
        public bool bFrameCaptured = false;
        public bool bLatestFrameReceived = false;
        public bool bStoredFrameReceived = false;
        public bool bNoMoreStoredFrames = true;
        public bool bCalibrated = false;
        //The pose of the sensor in the scene (used by the OpenGLWindow to show the sensor)
        public AffineTransform oCameraPose = new AffineTransform();
        //The transform that maps the vertices in the sensor coordinate system to the world corrdinate system.
        public AffineTransform oWorldTransform = new AffineTransform();

        public string sSocketState;

        public List<byte> lFrameRGB = new List<byte>();
        public List<Single> lFrameVerts = new List<Single>();
        public List<Body> lBodies = new List<Body>();
        public List<Single> lFaces = new List<Single>();   //얼굴 데이터 저장
        public List<Single> lObject = new List<Single>();   //물체 데이터 저장

        public event SocketChangedHandler eChanged;

        public KinectSocket(Socket clientSocket)
        {
            oSocket = clientSocket;
            sSocketState = oSocket.RemoteEndPoint.ToString() + " Calibrated = false";
        }

        public void CaptureFrame()
        {
            bFrameCaptured = false;
            byteToSend[0] = 0;
            SendByte();
        }

        public void Calibrate()
        {
            bCalibrated = false;
            sSocketState = oSocket.RemoteEndPoint.ToString() + " Calibrated = false";

            byteToSend[0] = 1;
            SendByte();

            UpdateSocketState();
        }

        public void SendSettings(KinectSettings settings)
        {
            List<byte> lData = settings.ToByteList();

            byte[] bTemp = BitConverter.GetBytes(lData.Count);
            lData.InsertRange(0, bTemp);
            lData.Insert(0, 2);

            if (SocketConnected())
                oSocket.Send(lData.ToArray());
        }

        public void RequestStoredFrame()
        {
            byteToSend[0] = 3;
            SendByte();
            bNoMoreStoredFrames = false;
            bStoredFrameReceived = false;
        }

        public void RequestLastFrame()
        {
            byteToSend[0] = 4;
            SendByte();
            bLatestFrameReceived = false;
        }

        public void SendCalibrationData()
        {
            int size = 1 + (9 + 3) * sizeof(float);
            byte[] data = new byte[size];
            int i = 0;

            data[i] = 5;
            i++;

            Buffer.BlockCopy(oWorldTransform.R, 0, data, i, 9 * sizeof(float));
            i += 9 * sizeof(float);
            Buffer.BlockCopy(oWorldTransform.t, 0, data, i, 3 * sizeof(float));
            i += 3 * sizeof(float);

            if (SocketConnected())
                oSocket.Send(data);
        }

        public void ClearStoredFrames()
        {
            byteToSend[0] = 6;
            SendByte();
        }

        public void ReceiveCalibrationData()
        {
            bCalibrated = true;

            byte[] buffer = Receive(sizeof(int) * 1);
            //currently not used
            int markerId = BitConverter.ToInt32(buffer, 0);

            buffer = Receive(sizeof(float) * 9);
            Buffer.BlockCopy(buffer, 0, oWorldTransform.R, 0, sizeof(float) * 9);

            buffer = Receive(sizeof(float) * 3);
            Buffer.BlockCopy(buffer, 0, oWorldTransform.t, 0, sizeof(float) * 3);

            oCameraPose.R = oWorldTransform.R;
            for (int i = 0; i < 3; i++)
            {
                oCameraPose.t[i] = 0.0f;
                for (int j = 0; j < 3; j++)
                {
                    oCameraPose.t[i] += oWorldTransform.t[j] * oWorldTransform.R[i, j];
                }
            }

            UpdateSocketState();
        }

        public void ReceiveFrame()
        {
            lFrameRGB.Clear();
            lFrameVerts.Clear();
            lBodies.Clear();
            lFaces.Clear();
            lObject.Clear();


            int nToRead;
            byte[] buffer = new byte[1024];

            while (oSocket.Available == 0)
            {
                if (!SocketConnected())
                    return;
            }

            oSocket.Receive(buffer, 8, SocketFlags.None);
            nToRead = BitConverter.ToInt32(buffer, 0);
            int iCompressed = BitConverter.ToInt32(buffer, 4);

            if (nToRead == -1)
            {
                bNoMoreStoredFrames = true;
                return;
            }

            buffer = new byte[nToRead];
            int nAlreadyRead = 0;

            while (nAlreadyRead != nToRead)
            {
                while (oSocket.Available == 0)
                {
                    if (!SocketConnected())
                        return;
                }

                nAlreadyRead += oSocket.Receive(buffer, nAlreadyRead, nToRead - nAlreadyRead, SocketFlags.None);
            }




            if (iCompressed == 1)
                buffer = ZSTDDecompressor.Decompress(buffer);

            //Receive depth and color data
            int startIdx = 0;

            int n_vertices = BitConverter.ToInt32(buffer, startIdx);
            startIdx += 4;

            for (int i = 0; i < n_vertices; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    lFrameRGB.Add(buffer[startIdx++]);
                }
                for (int j = 0; j < 3; j++)
                {
                    float val = BitConverter.ToInt16(buffer, startIdx);
                    //converting from milimeters to meters
                    val /= 1000.0f;
                    lFrameVerts.Add(val);
                    startIdx += 2;
                }
            }
            /*
            //Receive body data
            int nBodies = BitConverter.ToInt32(buffer, startIdx);
            startIdx += 4;
            for (int i = 0; i < nBodies; i++)
            {
                Body tempBody = new Body();
                tempBody.bTracked = BitConverter.ToBoolean(buffer, startIdx++);
                int nJoints = BitConverter.ToInt32(buffer, startIdx);
                startIdx += 4;

                tempBody.lJoints = new List<Joint>(nJoints);
                tempBody.lJointsInColorSpace = new List<Point2f>(nJoints);

                for (int j = 0; j < nJoints; j++)
                {
                    Joint tempJoint = new Joint();
                    Point2f tempPoint = new Point2f();

                    tempJoint.jointType = (JointType)BitConverter.ToInt32(buffer, startIdx);
                    startIdx += 4;
                    tempJoint.trackingState = (TrackingState)BitConverter.ToInt32(buffer, startIdx);
                    startIdx += 4;
                    tempJoint.position.X = BitConverter.ToSingle(buffer, startIdx);
                    startIdx += 4;
                    tempJoint.position.Y = BitConverter.ToSingle(buffer, startIdx);
                    startIdx += 4;
                    tempJoint.position.Z = BitConverter.ToSingle(buffer, startIdx);
                    startIdx += 4;

                    tempPoint.X = BitConverter.ToSingle(buffer, startIdx);
                    startIdx += 4;
                    tempPoint.Y = BitConverter.ToSingle(buffer, startIdx);
                    startIdx += 4;

                    tempBody.lJoints.Add(tempJoint);
                    tempBody.lJointsInColorSpace.Add(tempPoint);
                }

                lBodies.Add(tempBody);
            }
            */
            /////////////////////////// 통신//////////////////////////////
            // list.count 필요? face list 만들기!

            int nFaces = BitConverter.ToInt32(buffer, startIdx);
            startIdx += 4;


            for (int i = 0; i < nFaces; i++)
            {
                float headx = BitConverter.ToSingle(buffer, startIdx);
                startIdx += 4;
                float heady = BitConverter.ToSingle(buffer, startIdx);
                startIdx += 4;
                float headz = BitConverter.ToSingle(buffer, startIdx);
                startIdx += 4;
                float nosex = BitConverter.ToSingle(buffer, startIdx);
                startIdx += 4;
                float nosey = BitConverter.ToSingle(buffer, startIdx);
                startIdx += 4;
                float nosez = BitConverter.ToSingle(buffer, startIdx);
                startIdx += 4;
                float pid = BitConverter.ToSingle(buffer, startIdx);
                startIdx += 4;
                //        headx /= 1000.0f;
                //       heady /= 1000.0f;
                //       headz /= 1000.0f;

                //               nosex /= 1000.0f;
                //              nosey /= 1000.0f;
                //             nosez /= 1000.0f;



                lFaces.Add(headx);
                lFaces.Add(heady);
                lFaces.Add(headz);
                lFaces.Add(nosex);
                lFaces.Add(nosey);
                lFaces.Add(nosez);
                lFaces.Add(pid);
            }
            /////////////////////////////////////////////////////////////

            // object 정보

            int nob = BitConverter.ToInt32(buffer, startIdx);
            startIdx += 4;


            for (int i = 0; i < nob; i++)
            {
                float obid = BitConverter.ToSingle(buffer, startIdx); //오브젝트 ID
                startIdx += 4;
                float obx = BitConverter.ToSingle(buffer, startIdx); //오브젝트 X좌표
                startIdx += 4;
                float oby = BitConverter.ToSingle(buffer, startIdx); //오브젝트 Y좌표
                startIdx += 4;
                float obz = BitConverter.ToSingle(buffer, startIdx); //오브젝트 Z좌표
                startIdx += 4;
                float kiid = BitConverter.ToSingle(buffer, startIdx); //키넥트 ID 좌표
                startIdx += 4;
                float hou = BitConverter.ToSingle(buffer, startIdx); //시
                startIdx += 4;
                float mi = BitConverter.ToSingle(buffer, startIdx); //분
                startIdx += 4;
                float se = BitConverter.ToSingle(buffer, startIdx); //초
                startIdx += 4;

                lObject.Add(obid);
                lObject.Add(obx);
                lObject.Add(oby);
                lObject.Add(obz);
                lObject.Add(kiid);
                lObject.Add(hou);
                lObject.Add(mi);
                lObject.Add(se);
            }

///////////////////////////////////////////////////////////////////////////////////////////////

        }

        public byte[] Receive(int nBytes)
        {
            byte[] buffer;
            if (oSocket.Available != 0)
            {
                buffer = new byte[Math.Min(nBytes, oSocket.Available)];
                oSocket.Receive(buffer, nBytes, SocketFlags.None);
            }
            else
                buffer = new byte[0];

            return buffer;
        }

        public bool SocketConnected()
        {
            bool part1 = oSocket.Poll(1000, SelectMode.SelectRead);
            bool part2 = (oSocket.Available == 0);

            if (part1 && part2)
            {
                return false;
            }
            else
            {
                return true;
            }
        }

        private void SendByte()
        {
            oSocket.Send(byteToSend);
        }

        public void UpdateSocketState()
        {
            if (bCalibrated)
                sSocketState = oSocket.RemoteEndPoint.ToString() + " Calibrated = true";
            else
                sSocketState = oSocket.RemoteEndPoint.ToString() + " Calibrated = false";

            if (eChanged != null)
                eChanged();
        }


        public void Sendimp(int t, List<float> lPoint)
        {
            int size = 1 + ((4 * t) +1) * sizeof(float);
            byte[] data = new byte[size];
            int i = 0;
            data[i] = 7; // 좌표보내겠다 의미
            i++;

            float[] np = new float[1];


            //머리 개수
            np[0] = t;

            Buffer.BlockCopy(np, 0, data, i, 1 * sizeof(float));
            i += 1 * sizeof(float);


            for (int k = 0; k < t; k++) //좌표 쏘기
            {

                float[] hp = new float[4];
                hp[0] = lPoint[10 * k + 6];
                hp[1] = lPoint[10 * k + 7];
                hp[2] = lPoint[10 * k + 8];
                hp[3] = lPoint[10 * k + 9];

                Buffer.BlockCopy(hp, 0, data, i, 4 * sizeof(float));
                i += 4 * sizeof(float);

             }

            if (SocketConnected())
                oSocket.Send(data);


        }






}
}
