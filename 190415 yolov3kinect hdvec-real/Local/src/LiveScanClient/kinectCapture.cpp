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
#include "KinectCapture.h"
#include <chrono>





DWORD c_FaceFrameFeatures =
FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace
| FaceFrameFeatures::FaceFrameFeatures_PointsInColorSpace
| FaceFrameFeatures::FaceFrameFeatures_RotationOrientation
| FaceFrameFeatures::FaceFrameFeatures_Happy
| FaceFrameFeatures::FaceFrameFeatures_RightEyeClosed
| FaceFrameFeatures::FaceFrameFeatures_LeftEyeClosed
| FaceFrameFeatures::FaceFrameFeatures_MouthOpen
| FaceFrameFeatures::FaceFrameFeatures_MouthMoved
| FaceFrameFeatures::FaceFrameFeatures_LookingAway
| FaceFrameFeatures::FaceFrameFeatures_Glasses
| FaceFrameFeatures::FaceFrameFeatures_FaceEngagement;




KinectCapture::KinectCapture()
{
	pKinectSensor = NULL;
	pCoordinateMapper = NULL;
	pMultiSourceFrameReader = NULL;


////////////////////

	for (int i = 0; i < BODY_COUNT; i++)
	{
		m_pFaceFrameSources[i] = nullptr;
		m_pFaceFrameReaders[i] = nullptr;
		m_phdFaceFrameSources[i] = nullptr;
		m_phdFaceFrameReaders[i] = nullptr;
		m_phdFaceAlignments[i] = nullptr;
	}

}
////////////////////


KinectCapture::~KinectCapture()
{
	SafeRelease(pKinectSensor);
	SafeRelease(pCoordinateMapper);
	SafeRelease(pMultiSourceFrameReader);

////////////////////

	for (int i = 0; i < BODY_COUNT; i++)
	{
		SafeRelease(m_pFaceFrameSources[i]);
		SafeRelease(m_pFaceFrameReaders[i]);
		SafeRelease(m_phdFaceFrameSources[i]);
		SafeRelease(m_phdFaceFrameReaders[i]);
		SafeRelease(m_phdFaceAlignments[i]);

	}
////////////////////

}

bool KinectCapture::Initialize()
{
	HRESULT hr;

	hr = GetDefaultKinectSensor(&pKinectSensor);
	if (FAILED(hr))
	{
		bInitialized = false;
		return bInitialized;
	}

	if (pKinectSensor)
	{
		pKinectSensor->get_CoordinateMapper(&pCoordinateMapper);
		hr = pKinectSensor->Open();

		if (SUCCEEDED(hr))
		{
			pKinectSensor->OpenMultiSourceFrameReader(FrameSourceTypes::FrameSourceTypes_Color |
				FrameSourceTypes::FrameSourceTypes_Depth |
				FrameSourceTypes::FrameSourceTypes_Body |
				FrameSourceTypes::FrameSourceTypes_BodyIndex,
				&pMultiSourceFrameReader);

			////////////////////

			if (SUCCEEDED(hr))
			{
				// create a face frame source + reader to track each body in the fov
				for (int i = 0; i < BODY_COUNT; i++)
				{
					
					if (SUCCEEDED(hr))
					{
						// create the face frame source by specifying the required face frame features
						hr = CreateFaceFrameSource(pKinectSensor, 0, c_FaceFrameFeatures, &m_pFaceFrameSources[i]);
					}
					if (SUCCEEDED(hr))
					{
						// open the corresponding reader
						hr = m_pFaceFrameSources[i]->OpenReader(&m_pFaceFrameReaders[i]);
					}
					

///////////////////////////////////////hd /////////////////////////////////////
					if (SUCCEEDED(hr))
					{
						// create the face frame source by specifying the required face frame features
						hr = CreateHighDefinitionFaceFrameSource(pKinectSensor, &m_phdFaceFrameSources[i]);
					}
					if (SUCCEEDED(hr))
					{
						// open the corresponding reader
						hr = m_phdFaceFrameSources[i]->OpenReader(&m_phdFaceFrameReaders[i]);
					}


					
					if (SUCCEEDED(hr))
					{
						// create the face frame alignment
						hr = CreateFaceAlignment(&m_phdFaceAlignments[i]);
					}
					
//////////////////////////////////////////////////////////////////////////////////////

				}
			}


		}
	}

	bInitialized = SUCCEEDED(hr);

	if (bInitialized)
	{
		std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
		bool bTemp;
		do
		{
			bTemp = AcquireFrame();
			
			std::chrono::duration<double> elapsedSeconds = std::chrono::system_clock::now() - start;
			if (elapsedSeconds.count() > 5.0)
			{
				bInitialized = false;
				break;
			}

		} while (!bTemp);
	}

	return bInitialized;
}

bool KinectCapture::AcquireFrame()
{
	if (!bInitialized)
	{
		return false;
	}


	IMultiSourceFrame* pMultiFrame = NULL;
	HRESULT hr = pMultiSourceFrameReader->AcquireLatestFrame(&pMultiFrame);

	if (!SUCCEEDED(hr))
	{
		return false;
	}




	GetDepthFrame(pMultiFrame);
	GetColorFrame(pMultiFrame);
	GetBodyFrame(pMultiFrame);
	GetBodyIndexFrame(pMultiFrame);


	SafeRelease(pMultiFrame);


	return true;
}


void KinectCapture::MapDepthFrameToCameraSpace(Point3f *pCameraSpacePoints)
{
	pCoordinateMapper->MapDepthFrameToCameraSpace(nDepthFrameWidth * nDepthFrameHeight, pDepth, nDepthFrameWidth * nDepthFrameHeight, (CameraSpacePoint*)pCameraSpacePoints);
}

void KinectCapture::MapColorFrameToCameraSpace(Point3f *pCameraSpacePoints)
{
	pCoordinateMapper->MapColorFrameToCameraSpace(nDepthFrameWidth * nDepthFrameHeight, pDepth, nColorFrameWidth * nColorFrameHeight, (CameraSpacePoint*)pCameraSpacePoints);
}

void KinectCapture::MapDepthFrameToColorSpace(Point2f *pColorSpacePoints)
{
	pCoordinateMapper->MapDepthFrameToColorSpace(nDepthFrameWidth * nDepthFrameHeight, pDepth, nDepthFrameWidth * nDepthFrameHeight, (ColorSpacePoint*)pColorSpacePoints);
}

void KinectCapture::MapColorFrameToDepthSpace(Point2f *pDepthSpacePoints)
{
	pCoordinateMapper->MapColorFrameToDepthSpace(nDepthFrameWidth * nDepthFrameHeight, pDepth, nColorFrameWidth * nColorFrameHeight, (DepthSpacePoint*)pDepthSpacePoints);;
}

void KinectCapture::GetDepthFrame(IMultiSourceFrame* pMultiFrame)
{
	IDepthFrameReference* pDepthFrameReference = NULL;
	IDepthFrame* pDepthFrame = NULL;
	pMultiFrame->get_DepthFrameReference(&pDepthFrameReference);
	HRESULT hr = pDepthFrameReference->AcquireFrame(&pDepthFrame);

	if (SUCCEEDED(hr))
	{
		////////////////////
		unsigned int sz;
		unsigned short* buf;
		pDepthFrame->AccessUnderlyingBuffer(&sz, &buf);
		pCoordinateMapper->MapDepthFrameToCameraSpace(sz, buf, 217088, depth2xyz);
		pCoordinateMapper->MapDepthFrameToColorSpace(sz, buf, 217088, depth2rgb);  //18.05.07 : MapColorFrameToDepthSpace 써야할수도...
	    ////////////////////

		if (pDepth == NULL)
		{
			IFrameDescription* pFrameDescription = NULL;
			hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
			pFrameDescription->get_Width(&nDepthFrameWidth);
			pFrameDescription->get_Height(&nDepthFrameHeight);
			pDepth = new UINT16[nDepthFrameHeight * nDepthFrameWidth];
			SafeRelease(pFrameDescription);
		}

		UINT nBufferSize = nDepthFrameHeight * nDepthFrameWidth;
		hr = pDepthFrame->CopyFrameDataToArray(nBufferSize, pDepth);
	}

	SafeRelease(pDepthFrame);
	SafeRelease(pDepthFrameReference);
}

void KinectCapture::GetColorFrame(IMultiSourceFrame* pMultiFrame)
{
	IColorFrameReference* pColorFrameReference = NULL;
	IColorFrame* pColorFrame = NULL;
	pMultiFrame->get_ColorFrameReference(&pColorFrameReference);
	HRESULT hr = pColorFrameReference->AcquireFrame(&pColorFrame);

	if (SUCCEEDED(hr))
	{
		if (pColorRGBX == NULL)
		{
			IFrameDescription* pFrameDescription = NULL;
			hr = pColorFrame->get_FrameDescription(&pFrameDescription);
			hr = pFrameDescription->get_Width(&nColorFrameWidth);
			hr = pFrameDescription->get_Height(&nColorFrameHeight);
			pColorRGBX = new RGB[nColorFrameWidth * nColorFrameHeight];
			SafeRelease(pFrameDescription);
		}

		UINT nBufferSize = nColorFrameWidth * nColorFrameHeight * sizeof(RGB);
		hr = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(pColorRGBX), ColorImageFormat_Bgra);
	}

	SafeRelease(pColorFrame);
	SafeRelease(pColorFrameReference);
}

void KinectCapture::GetBodyFrame(IMultiSourceFrame* pMultiFrame)
{
	IBodyFrameReference* pBodyFrameReference = NULL; //if (m_pBodyFrameReader != nullptr)
	IBodyFrame* pBodyFrame = NULL;
	pMultiFrame->get_BodyFrameReference(&pBodyFrameReference);
	HRESULT hr = pBodyFrameReference->AcquireFrame(&pBodyFrame); // hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);


	IBody* bodies[BODY_COUNT] = { NULL };
	////////////////////
	IFaceFrameSource* FaceFrameSource = nullptr;
	IFaceFrame* faceFrame = nullptr;
	IFaceFrameResult* faceResult = nullptr;

	for (int i = 0; i < 6; i++)
		for (int j = 0; j < 10; j++)
			FaceData[i][j] = 0; //초기화
	////////////////////
	if (SUCCEEDED(hr))
	{

		hr = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, bodies);

	}
	//SafeRelease(pBodyFrame);



	bool bHaveBodyData = SUCCEEDED(hr);



	vBodies = std::vector<Body>(BODY_COUNT);
	for (int i = 0; i < BODY_COUNT; i++)
	{



		//////////////////// hd /////////////////////////////

		IHighDefinitionFaceFrame* phdFaceFrame = nullptr;
		hr = m_phdFaceFrameReaders[i]->AcquireLatestFrame(&phdFaceFrame);

		BOOLEAN bFaceTracked = false;

		if (SUCCEEDED(hr) && nullptr != phdFaceFrame)
		{

			hr = phdFaceFrame->get_IsTrackingIdValid(&bFaceTracked);

		}

		if (bFaceTracked)
		{

			IFaceAlignment* pFaceAlignment = nullptr;


			//HR(CreateFaceAlignment(&pFaceAlignment));
			hr = CreateFaceAlignment(&pFaceAlignment);

			if (SUCCEEDED(hr))
			{
				hr = phdFaceFrame->GetAndRefreshFaceAlignmentResult(pFaceAlignment);
			}

			if (SUCCEEDED(hr))
			{
				//HR(phdFaceFrame->GetAndRefreshFaceAlignmentResult(pFaceAlignment));

				//we have a hd face frame so get the vertices							
				//hr = phdFaceFrame->GetAndRefreshFaceAlignmentResult(m_phdFaceAlignments[i]);

				IFaceModel * pFaceModel = nullptr;

				//we have updated the faceAlignment results
				hr = phdFaceFrame->get_FaceModel(&pFaceModel);
				if (SUCCEEDED(hr) && nullptr != pFaceModel)
				{
					UINT32 vertexCount = 0;
					hr = GetFaceModelVertexCount(&vertexCount);
					if (SUCCEEDED(hr))
					{

						//CameraSpacePoint * pFacePoints = new CameraSpacePoint[vertexCount];
						//hr = pFaceModel->CalculateVerticesForAlignment(m_phdFaceAlignments[iFace], vertexCount, pFacePoints);

						std::vector<CameraSpacePoint> vertexs(vertexCount);


						//hr = pFaceModel->CalculateVerticesForAlignment(m_phdFaceAlignments[i], vertexCount, &vertexs[0]);
						hr = pFaceModel->CalculateVerticesForAlignment(pFaceAlignment, vertexCount, &vertexs[0]);

						if (SUCCEEDED(hr) && vertexs[HighDetailFacePoints_NoseTip].X < 10 && vertexs[HighDetailFacePoints_NoseTip].Y < 10 && vertexs[HighDetailFacePoints_NoseTip].Z < 10)   // 쓰레기 값 버리는 조건문 필요
						{


							FaceData[i][0] = 1;

							FaceData[i][3] = (int)((vertexs[HighDetailFacePoints_NoseTip].X) * 1000 + 0.5);
							FaceData[i][4] = (int)((vertexs[HighDetailFacePoints_NoseTip].Y) * 1000 + 0.5);
							FaceData[i][5] = (int)((vertexs[HighDetailFacePoints_NoseTip].Z) * 1000 + 0.5);

							FaceData[i][6] = (int)(((vertexs[HighDetailFacePoints_ChinCenter].X + vertexs[HighDetailFacePoints_ForeheadCenter].X) / 2) * 1000 + 0.5);
							FaceData[i][7] = (int)(((vertexs[HighDetailFacePoints_ChinCenter].Y + vertexs[HighDetailFacePoints_ForeheadCenter].Y) / 2) * 1000 + 0.5);
							FaceData[i][8] = (int)(((vertexs[HighDetailFacePoints_ChinCenter].Z + vertexs[HighDetailFacePoints_ForeheadCenter].Z) / 2) * 1000 + 0.5);




						}


					}
				}
				SafeRelease(pFaceModel);
			}
		}
		else
		{

			/////////////////////////////////////////////


			if (bodies[i])
			{
				Joint joints[JointType_Count];
				BOOLEAN isTracked;
				BOOLEAN FaceTracked;

				bodies[i]->get_IsTracked(&isTracked);
				bodies[i]->GetJoints(JointType_Count, joints);

				vBodies[i].vJoints.assign(joints, joints + JointType_Count);

				vBodies[i].vJointsInColorSpace.resize(JointType_Count);

				//FaceData[i][6]=(int)((joints[JointType_Head].Position.X)*1000+0.5);
				//FaceData[i][7] = (int)((joints[JointType_Head].Position.Y) * 1000 + 0.5);
				//FaceData[i][8] = (int)((joints[JointType_Head].Position.Z) * 1000 + 0.5);

				//hxx=(int)((joints[JointType_Head].Position.X)*1000+0.5);
				//hyy = (int)((joints[JointType_Head].Position.Y) * 1000 + 0.5);
				//hzz = (int)((joints[JointType_Head].Position.Z) * 1000 + 0.5);

				for (int j = 0; j < JointType_Count; j++)
				{
					ColorSpacePoint tempPoint;
					pCoordinateMapper->MapCameraPointToColorSpace(joints[j].Position, &tempPoint);
					vBodies[i].vJointsInColorSpace[j].X = tempPoint.X;
					vBodies[i].vJointsInColorSpace[j].Y = tempPoint.Y;
				}





				if (isTracked == TRUE)
				{
					vBodies[i].bTracked = true;
					//face
					hr = bodies[i]->get_TrackingId(&TrackingID);
					if (SUCCEEDED(hr))
					{
						
						//////////////////////////////////////////////hd///////////////////////////////////////////////
						hr = m_phdFaceFrameSources[i]->put_TrackingId(TrackingID);
						if (!SUCCEEDED(hr)) continue;
						////////////////////////////////////////////////////////////////////////////////////////////////
						
						
						
						/////////////////////////////////////////////////////////////hd///////////////////////////////////////////////////////////////////////

						//let's see if we can get hd face frame here
						// retrieve the latest face frame from this reader
						IHighDefinitionFaceFrame* phdFaceFrame = nullptr;
						hr = m_phdFaceFrameReaders[i]->AcquireLatestFrame(&phdFaceFrame);
						if (SUCCEEDED(hr) && nullptr != phdFaceFrame)
						{
							//we have a hd face frame so get the vertices							
							hr = phdFaceFrame->GetAndRefreshFaceAlignmentResult(m_phdFaceAlignments[i]);

							IFaceModel * pFaceModel = nullptr;
							if (SUCCEEDED(hr))
							{
								//we have updated the faceAlignment results
								hr = phdFaceFrame->get_FaceModel(&pFaceModel);
								if (SUCCEEDED(hr) && nullptr != pFaceModel)
								{
									UINT32 vertexCount = 0;
									hr = GetFaceModelVertexCount(&vertexCount);
									if (SUCCEEDED(hr))
									{

										//CameraSpacePoint * pFacePoints = new CameraSpacePoint[vertexCount];
										//hr = pFaceModel->CalculateVerticesForAlignment(m_phdFaceAlignments[iFace], vertexCount, pFacePoints);

										std::vector<CameraSpacePoint> vertexs(vertexCount);


										hr = pFaceModel->CalculateVerticesForAlignment(m_phdFaceAlignments[i], vertexCount, &vertexs[0]);


										if (SUCCEEDED(hr) && vertexs[HighDetailFacePoints_NoseTip].X < 10 && vertexs[HighDetailFacePoints_NoseTip].Y < 10 && vertexs[HighDetailFacePoints_NoseTip].Z < 10)   // 쓰레기 값 버리는 조건문 필요
										{


											FaceData[i][0] = 1;

											FaceData[i][3] = (int)((vertexs[HighDetailFacePoints_NoseTip].X) * 1000 + 0.5);
											FaceData[i][4] = (int)((vertexs[HighDetailFacePoints_NoseTip].Y) * 1000 + 0.5);
											FaceData[i][5] = (int)((vertexs[HighDetailFacePoints_NoseTip].Z) * 1000 + 0.5);

											FaceData[i][6] = (int)(((vertexs[HighDetailFacePoints_ChinCenter].X + vertexs[HighDetailFacePoints_ForeheadCenter].X) / 2) * 1000 + 0.5);
											FaceData[i][7] = (int)(((vertexs[HighDetailFacePoints_ChinCenter].Y + vertexs[HighDetailFacePoints_ForeheadCenter].Y) / 2) * 1000 + 0.5);
											FaceData[i][8] = (int)(((vertexs[HighDetailFacePoints_ChinCenter].Z + vertexs[HighDetailFacePoints_ForeheadCenter].Z) / 2) * 1000 + 0.5);




											//							CameraSpacePoint headPivot;
											//							m_phdFaceAlignments[i]->get_HeadPivotPoint(&headPivot);

											//						    FaceData[i][6] = (int)((headPivot.X) * 1000 + 0.5);
											//							FaceData[i][7] = (int)((headPivot.Y) * 1000 + 0.5);
											//							FaceData[i][8] = (int)((headPivot.Z) * 1000 + 0.5);
											//delete& headPivot;

										}

										//delete pFacePoints;

									}
								}

							}

							SafeRelease(pFaceModel);


						}

						SafeRelease(phdFaceFrame);


						
						
						
						
						
						
						
						
						hr = m_pFaceFrameReaders[i]->get_FaceFrameSource(&FaceFrameSource);
						if (!SUCCEEDED(hr)) continue;

						hr = FaceFrameSource->put_TrackingId(TrackingID);
						if (!SUCCEEDED(hr)) continue;

						hr = m_pFaceFrameReaders[i]->AcquireLatestFrame(&faceFrame);
						if (!SUCCEEDED(hr)) continue;

						hr = faceFrame->get_IsTrackingIdValid(&FaceTracked);
						if (!SUCCEEDED(hr)) continue;

						hr = faceFrame->get_FaceFrameResult(&faceResult);
						if (!SUCCEEDED(hr)) continue;

						if (faceResult)
						{

							//얼굴박스 치기

							hr = faceResult->get_FaceBoundingBoxInColorSpace(&facebox); // 얼굴검출 박스 치기
							if (SUCCEEDED(hr))
							{
								FaceBox[i][0] = facebox.Bottom;
								FaceBox[i][1] = facebox.Left;
								FaceBox[i][2] = facebox.Right;
								FaceBox[i][3] = facebox.Top;
							}



							PointF pointst[FacePointType::FacePointType_Count];
							hr = faceResult->GetFacePointsInColorSpace(FacePointType::FacePointType_Count, pointst);

							FaceData[i][1] = pointst[FacePointType::FacePointType_Nose].X * 10 + 0.5;
							FaceData[i][2] = pointst[FacePointType::FacePointType_Nose].Y * 10 + 0.5;



						}
						/////////////////////////////////////////////////////////////hd///////////////////////////////////////////////////////////////////////

														//let's see if we can get hd face frame here
														// retrieve the latest face frame from this reader


										//delete pFacePoints;

					}

				}
				else
				{
					vBodies[i].bTracked = false;
				}

			}
		}




		SafeRelease(phdFaceFrame);

	}

	SafeRelease(pBodyFrame);
	SafeRelease(pBodyFrameReference);

	SafeRelease(FaceFrameSource);
	SafeRelease(faceFrame);
	SafeRelease(faceResult);


}











	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


							   //190411 : 기존 코 찾는 방법 주석처리
								/*
								PointF points[FacePointType::FacePointType_Count];
								hr = faceResult->GetFacePointsInColorSpace(FacePointType::FacePointType_Count, points);

								FaceData[i][1] = points[FacePointType::FacePointType_Nose].X * 10 + 0.5;
								FaceData[i][2] = points[FacePointType::FacePointType_Nose].Y * 10 + 0.5;
								if (!FaceData[i][1] == 0)
								{
									FaceData[i][0] = 1;

									ColorSpacePoint* p = depth2rgb;
									float nose_x, nose_y;// , eye_x, eye_y;
									float nose_ditc;// , eye_ditc;
									//float nose_R2Ddic = 10;
									float nose_R2Ddic = 4;

									for (int m = 0; m < nDepthFrameHeight; m++)
									{
										for (int n = 0; n < nDepthFrameWidth; n++)
										{
											if (p->X < 0 || p->Y < 0 || p->X > nColorFrameWidth || p->Y > nColorFrameHeight);
											else
											{
												nose_x = p->X - points[FacePointType::FacePointType_Nose].X;
												nose_y = p->Y - points[FacePointType::FacePointType_Nose].Y;
												nose_ditc = sqrtf((nose_x*nose_x) + (nose_y*nose_y));
												if (nose_ditc < nose_R2Ddic)
												{
													nose_R2Ddic = nose_ditc;
													//FaceData[i][3] = depth2xyz[m*nDepthFrameWidth + n].X * 1000 + 0.5;
													//FaceData[i][4] = depth2xyz[m*nDepthFrameWidth + n].Y * 1000 + 0.5; //코데이터 더 잘찾아야 한다;;
													//FaceData[i][5] = depth2xyz[m*nDepthFrameWidth + n].Z * 1000 + 0.5; // 18.05.07 요 kinect capture 부분이 핵심일지도 몰것네
												}
											}
											// Don't copy alpha channel
											p++;
										}
									}



								}

								*/




				



void KinectCapture::GetBodyIndexFrame(IMultiSourceFrame* pMultiFrame)
{
	IBodyIndexFrameReference* pBodyIndexFrameReference = NULL;
	IBodyIndexFrame* pBodyIndexFrame = NULL;
	pMultiFrame->get_BodyIndexFrameReference(&pBodyIndexFrameReference);
	HRESULT hr = pBodyIndexFrameReference->AcquireFrame(&pBodyIndexFrame);


	if (SUCCEEDED(hr))
	{
		if (pBodyIndex == NULL)
		{
			pBodyIndex = new BYTE[nDepthFrameHeight * nDepthFrameWidth];
		}

		UINT nBufferSize = nDepthFrameHeight * nDepthFrameWidth;
		hr = pBodyIndexFrame->CopyFrameDataToArray(nBufferSize, pBodyIndex);
	}

	SafeRelease(pBodyIndexFrame);
	SafeRelease(pBodyIndexFrameReference);
}


