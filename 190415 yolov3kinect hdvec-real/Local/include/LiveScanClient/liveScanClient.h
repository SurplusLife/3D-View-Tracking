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
#pragma once

#include "resource.h"
#include "ImageRenderer.h"
#include "SocketCS.h"
#include "calibration.h"
#include "utils.h"
#include "KinectCapture.h"
#include "frameFileWriterReader.h"
#include <thread>
#include <mutex>
#include "filter.h"

class LiveScanClient
{
public:
    LiveScanClient();
    ~LiveScanClient();


    static LRESULT CALLBACK MessageRouter(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
    LRESULT CALLBACK        DlgProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
    int                     Run(HINSTANCE hInstance, int nCmdShow);

	bool m_bSocketThread;
private:
	Calibration calibration;

	bool m_bCalibrate;
	bool m_bFilter;
	bool m_bStreamOnlyBodies;

	ICapture *pCapture;

	int m_nFilterNeighbors;
	float m_fFilterThreshold;

	bool m_bCaptureFrame;
	bool m_bConnected;
	bool m_bConfirmCaptured;
	bool m_bConfirmCalibrated;
	bool m_bShowDepth;
	bool m_bFrameCompression;
	int m_iCompressionLevel;

	FrameFileWriterReader m_framesFileWriterReader;

	//////////////////////
	Point3f pview[10];      //뷰 포인트 저장 /180410 : 추후 배열을 동적할당으로 다 바꿔야됨 / 180414 : 일단 이 local의 최대 뷰포인트 출력은 10개
	int hunum = 0;        // 180414 : 서버로부터 받은 시선 추정 사람 수
	//////////////////////
	float facd[6][10]; // 전역 헤드 포즈
	/////////////////////
	float obid[30][9]; // 18.05.07 : 최대 30개 , 오브젝트 여부 , 오브젝트 id, 오브젝트 3D 좌표(x,y,z) , 키넥트id, 시간(시, 분, 초)
	float viid[30][7]; // 18.05.30 : 2차원 정보 저장 이미지 x좌표, 이미지 y좌표, 이미지 너비, 이미지 너비,오브젝트 아이디, 오브젝트 확률, 뷰포인트 최소 점 파라미터 갱신
	/////////////////////
	cv::Mat *frames;
	/////////////////////
	int pcheck = 0;    // 뷰데이터 송수신 체크
    /////////////////
	int pcheckcnt = 0;
	/////////////////

	SocketClient *m_pClientSocket;
	std::vector<float> m_vBounds;

	std::vector<Point3s> m_vLastFrameVertices;
	std::vector<RGB> m_vLastFrameRGB;
	std::vector<Body> m_vLastFrameBody;
	std::vector<Point2i> m_vLastFrameimp;

	HWND m_hWnd;
    INT64 m_nLastCounter;
    double m_fFreq;
    INT64 m_nNextStatusTime;
    DWORD m_nFramesSinceUpdate;	

	Point3f* m_pCameraSpaceCoordinates;
	Point2f* m_pColorCoordinatesOfDepth;
	Point2f* m_pDepthCoordinatesOfColor;

    // Direct2D
    ImageRenderer* m_pDrawColor;
    ID2D1Factory* m_pD2DFactory;
	RGB* m_pDepthRGBX;

	void UpdateFrame();
    void ProcessColor(RGB* pBuffer, int nWidth, int nHeight);
	void ProcessDepth(const UINT16* pBuffer, int nHeight, int nWidth);

    bool SetStatusMessage(_In_z_ WCHAR* szMessage, DWORD nShowTimeMsec, bool bForce);

	void HandleSocket();
	void SendFrame(vector<Point3s> vertices, vector<RGB> RGB, vector<Body> body);

	void SocketThreadFunction();
	void StoreFrame(Point3f *vertices, Point2f *mapping, RGB *color, vector<Body> &bodies, BYTE* bodyIndex);
	void ShowFPS();
	void ReadIPFromFile();
	void WriteIPToFile();
};

