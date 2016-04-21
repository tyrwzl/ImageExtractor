#pragma once

#include <Kinect.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <fstream>



class TKinect2CV
{
private:
	IKinectSensor* pSensor;
	IColorFrameSource* pColorSource;
	IDepthFrameSource* pDepthSource;
	IInfraredFrameSource* pInfraredSource;
	IBodyFrameSource* pBodySource;

	IColorFrameReader* pColorReader;
	IDepthFrameReader* pDepthReader;
	IInfraredFrameReader* pInfraredReader;
	IBodyFrameReader* pBodyReader;

	unsigned int colorbufferSize;
	unsigned int depthbufferSize;
	unsigned int infraredbufferSize;

	cv::Mat colorbufferMat, colorMat;
	cv::Mat depthbufferMat, depthMat;
	cv::Mat infraredbufferMat, infraredMat;


	cv::Mat colorImage;

	cv::Mat MatVideo, MatDepth, MatInfrared;


	std::vector<BYTE> colorBuffer;
	std::vector<UINT16> depthBuffer;

	int colorWidth;
	int colorHeight;
	const int ColorBytesPerPixel = 4;

	int depthWidth;
	int depthHeight;
	IBody* bodies[BODY_COUNT];

	//const char* ColorWindowName = "Color Image";
	//const char* DepthWindowName = "Depth Image";


public:
	TKinect2CV();
	~TKinect2CV();
	bool init();
	bool queryFrame();
	void draw();
	bool isValidColorRange(int x, int y);
	bool isValidDepthRange(int index);
	cv::Mat matVideo();
	cv::Mat matDepth();
	cv::Mat matInfrared();
	bool WriteMatBinary(std::ofstream& ofs, const cv::Mat& out_mat);
	bool SaveMatBinary(const std::string& filename, const cv::Mat& output);
	void TKinect2CV::drawBodyIndexFrame();
	void TKinect2CV::drawEllipse(cv::Mat& bodyImage, const Joint& joint, int r, const cv::Scalar& color);
	cv::Mat bodyImage = cv::Mat::zeros(424, 512, CV_8UC4);;
};
