#include <iostream>
#include <sstream>
#include <stdint.h>

#include <Kinect.h>
#include <opencv2/opencv.hpp>

#include <atltime.h>

#include "kinect_template.h"

// ERROR_CHECK
#define ERROR_CHECK(ret) \
	if ((ret) != S_OK) { \
		std::stringstream ss; \
		ss << "failed " #ret " " << std::hex << ret << std::endl; \
		throw std::runtime_error( ss.str().c_str() ); \
	}

/* 動作チェック
void main()
{
	try {
		IKinectSensor* kinect = nullptr;
		ERROR_CHECK(::GetDefaultKinectSensor(&kinect));

		ERROR_CHECK(kinect->Open());

		BOOLEAN isOpen = false;
		ERROR_CHECK(kinect->get_IsOpen(&isOpen));
		std::cout << "kinect is " << (isOpen ? "Open" : "Not Open") << std::endl;

		::Sleep(3000);

		kinect->Close();
		kinect->Release();
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}
}
*/

std::string GetNowTime(void)
{
	CTime current_time = CTime::GetCurrentTime();
	CStringA year_month_day = current_time.Format(_T("%Y%m%d"));

	const size_t size = (year_month_day.GetLength() + 1);
	char *ret = new char[size];
	strcpy_s(ret, size, year_month_day);

	return ret;
}

// BGR order in OpenCV
uint8_t GetPixelFromColorMat(cv::Mat& ColorMat, uint16_t row_i, uint16_t col_i, uint8_t rgb)
{
	return ColorMat.data[ColorMat.step[0] * row_i + ColorMat.step[1] * col_i + rgb];

}

bool CheckEndKinectStudio(cv::Mat& ColorMat)
{
	uint32_t sum_rgb = 0;
	for (uint32_t row_i = 0; row_i < 424; ++row_i) {
		for (uint32_t col_i = 0; col_i < 512; ++col_i) {
			uint8_t b = GetPixelFromColorMat(ColorMat, row_i, col_i, 0);
			uint8_t g = GetPixelFromColorMat(ColorMat, row_i, col_i, 1);
			uint8_t r = GetPixelFromColorMat(ColorMat, row_i, col_i, 2);
			sum_rgb += GetPixelFromColorMat(ColorMat, row_i, col_i, 0) + GetPixelFromColorMat(ColorMat, row_i, col_i, 1) + GetPixelFromColorMat(ColorMat, row_i, col_i, 2);
		}
	}
	if (sum_rgb > 250 * 3 * 512 * 424) 
		return true;
	else 
		return false;
}


int main()
{
	std::string filename = "20160221_080519_00\\";
	TKinect2CV kinect2;

	std::string s;
	int number = 1;



	if (!kinect2.init()) {
		std::cerr << "init failed" << std::endl;
		return -1;
	}


	std::string ColorWindowName = "color";
	cv::namedWindow(ColorWindowName);

	//cv::setMouseCallback(ColorWindowName, mouseCallback, 0);
	//cv::namedWindow("Color");
	//cv::namedWindow("Depth");
	//cv::namedWindow("Infrared");

	clock_t t = clock();

	while (1) {
		kinect2.queryFrame();
		kinect2.draw();
		if (clock() - t < 3000) continue;

		if (CheckEndKinectStudio(kinect2.matVideo())) 
			break;

		//CTime ctime = CTime::GetCurrentTime();
		//CStringA str = ctime.Format(_T("%Y%m%d%H%M%S"));
		//
		//const size_t newsizea = (str.GetLength() + 1);
		//char *nstringa = new char[newsizea];
		//strcpy_s(nstringa, newsizea, str);
		//std::string s = nstringa;


		s = filename;// + GetNowTime();

		std::stringstream ss;
		ss << number;
		s += ss.str();

		//std::cout << s << " (char *)" << std::endl;
		//s = "../../../../../../../onedrive/5body/" + s;
		//cv::imshow("Color", kinect2.matVideo());


		cv::imshow(ColorWindowName, kinect2.matVideo());
		//::imshow("Body Image", kinect2.bodyImage);


		//cv::imshow("Depth", kinect2.matDepth());

		if (!cv::imwrite(s + ".bmp", kinect2.matDepth()))
			std::cout << "error!" << std::endl;
		cv::imwrite(s + "infraed.bmp", kinect2.matInfrared());
		cv::imwrite(s + "color.bmp", kinect2.matVideo());
		kinect2.SaveMatBinary(s, kinect2.matDepth());

		//cv::imshow("Infrared", kinect2.matInfrared());


		if (cv::waitKey(200) == 'q') break;
		++number;
	}

	return 0;
}