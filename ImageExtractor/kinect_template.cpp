#include "kinect_template.h"

#define ERROR_CHECK( ret )  \
    if ( (ret) != S_OK ) {    \
        std::stringstream ss;	\
        ss << "failed " #ret " " << std::hex << ret << std::endl;			\
        throw std::runtime_error( ss.str().c_str() );			\
	    }

TKinect2CV::TKinect2CV()
{
}

TKinect2CV::~TKinect2CV()
{
	if (pColorSource != NULL) pColorSource->Release();
	if (pDepthSource != NULL) pDepthSource->Release();
	if (pInfraredSource != NULL) pInfraredSource->Release();
	if (pSensor) pSensor->Close();
	if (pSensor != NULL) pSensor->Release();
}

bool TKinect2CV::init()
{
	HRESULT res;
	int width, height;

	res = GetDefaultKinectSensor(&pSensor);
	if (FAILED(res)) {
		std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
		return false;
	}
	res = pSensor->Open();
	if (FAILED(res)) {
		std::cerr << "Error : IKinectSensor::Open()" << std::endl;
		return false;
	}

	res = pSensor->get_ColorFrameSource(&pColorSource);
	if (FAILED(res)) {
		std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
		return false;
	}
	res = pColorSource->OpenReader(&pColorReader);
	if (FAILED(res)) {
		std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
		return false;
	}
	IFrameDescription* pDescription;
	res = pColorSource->get_FrameDescription(&pDescription);
	if (FAILED(res)) {
		std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;
		return false;
	}
	pDescription->get_Width(&colorWidth);
	pDescription->get_Height(&colorHeight);
	colorbufferSize = colorWidth * colorHeight * 4 * sizeof(unsigned char);
	colorbufferMat.create(colorHeight, colorWidth, CV_8UC4);
	colorMat.create(colorHeight, colorWidth, CV_8UC4);
	colorBuffer.resize(colorWidth * colorHeight * ColorBytesPerPixel);

	res = pSensor->get_DepthFrameSource(&pDepthSource);
	if (FAILED(res)) {
		std::cerr << "Error : IKinectSensor::get_DepthFrameSource()" << std::endl;
		return false;
	}
	res = pDepthSource->OpenReader(&pDepthReader);
	if (FAILED(res)) {
		std::cerr << "Error : IDepthFrameSource::OpenReader()" << std::endl;
		return false;
	}
	res = pDepthSource->get_FrameDescription(&pDescription);
	if (FAILED(res)) {
		std::cerr << "Error : IDepthFrameSource::get_FrameDescription()" << std::endl;
		return false;
	}
	pDescription->get_Width(&depthWidth);
	pDescription->get_Height(&depthHeight);
	depthbufferSize = depthWidth * depthHeight * sizeof(unsigned short);
	depthbufferMat.create(depthHeight, depthWidth, CV_16UC1);
	depthMat.create(depthHeight, depthWidth, CV_8UC1);
	depthBuffer.resize(depthWidth * depthHeight);

	res = pSensor->get_InfraredFrameSource(&pInfraredSource);
	if (FAILED(res)) {
		std::cerr << "Error : IKinectSensor::get_InfraredFrameSource()" << std::endl;
		return false;
	}
	res = pInfraredSource->OpenReader(&pInfraredReader);
	if (FAILED(res)) {
		std::cerr << "Error : IInfraredFrameSource::OpenReader()" << std::endl;
		return false;
	}
	res = pInfraredSource->get_FrameDescription(&pDescription);
	if (FAILED(res)) {
		std::cerr << "Error : IInfraredFrameSource::get_FrameDescription()" << std::endl;
		return false;
	}
	pDescription->get_Width(&width);
	pDescription->get_Height(&height);
	infraredbufferMat.create(height, width, CV_16UC1);
	infraredMat.create(height, width, CV_8UC1);

	res = pSensor->get_BodyFrameSource(&pBodySource);
	if (FAILED(res)) {
		std::cerr << "Error : IKinectSensor::get_InfraredFrameSource()" << std::endl;
		return false;
	}
	res = pBodySource->OpenReader(&pBodyReader);
	if (FAILED(res)) {
		std::cerr << "Error : IInfraredFrameSource::OpenReader()" << std::endl;
		return false;
	}

	for (auto& body : bodies) {
		body = nullptr;
	}

	return true;
}


bool TKinect2CV::queryFrame()
{
	HRESULT res;

	IColorFrame* pColorFrame;
	res = pColorReader->AcquireLatestFrame(&pColorFrame);
	if (SUCCEEDED(res)) {
		res = pColorFrame->CopyConvertedFrameDataToArray(colorbufferSize,
			reinterpret_cast<BYTE*>(colorbufferMat.data), ColorImageFormat::ColorImageFormat_Bgra);
		if (SUCCEEDED(res)) {
			colorbufferMat.copyTo(colorMat);
			cv::flip(colorMat, colorMat, 1);
		}
		res = pColorFrame->CopyConvertedFrameDataToArray(
			colorBuffer.size(), &colorBuffer[0], ColorImageFormat_Bgra);
		if ((res) != S_OK) {
			std::stringstream ss;
			ss << "failed " << std::hex << res << std::endl;
			throw std::runtime_error(ss.str().c_str());
		}
	}
	if (pColorFrame != NULL) pColorFrame->Release();

	IDepthFrame* pDepthFrame;
	res = pDepthReader->AcquireLatestFrame(&pDepthFrame);
	if (SUCCEEDED(res)) {
		res = pDepthFrame->AccessUnderlyingBuffer(&depthbufferSize,
			reinterpret_cast<UINT16**>(&depthbufferMat.data));
		if (SUCCEEDED(res)) {
			//depthbufferMat.convertTo(depthMat, CV_8U, -255.0f / 8000.0f, 255.0f);
			//cv::flip(depthMat, depthMat, 1);
			depthbufferMat.copyTo(depthMat);
		}
		res = pDepthFrame->CopyFrameDataToArray(depthBuffer.size(), &depthBuffer[0]);
		if ((res) != S_OK) {
			std::stringstream ss;
			ss << "failed " << std::hex << res << std::endl;
			throw std::runtime_error(ss.str().c_str());
		}
	}
	if (pDepthFrame != NULL) pDepthFrame->Release();

	IInfraredFrame* pInfraredFrame;
	res = pInfraredReader->AcquireLatestFrame(&pInfraredFrame);
	if (SUCCEEDED(res)) {
		res = pInfraredFrame->AccessUnderlyingBuffer(&infraredbufferSize,
			reinterpret_cast<UINT16**>(&infraredbufferMat.data));
		if (SUCCEEDED(res)) {
			infraredbufferMat.convertTo(infraredMat, CV_8U, 1.0f / 255.0f, 0.0f);
			cv::flip(infraredMat, infraredMat, 1);
		}
	}
	if (pInfraredFrame != NULL) pInfraredFrame->Release();

	IBodyFrame* bodyFrame;
	res = pBodyReader->AcquireLatestFrame(&bodyFrame);
	if (SUCCEEDED(res)) {

		for (auto& body : bodies) {
			if (body != nullptr) {
				body->Release();
				body = nullptr;
			}
		}
		res = bodyFrame->GetAndRefreshBodyData(6, &bodies[0]);
		if (SUCCEEDED(res)) {
		}
	}

	return true;
}

void TKinect2CV::draw() {
	HRESULT res;
	ICoordinateMapper *mapper;
	res = pSensor->get_CoordinateMapper(&mapper);
	if ((res) != S_OK) {
		std::stringstream ss;
		ss << "failed " << std::hex << res << std::endl;
		throw std::runtime_error(ss.str().c_str());
	}
	//ERROR_CHECK(kinect->get_CoordinateMapper(&mapper));

	std::vector<ColorSpacePoint> colorSpacePoints(depthBuffer.size());
	res = mapper->MapDepthFrameToColorSpace(depthBuffer.size(), &depthBuffer[0], colorSpacePoints.size(), &colorSpacePoints[0]);
	if ((res) != S_OK) {
		std::stringstream ss;
		ss << "failed " << std::hex << res << std::endl;
		throw std::runtime_error(ss.str().c_str());
	}
	//ERROR_CHECK(mapper->MapDepthFrameToColorSpace(depthBuffer.size(), &depthBuffer[0],
	//	colorSpacePoints.size(), &colorSpacePoints[0]));

	colorImage.create(depthHeight, depthWidth, CV_8UC4);

	for (int i = 0; i < colorImage.total(); ++i) {
		int x = (int)colorSpacePoints[i].X;
		int y = (int)colorSpacePoints[i].Y;

		int srcIndex = ((y * colorWidth) + x) * ColorBytesPerPixel;
		int destIndex = i * ColorBytesPerPixel;

		if (isValidColorRange(x, y) && isValidDepthRange(i)) {
			colorImage.data[destIndex + 0] = colorBuffer[srcIndex + 0];
			colorImage.data[destIndex + 1] = colorBuffer[srcIndex + 1];
			colorImage.data[destIndex + 2] = colorBuffer[srcIndex + 2];
		}
		else {
			colorImage.data[destIndex + 0] = 255;
			colorImage.data[destIndex + 1] = 255;
			colorImage.data[destIndex + 2] = 255;
		}
	}
	drawBodyIndexFrame();
	//cv::imshow(ColorWindowName, colorImage);
}

void TKinect2CV::drawBodyIndexFrame()
{


	for (auto body : bodies) {
		if (body == nullptr) {
			continue;
		}

		BOOLEAN isTracked = false;
		ERROR_CHECK(body->get_IsTracked(&isTracked));
		if (!isTracked) {
			continue;
		}

		Joint joints[JointType::JointType_Count];
		body->GetJoints(JointType::JointType_Count, joints);
		for (auto joint : joints) {
			if (joint.TrackingState == TrackingState::TrackingState_Tracked) {
				drawEllipse(colorImage, joint, 10, cv::Scalar(255, 0, 0));
			}
			else if (joint.TrackingState == TrackingState::TrackingState_Inferred) {
				drawEllipse(colorImage, joint, 10, cv::Scalar(255, 255, 0));
			}
		}
	}


}

void TKinect2CV::drawEllipse(cv::Mat& bodyImage, const Joint& joint, int r, const cv::Scalar& color)
{
	ICoordinateMapper* mapper;
	ERROR_CHECK(pSensor->get_CoordinateMapper(&mapper));

	DepthSpacePoint point;
	mapper->MapCameraPointToDepthSpace(joint.Position, &point);

	cv::circle(bodyImage, cv::Point(point.X, point.Y), r, color, -1);
}

bool TKinect2CV::isValidColorRange(int x, int y)
{
	return ((0 <= x) && (x < colorWidth)) && ((0 <= y) && (y < colorHeight));
}

bool TKinect2CV::isValidDepthRange(int index)
{
	return (500 <= depthBuffer[index]) && (depthBuffer[index] <= 8000);
}

cv::Mat TKinect2CV::matVideo()
{
	return colorImage;
}

cv::Mat TKinect2CV::matDepth()
{
	return depthMat;
}

cv::Mat TKinect2CV::matInfrared()
{
	return infraredMat;
}

bool TKinect2CV::WriteMatBinary(std::ofstream& ofs, const cv::Mat& out_mat)
{
	if (!ofs.is_open()) {
		return false;
	}
	if (out_mat.empty()) {
		int s = 0;
		ofs.write((const char*)(&s), sizeof(int));
		return true;
	}
	int type = out_mat.type();
	ofs.write((const char*)(&out_mat.rows), sizeof(int));
	ofs.write((const char*)(&out_mat.cols), sizeof(int));
	ofs.write((const char*)(&type), sizeof(int));
	ofs.write((const char*)(out_mat.data), out_mat.elemSize() * out_mat.total());

	return true;
}


//! Save cv::Mat as binary
/*!
\param[in] filename filaname to save
\param[in] output cvmat to save
*/
bool TKinect2CV::SaveMatBinary(const std::string& filename, const cv::Mat& output) {
	std::ofstream ofs(filename, std::ios::binary);
	return WriteMatBinary(ofs, output);
}
