#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/improc.hpp>

int main(int argc, char **argv){
	(void)argc;
	(void)argv;

	std::vector<cv::String> fileNames;
	cv::glob("/CalibrationImages/Image*.png", fileNames, False);
	cv::Size patternSize(25 - 1, 18 - 1);
	std::vector<std::vector<cv:Point2f>> q(fileNames.size());
	std::vector<std::vector<cv::Point3f>> Q;
	// generate checkerboard in world coordinates Q, board is 25*18
	// field size 15*15mm

	int checkerBoard[2]  = {25,18};
	int fieldSize = 15;
	// Define world coordinates for 3d points

	std::vector<cv::Point3f> objp;
	for(int i=1; i<checkerBoard[1]; i++){
		for(int j = 1; j<checkerBoard[0]; j++){
			objp.push_back(cv::Point3f(j*fieldSize,i*fieldSize,0));
		}
	}

	std::vector<cv::Point2f> imPoint;
	// Detecht feature points
	std::size_t i = 0;
	for (auto const &f : fileNames){
		std::cout << std::string(f) << std::endl;

		// 2. Read in the image an call cv::findChessboardCorners()
		cv::Mat img = cv::imread(fileNames[i]);
		cv::Mat gray;
		cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);

		bool patternFound = cv::findChessboardCorners(gray, patternSize, q[i], cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);

		//use cv::cornerSubPix() for fine corner detection
		if(patternFound){
			cv::cornerSubPix(gray, q[i], cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
			Q.push_back(objp);
		}
		// Display
		cv::drawChessboardCorners(img, patternSize, q[i], patternFound);
		cv::imshow("chessboard detection", img);
		cv::waitKey(0);

		i++;
	}
	// intrinsic camera matrix
	cv::Matx33f K(cv::Matx33f::eye());
	// distortion coefficients
	cv::Vec<float, 5> k(0,0,0,0);

	std::vector<cv::Mat> rvecs, tvecs;
	std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
	int flags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 + cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_FIX_PRINCIPAL_POINT;
	cv::Size frameSize(1440,1080);

	std::cout << "Calibrating..." << std::endl;
	// call fload error

	float error = cv::calibrateCamera(Q, q, frameSize, K, k, rvecs, tvecs, flags);

	std::cout << "Reprojection error = " << error << "\nK = \n" << K << "nk = \n" << k << std::endl;

	// Precompute correction interpolation
	cv::Mat mapX, mapY;
	cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, framSize, CV_32FC1, mapX, mapY);

	// show corrected images
	for (auto const &f : fileNames){
		std::cout << std::string(f) << std::endl;

		cv::Mat img = cv::imread(f, cv::IMREAD_COLOR);

		cv::Mat imgUndistorted;
		// remap images using interpolation map
		cv::remap(img, imgUndistorted, mapX, mapY, cv::INTER_LINEAR);
		cv::waitKey(0);
	}

	return 0;

}