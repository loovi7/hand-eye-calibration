

#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "binocularCalibration.h"
#include "handeye.h"
#include <stdio.h>
#include <stdlib.h>

#include <time.h>


#define PI 3.1415926
Mat cameraLeftMatrix1 = Mat::eye(3, 3, CV_64F);
Mat cameraRightMatrix1 = Mat::eye(3, 3, CV_64F);
Mat distLeftCoeffs1 = Mat::zeros(5, 1, CV_64F);//畸变系数（k1,k2,p1,p2,k3）
Mat distRightCoeffs1 = Mat::zeros(5, 1, CV_64F);
Mat cameraMatrixR11 = Mat::zeros(3, 3, CV_64F);
Mat cameraMatrixT1 = Mat::zeros(3, 1, CV_64F);

Mat cameraLeftMatrix2 = Mat::eye(3, 3, CV_64F);
Mat cameraRightMatrix2 = Mat::eye(3, 3, CV_64F);
Mat distLeftCoeffs2 = Mat::zeros(5, 1, CV_64F);
Mat distRightCoeffs2 = Mat::zeros(5, 1, CV_64F);
Mat cameraMatrixR22 = Mat::zeros(3, 3, CV_64F);
Mat cameraMatrixT2 = Mat::zeros(3, 1, CV_64F);

Mat cameraLeftMatrix = Mat::eye(3, 3, CV_64F);
Mat cameraRightMatrix = Mat::eye(3, 3, CV_64F);
Mat distLeftCoeffs = Mat::zeros(5, 1, CV_64F);
Mat distRightCoeffs = Mat::zeros(5, 1, CV_64F);
Mat cameraMatrixR = Mat::zeros(3, 3, CV_64F);
Mat cameraMatrixT = Mat::zeros(3, 1, CV_64F);

Mat cameraMatrixR1 = Mat::zeros(3, 3, CV_64F);
Mat cameraMatrixR2 = Mat::zeros(3, 3, CV_64F);
Mat cameraMatrixP1 = Mat::zeros(3, 4, CV_64F);
Mat cameraMatrixP2 = Mat::zeros(3, 4, CV_64F);
Mat cameraMatrixQ = Mat::zeros(4, 4, CV_64F);



void BinocularCalib::initMatrix() {	//cameraLeft/RightMatrix是相机内参
	//双目标定的矩阵初始化
	cameraLeftMatrix1.at<double>(0, 0) = cameraMatrix[0].at<double>(0, 0);
	cameraLeftMatrix1.at<double>(0, 1) = cameraMatrix[0].at<double>(0, 1);
	cameraLeftMatrix1.at<double>(0, 2) = cameraMatrix[0].at<double>(0, 2);
	cameraLeftMatrix1.at<double>(1, 1) = cameraMatrix[0].at<double>(1, 1);
	cameraLeftMatrix1.at<double>(1, 2) = cameraMatrix[0].at<double>(1, 2);


	cameraRightMatrix1.at<double>(0, 0) = cameraMatrix[1].at<double>(0, 0);
	cameraRightMatrix1.at<double>(0, 1) = cameraMatrix[1].at<double>(0, 1);
	cameraRightMatrix1.at<double>(0, 2) = cameraMatrix[1].at<double>(0, 2);
	cameraRightMatrix1.at<double>(1, 1) = cameraMatrix[1].at<double>(1, 1);
	cameraRightMatrix1.at<double>(1, 2) = cameraMatrix[1].at<double>(1, 2);

	//cout << cameraRightMatrix1 << endl;
	//下面两断是转置吗
	distLeftCoeffs1.at<double>(0, 0) = distCoeffs[0].at<double>(0, 0);
	distLeftCoeffs1.at<double>(1, 0) = distCoeffs[0].at<double>(0, 1);
	distLeftCoeffs1.at<double>(2, 0) = distCoeffs[0].at<double>(0, 2);
	distLeftCoeffs1.at<double>(3, 0) = distCoeffs[0].at<double>(0, 3);
	distLeftCoeffs1.at<double>(4, 0) = distCoeffs[0].at<double>(0, 4);

	//cout << distLeftCoeffs1 << endl;

	distRightCoeffs1.at<double>(0, 0) = distCoeffs[1].at<double>(0, 0);
	distRightCoeffs1.at<double>(1, 0) = distCoeffs[1].at<double>(0, 1);
	distRightCoeffs1.at<double>(2, 0) = distCoeffs[1].at<double>(0, 2);
	distRightCoeffs1.at<double>(3, 0) = distCoeffs[1].at<double>(0, 3);
	distRightCoeffs1.at<double>(4, 0) = distCoeffs[1].at<double>(0, 4);

	//cout << distRightCoeffs1 << endl;

	cameraMatrixR11.at<double>(0, 0) = R.at<double>(0, 0);
	cameraMatrixR11.at<double>(0, 1) = R.at<double>(0, 1);
	cameraMatrixR11.at<double>(0, 2) = R.at<double>(0, 2);
	cameraMatrixR11.at<double>(1, 0) = R.at<double>(1, 0);
	cameraMatrixR11.at<double>(1, 1) = R.at<double>(1, 1);
	cameraMatrixR11.at<double>(1, 2) = R.at<double>(1, 2);
	cameraMatrixR11.at<double>(2, 0) = R.at<double>(2, 0);
	cameraMatrixR11.at<double>(2, 1) = R.at<double>(2, 1);
	cameraMatrixR11.at<double>(2, 2) = R.at<double>(2, 2);

	cout << cameraMatrixR11 << endl;

	cameraMatrixT1.at<double>(0, 0) = T.at<double>(0, 0);
	cameraMatrixT1.at<double>(1, 0) = T.at<double>(1, 0);
	cameraMatrixT1.at<double>(2, 0) = T.at<double>(2, 0);

	cout << cameraMatrixT1 << endl;
}

BinocularCalib::BinocularCalib() {
	displayCorners = false;
	useCalibrated = true;
	showRectified = false;

	boardSize.width = /*9*/9;
	boardSize.height = /*6*/7;
	squareSize = /*1.0*/25;
	//imagelistfn = "C:\\Users\\Admin\\Desktop\\标定程序\\Project1\\Project1\\标定图片\\stereo_calib.xml";
	imagelistfn = "标定图片\\stereo_calib.xml";
//	nums = 15;
//	robot.resize(nums);
//	camera.resize(nums);
	/*distCoeffs[0] = Mat::zeros(1, 5, CV_64F);
	distCoeffs[1] = Mat::zeros(1, 5, CV_64F);*/
}



void BinocularCalib::StereoCalib()
{	
	if (!readStringList(imagelistfn)) {
		cout << "获取图片失败\n";
		return;
	}
	if (imagelist.size() % 2 != 0)
	{
		cout << "Error: the image list contains odd (non-even) number of elements\n";
		return;
	}

	const int maxScale = 1;
	// ARRAY AND VECTOR STORAGE:

	vector<vector<Point2f> > imagePoints[2];
	vector<vector<Point3f> > objectPoints;
	Size imageSize;

	int i, j, k, nimages = (int)imagelist.size() / 2;

	imagePoints[0].resize(nimages);
	imagePoints[1].resize(nimages);
	vector<string> goodImageList;

	for (i = j = 0; i < nimages; i++)
	{
		for (k = 0; k < 2; k++)
		{
			const string& filename = "标定图片\\" + imagelist[i * 2 + k];
			Mat img = imread(filename, 0);
			if (img.empty())
				break;
			if (imageSize == Size())
				imageSize = img.size();
			else if (img.size() != imageSize)
			{
				cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
				break;
			}
			bool found = false;
			
			vector<Point2f>& corners = imagePoints[k][j];
			for (int scale = 1; scale <= maxScale; scale++)
			{
				//Mat timg;
				//if (scale == 1)
				//	timg = img;
				//else
				//	resize(img, timg, Size(), scale, scale, INTER_LINEAR_EXACT);
				found = findChessboardCorners(/*timg*/img, boardSize, corners,
					CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
				if (found)
				{
					if (scale > 1)
					{
						Mat cornersMat(corners);
						cornersMat *= 1. / scale;
					}
					break;
				}
			}
			if (displayCorners)
			{
				cout << filename << endl;
				Mat cimg, cimg1;
				cvtColor(img, cimg, COLOR_GRAY2BGR);
				drawChessboardCorners(cimg, boardSize, corners, found);
				double sf = 640. / MAX(img.rows, img.cols);
				resize(cimg, cimg1, Size(), sf, sf, INTER_LINEAR_EXACT);
				imshow("corners", /*cimg1*/cimg);
				
				char c = (char)waitKey(0);
				if (c == 27 || c == 'q' || c == 'Q') //Allow ESC to quit
					exit(-1);
			}
			else
				putchar('.');
			if (!found)
				break;
			cornerSubPix(img, corners, Size(5, 5), Size(-1, -1),
				TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
					500, 1e-10));
		}
		if (k == 2)
		{
			goodImageList.push_back(imagelist[i * 2]);
			goodImageList.push_back(imagelist[i * 2 + 1]);
			j++;
		}
	}
	
	cout << j << " pairs have been successfully detected.\n";
	nimages = j;
	if (nimages < 2)
	{
		cout << "Error: too little pairs to run the calibration\n";
		return;
	}

	imagePoints[0].resize(nimages);
	imagePoints[1].resize(nimages);
	objectPoints.resize(nimages);

	for (i = 0; i < nimages; i++)
	{
		for (j = 0; j < boardSize.height; j++)
			for (k = 0; k < boardSize.width; k++)
				objectPoints[i].push_back(Point3f(k*squareSize, j*squareSize, 0));
	}

	cout << "Running stereo calibration ...\n";

//	Mat cameraMatrix[2], distCoeffs[2];
	//cout << imageSize;
	cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints[0], imageSize, 0);
	cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints[1], imageSize, 0);
//	Mat R, T, E, F;

	double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
		cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, E, F,
		CALIB_FIX_ASPECT_RATIO +
		CALIB_ZERO_TANGENT_DIST +
		CALIB_USE_INTRINSIC_GUESS +
		/*CALIB_FIX_PRINCIPAL_POINT +*/
		/*CALIB_SAME_FOCAL_LENGTH +*/
		CALIB_RATIONAL_MODEL +
		CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,
		TermCriteria(TermCriteria::COUNT/* + TermCriteria::EPS*/, 500, 1e-20));
	cout << "done with RMS error=" << rms << endl;
	cout << T << endl;
	// CALIBRATION QUALITY CHECK
	// because the output fundamental matrix implicitly
	// includes all the output information,
	// we can check the quality of calibration using the
	// epipolar geometry constraint: m2^t*F*m1=0
	double err = 0;
	int npoints = 0;
	vector<Vec3f> lines[2];
	for (i = 0; i < nimages; i++)
	{
		int npt = (int)imagePoints[0][i].size();
		Mat imgpt[2];
		for (k = 0; k < 2; k++)
		{
			imgpt[k] = Mat(imagePoints[k][i]);
			undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
			computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);
		}
		for (j = 0; j < npt; j++)
		{
			double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
				imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
				fabs(imagePoints[1][i][j].x*lines[0][j][0] +
					imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
			err += errij;
		}
		npoints += npt;
	}
	cout << "average epipolar err = " << err / npoints << endl;

	// save intrinsic parameters
	FileStorage fs("intrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
			"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
		fs.release();
	}
	else
		cout << "Error: can not save the intrinsic parameters\n";

//	Mat R1, R2, P1, P2, Q;
	Rect validRoi[2];

	stereoRectify(cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		/*imageSize*/Size(1280,960), R, T, R1, R2, P1, P2, Q,
		CALIB_ZERO_DISPARITY, 0, /*imageSize*/Size(1280, 960), &validRoi[0], &validRoi[1]);

	fs.open("extrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs.release();
	}
	else
		cout << "Error: can not save the extrinsic parameters\n";

	// OpenCV can handle left-right
	// or up-down camera arrangements
	bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

	// COMPUTE AND DISPLAY RECTIFICATION
	if (!showRectified)
		return;

	Mat rmap[2][2];
	// IF BY CALIBRATED (BOUGUET'S METHOD)
	if (useCalibrated)
	{
		// we already computed everything
	}
	// OR ELSE HARTLEY'S METHOD
	else
		// use intrinsic parameters of each camera, but
		// compute the rectification transformation directly
		// from the fundamental matrix
	{
		vector<Point2f> allimgpt[2];
		for (k = 0; k < 2; k++)
		{
			for (i = 0; i < nimages; i++)
				std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
		}
		F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
		Mat H1, H2;
		stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

		R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
		R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
		P1 = cameraMatrix[0];
		P2 = cameraMatrix[1];
	}

	//Precompute maps for cv::remap()
	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

	Mat canvas;
	double sf;
	int w, h;
	if (!isVerticalStereo)
	{
		sf = 600. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width*sf);
		h = cvRound(imageSize.height*sf);
		canvas.create(h, w * 2, CV_8UC3);
	}
	else
	{
		sf = 300. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width*sf);
		h = cvRound(imageSize.height*sf);
		canvas.create(h * 2, w, CV_8UC3);
	}

	for (i = 0; i < nimages; i++)
	{
		for (k = 0; k < 2; k++)
		{
			Mat img = imread("标定图片\\" + goodImageList[i * 2 + k], 0), rimg, cimg;
			remap(img, rimg, rmap[k][0], rmap[k][1], INTER_LINEAR);
			cvtColor(rimg, cimg, COLOR_GRAY2BGR);
			Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));
			resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
			if (useCalibrated)
			{
				Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
					cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
				rectangle(canvasPart, vroi, Scalar(0, 0, 255), 3, 8);
			}
		}

		if (!isVerticalStereo)
			for (j = 0; j < canvas.rows; j += 16)
				line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
		else
			for (j = 0; j < canvas.cols; j += 16)
				line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
		imshow("rectified", canvas);
		char c = (char)waitKey();
		if (c == 27 || c == 'q' || c == 'Q')
			break;
	}
}


bool BinocularCalib::readStringList(const string& filename)
{
	imagelist.resize(0);//清空这个vector
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	FileNode n = fs.getFirstTopLevelNode();
	if (n.type() != FileNode::SEQ)
		return false;
	FileNodeIterator it = n.begin(), it_end = n.end();
	for (; it != it_end; ++it)
		imagelist.push_back((string)*it);
	return true;
}

/* 从旋转矩阵中得到欧拉角 */
Mat rot2Euler(Mat R) {
	Mat ret = Mat::zeros(3, 1, CV_64F);
	ret.at<double>(0, 0) = atan2(R.at<float>(1, 0), R.at<float>(0, 0)) / PI * 180;
	ret.at<double>(1, 0) = atan2(-1 * R.at<float>(2, 0), sqrt(R.at<float>(2, 1)*R.at<float>(2, 1) + R.at<float>(2, 2)*R.at<float>(2, 2))) / PI * 180;
	ret.at<double>(2, 0) = atan2(R.at<float>(2, 1), R.at<float>(2, 2)) / PI * 180;

	return ret;
}

/* 把*/
Mat RT2Homogeneous(Mat R, Mat T) {
	//cout << R << endl << T << endl;

	Mat ret = Mat::zeros(4, 4, CV_64F);
	
	ret.at<double>(0, 0) = R.at<float>(0, 0);
	ret.at<double>(0, 1) = R.at<float>(0, 1);
	ret.at<double>(0, 2) = R.at<float>(0, 2);
	ret.at<double>(0, 3) = T.at<float>(0, 0)*0.001f;// 这儿乘的0.001是把单位转化成米

	ret.at<double>(1, 0) = R.at<float>(1, 0);
	ret.at<double>(1, 1) = R.at<float>(1, 1);
	ret.at<double>(1, 2) = R.at<float>(1, 2);
	ret.at<double>(1, 3) = T.at<float>(1, 0)*0.001f;

	ret.at<double>(2, 0) = R.at<float>(2, 0);
	ret.at<double>(2, 1) = R.at<float>(2, 1);
	ret.at<double>(2, 2) = R.at<float>(2, 2);
	ret.at<double>(2, 3) = T.at<float>(2, 0)*0.001f;

	ret.at<double>(3, 0) = 0.0;
	ret.at<double>(3, 1) = 0.0;
	ret.at<double>(3, 2) = 0.0;
	ret.at<double>(3, 3) = 1.0;

	return ret;
}

void BinocularCalib::objToCam() {

	char RotationVectors[30];
	char TranslationVectors[30];
	char end_base[30];

	CvMat* rotation_vector = cvCreateMat(3, 1, CV_32FC1);//因为后面罗德里格斯变换要求参数都是CvMat类的指针

	Mat rot, tran, rot1, tran1;

	FileStorage fs("leftVectors.yml", FileStorage::READ);//fs读取旋转矩阵和平移矩阵
	FileStorage End2Base("handEye.yml", FileStorage::READ);//End2Base读取A阵
	FileStorage fs1("rightVectors.yml", FileStorage::READ);
	
	if (!fs.isOpened() || !End2Base.isOpened() || !fs1.isOpened()) {
		std::cout << "Error In Open File" << endl;
		exit(0);
	}
	
	for (int n = 1; n <= 9/*nimages*/; n=n+2) {					
		for (int j = 0; j < 2; j++) {						
			if (j == 0) {
				sprintf(RotationVectors, "RotationVectors%02d", n);
				sprintf(TranslationVectors, "TranslationVectors%02d", n);
				sprintf(end_base, "Matrix%02d", n);
			}
			else {
				sprintf(RotationVectors, "RotationVectors%02d", n+1);
				sprintf(TranslationVectors, "TranslationVectors%02d", n+1);
				sprintf(end_base, "Matrix%02d", n+1);
			}
			//n=1,给RotationVectors TranslationVectors end_base赋一次值
			//std::cout << RotationVectors << "  " << TranslationVectors << "  " << end_base << endl;
			
			fs[RotationVectors] >> rot;
			fs[TranslationVectors] >> tran;
			//cout << rot << endl << tran << endl;
			
			CV_MAT_ELEM(*rotation_vector, float, 0, 0) = rot.at<float>(0, 0);
			CV_MAT_ELEM(*rotation_vector, float, 1, 0) = rot.at<float>(1, 0);
			CV_MAT_ELEM(*rotation_vector, float, 2, 0) = rot.at<float>(2, 0);

			CvMat * rotationMatrix = cvCreateMat(3, 3, CV_32FC1);
			//CvMat * anther_rotationMatrix = cvCreateMat(3, 3, CV_32FC1);
			cvRodrigues2(rotation_vector, rotationMatrix, 0);
			//cvRodrigues2(another_rotation_vector, anther_rotationMatrix, 0);
			/*处理三维旋转问题时，通常采用旋转矩阵的方式来描述。
			一个向量乘以旋转矩阵等价于向量以某种方式进行旋转。
			除了采用旋转矩阵描述外，还可以用旋转向量来描述旋转，
			旋转向量的长度（模）表示绕轴逆时针旋转的角度（弧度）。
			旋转向量与旋转矩阵可以通过罗德里格斯（Rodrigues）变换进行转换。*/
			eye[j] = RT2Homogeneous(cvarrToMat(rotationMatrix), /*cvarrToMat(translation_vector)*/ tran);
			//eye1[j] = RT2Homogeneous(cvarrToMat(anther_rotationMatrix), tran1);
			//cout << RotationVectors << eye[j] << endl;
			//cout << handEye[j].inv() << endl;
		    //cout << handEye[j].inv()*handEye[j] << endl;		
            //std::cout << n << endl << eye[j] << endl;
			//std::cout << Rotation[j] << endl;
			//std::cout << Translation[j] << endl;
			//cout << rot2Euler(Rotation[j]) << endl;


			End2Base[end_base] >> base[j];
			//cout << end_base << base[j] << endl;
		}	
		camera.push_back(eye[1] * eye[0].inv());
		robot.push_back(base[1].inv() * base[0]);//得到14组A阵和C阵
		//camera1.push_back(eye1[1] * eye1[0].inv());
		//robot1.push_back(base1[1].inv() * base1[0]);
		//camera.push_back(eye[1].inv() * eye[0]);
		//robot.push_back(base[1].inv() * base[0]);
	}
	fs.release();
	End2Base.release();
	fs1.release();
}

void BinocularCalib::objToCamOpenCV() {
	
	CvMat *intrinsic, *distortion;
	CvMat* rotation_vector = cvCreateMat(3, 1, CV_32FC1);
	CvMat* translation_vector = cvCreateMat(3, 1, CV_32FC1);
	Mat  D, M;
	static string fileName = "C:/Users/Admin/Desktop/单目标定/Vectors.yml";
	FileStorage fs(fileName, FileStorage::READ);
	if (!fs.isOpened()) {
		cout << "Error In Open File" << endl;
		system("PAUSE");
		exit(0);
	}
	
	if (fs.isOpened())
	{
		fs["IntrinsicMatrix"] >> M;
		fs["Distortion"] >> D;
	}
	
	intrinsic = &((CvMat)M);
	distortion = &((CvMat)D);

	
	char RotationVectors[30];
	char TranslationVectors[30];
	char end_base[30];
	char imageName[30];
	Mat img;
	vector<Point2f> imageCorners;
	vector<Point3f> objectCorners;
	//CvMat* translation_vector = cvCreateMat(3, 1, CV_32FC1);

	for (int n = 1; n <= 30/*nimages*/; n = n + 2) {

		for (int j = 0; j < 2; j++) {
			if (j == 0) {
				//sprintf(RotationVectors, "RotationVectors%02d", n);
				//sprintf(TranslationVectors, "TranslationVectors%02d", n);
				sprintf(end_base, "Matrix%02d", n);
				sprintf(imageName, "C:/Users/Admin/Desktop/单目标定/图像关节角0115/图片/%d_Color.png", n);
			}
			else {
				//sprintf(RotationVectors, "RotationVectors%02d", n + 8);
				//sprintf(TranslationVectors, "TranslationVectors%02d", n + 8);
				sprintf(end_base, "Matrix%02d", n + 1);
				sprintf(imageName, "C:/Users/Admin/Desktop/单目标定/图像关节角0115/图片/%d_Color.png", n + 1);
			}

			//			std::cout << RotationVectors << "  " << TranslationVectors << "  " << end_base << endl;
			cout << imageName << endl;
			img = imread(imageName, 0);
			bool found = cv::findChessboardCorners(img, boardSize, imageCorners);// 计算角点信息
			if (!found) {
				std::cout << "角点未检测到" << endl;
				//	break;
			}
			cv::cornerSubPix(img, imageCorners, cv::Size(11, 11), cv::Size(-1, -1),
				cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.1));// 计算亚像素级别角点信息
			CvMat* image_points = cvCreateMat(imageCorners.size(), 2, CV_32FC1);
			CvMat* object_points = cvCreateMat(imageCorners.size(), 3, CV_32FC1);

			if (imageCorners.size() == boardSize.area()) {
				for (int h = 0; h < boardSize.height; h++) {
					for (int w = 0; w < boardSize.width; w++) {

						CV_MAT_ELEM(*image_points, float, h*boardSize.width + w, 0) = imageCorners[h*boardSize.width + w].x;
						CV_MAT_ELEM(*image_points, float, h*boardSize.width + w, 1) = imageCorners[h*boardSize.width + w].y;

						CV_MAT_ELEM(*object_points, float, h*boardSize.width + w, 0) = h*squareSize;
						CV_MAT_ELEM(*object_points, float, h*boardSize.width + w, 1) = w*squareSize;
						CV_MAT_ELEM(*object_points, float, h*boardSize.width + w, 2) = 0.0f;
					}
				}
			}
			cvFindExtrinsicCameraParams2(object_points, image_points, intrinsic, distortion, rotation_vector, translation_vector);
			CvMat * rotationMatrix = cvCreateMat(3, 3, CV_32FC1);
			cvRodrigues2(rotation_vector, rotationMatrix, 0);
			eye[j] = RT2Homogeneous(cvarrToMat(rotationMatrix), cvarrToMat(translation_vector));
			cout << /*n <<*/ eye[j] << endl;
			//cout << handEye[j].inv() << endl;
			//cout << handEye[j].inv()*handEye[j] << endl;		
			//			std::cout << n << endl << eye[j] << endl;
			//std::cout << Rotation[j] << endl;
			//std::cout << Translation[j] << endl;
			//End2Base[end_base] >> base[j];
			//cout << end_base << base[j] << endl;
		}
		//camera.push_back(eye[1] * eye[0].inv());
		//robot.push_back(base[1].inv() * base[0]);
		//camera.push_back(eye[1].inv() * eye[0]);
		//robot.push_back(base[1].inv() * base[0]);
	}
	//Rot.release();
	//Tran.release();
	//End2Base.release();

	fs.release();
}

int main(int argc, char** argv)
{
	BinocularCalib a;
	//a.StereoCalib();
	//a.initMatrix();
	//a.objToCamOpenCV();
	a.objToCam();
	
	//  五种方法算出的手眼关系
	Mat leftHcg1(4, 4, CV_64FC1);	
	Mat leftHcg2(4, 4, CV_64FC1);
	Mat leftHcg3(4, 4, CV_64FC1);
	Mat leftHcg4(4, 4, CV_64FC1);
	Mat leftHcg5(4, 4, CV_64FC1);

	//  五种方法实现
	Tsai_HandEye(leftHcg1,  a.robot, a.camera);
	DualQ_HandEye(leftHcg2, a.robot, a.camera);
	Inria_HandEye(leftHcg3, a.robot, a.camera);
	Navy_HandEye(leftHcg4, a.robot, a.camera);
	Kron_HandEye(leftHcg5, a.robot, a.camera);

	/*写出标定结果
	  2019.3.1 luowei*/

	FileStorage calibration_result("calibration_result.yml", FileStorage::WRITE);//左臂
	//FileStorage calibration_result1("another_calibration_result.yml", FileStorage::WRITE);//右臂
	if (!calibration_result.isOpened() /*|| !calibration_result1.isOpened() */) {
		cout << "文件创建失败" << endl;
		std::system("PAUSE");
		exit(0);
	}

	/*这里遇到过一个问题，FileStorage对象写入字符串时千万不能含有非法字符
	  仅允许字母、数字、下划线*/

	calibration_result << "Tsai_result" << leftHcg1;
	calibration_result << "DualQ_result" << leftHcg2;
	calibration_result << "Inria_result" << leftHcg3;
	calibration_result << "Navy_result" << leftHcg4;
	calibration_result << "Kron_result" << leftHcg5;
	calibration_result.release();

	/*!!!!!!!一定要记得release()*/
	std::cout <<"\n\n\n\n\n" << leftHcg1 << endl << endl;
	std::cout << leftHcg2 << endl << endl;
	std::cout << leftHcg3 << endl << endl;
	std::cout << leftHcg4 << endl << endl;
	std::cout << leftHcg5 << endl << endl<<"\n\n\n\n\n";
	
	std::system("PAUSE");
	return 0;
	
}
