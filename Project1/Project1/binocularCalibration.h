#pragma once

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

using namespace cv;
using namespace std;

class BinocularCalib {
private:
	string imagelistfn;
	vector<string> imagelist;
	Size boardSize;
	float squareSize;
	bool displayCorners;
	bool useCalibrated;
	bool showRectified;
	Mat cameraMatrix[2], distCoeffs[2];
	Mat handEyeR[2],  handEyeT[2];
	Mat R, T, E, F;
	Mat R1, R2, P1, P2, Q;
	Mat Rotation[2], Translation[2];
	Mat eye[2];
	Mat eye1[2];
//	int nums;


public:
	vector<Mat> camera, robot, camera1, robot1;
	Mat  base[2];
	Mat base1[2];
	BinocularCalib();
	void StereoCalib();
	bool readStringList(const string& filename/*, vector<string>& l*/);
	void initMatrix();
	void objToCam();
	void objToCamOpenCV();

};



