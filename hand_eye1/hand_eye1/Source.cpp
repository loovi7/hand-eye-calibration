#include "my_ur_socket.h"
#include "ur_socket.h"
//#include "HalconCpp.h"
#include <opencv2/opencv.hpp>

#define PI 3.1415926


void move_ur3() {
	SocketThread b(30003, "192.168.1.108");
	ur_socket a(30003, "192.168.1.108");
	FileStorage fs("handEye.yml", FileStorage::WRITE);
	FileStorage Joint("joint.yml", FileStorage::READ);
	if (!fs.isOpened() || !Joint.isOpened()) {
		std::cout << "error in open file" << endl;
		std::system("PAUSE");
		exit(0);
	}
	char jointName[30];
	char matrixName[30];
	double angel[6];
	for (int i = 1; i <= 30; i++) {
		
		for (int j = 0; j < 6; j++) {
			sprintf(jointName, "%s%02d%s%d", "pos", i, "_angel_", j);
			Joint[jointName] >> angel[j];
			//cout << angel[j] << "  " ;
		}
		//cout << endl;

		b.movej(angel[0], angel[1], angel[2], angel[3], angel[4], angel[5], 1.6, 0.08);
		Sleep(7000);
		b.stopl(0.5);
		sprintf(matrixName, "%s%02d", "Matrix", i);
		fs << matrixName << a.forwardkin(3, angel);
		cout << a.forwardkin(3, angel) << endl;
	}
	Joint.release();
	fs.release();
}



int main() {

	char input;
	char matrixName[30];
	char jointAngel[30];
	
	ur_socket a(30003, "192.168.30.16");
	SocketThread b(30003, "192.168.30.16");
	/*
	ur_socket aa(30003, "192.168.1.107");
	SocketThread bb(30003, "192.168.1.107");
	*/

	FileStorage fs("handEye.yml", FileStorage::WRITE);
	FileStorage fJoint("joint.yml", FileStorage::WRITE);


	//cv::VideoCapture capl(2);
	//cv::VideoCapture capr(0);
	//int i = 1;
	//cv::Mat src_imgl, src_imgr;
	//char filename_l[15];
	//char filename_r[15];
	
	if (!fs.isOpened() || !fJoint.isOpened()) {
		cout << "读写文件创建失败" << endl;
		std::system("PAUSE");
		exit(0);
	}

	//while (1){
	//	//
	//	//cv::imshow("src_imgl", src_imgl);
	//	//cv::imshow("src_imgr", src_imgr);
	//	//char c = cv::waitKey(1);
	//	//
	//	//
	//	if (c == ' ') //按空格采集图像
	//	{
	//		sprintf(filename_l, "image/left%d.jpg", i);
	//		imwrite(filename_l, src_imgl);
	//		sprintf(filename_r, "image/right%d.jpg", i);
	//		imwrite(filename_r, src_imgr);



	for(int i=1; i<21; i++){
		char c;
		cin >> c;

		if(c == 's'){

			double angel[6];
			double tcp_pos[6];
			double tcp_vel[6];
			a.get_joint_pos(angel);
			a.get_tcp_pos(tcp_pos);
			a.get_tcp_vel(tcp_vel);
			for (int j = 0; j < 6; j++) {
				cout << angel[j] << "   ";
				sprintf(jointAngel, "%s%02d%s%d", "rightpos", i, "_angel_", j);
				fJoint << jointAngel << angel[j];
			}
			cout << "" << endl;
			for (int j = 0; j < 6; j++) {
				cout << tcp_pos[j] << "   ";
			}
			cout << "" << endl;
			for (int j = 0; j < 6; j++) {
				cout << tcp_vel[j] << "   ";
			}
			cout << "" << endl;
			sprintf(matrixName, "%s%02d", "Matrix", i);
			cout << i << endl;
			fs << matrixName << a.forwardkin(3, angel);
			cout << a.forwardkin(3, angel) << endl;
			/*
			aa.get_joint_pos(angel);
			for (int j = 0; j < 6; j++) {
				//cout << angel[i] << "   ";
				sprintf(jointAngel, "%s%02d%s%d", "leftpos", i, "_angel_", j);
				fJoint << jointAngel << angel[j];
			}

			sprintf(matrixName, "%s%02d", "leftMatrix", i);
			fs << matrixName << a.forwardkin(3, angel);
			cout << aa.forwardkin(3, angel) << endl;
			*/
			
		}
		if (c == 'q' || c == 'Q') // 按q退出
		{
			
			break;
		}
	}


//	for (int i = 1; i <= 30; i++) {
//		cin >> input;
//		sprintf(matrixName, "%s%02d", "rightMatrix", i);
//
//		if (input == 'q') {
//
////			b.send_urscript("gripper_open.txt");
//
//			double angel[6];
//			a.get_joint_pos(angel);
//
//			for (int j = 0; j < 6; j++) {
//				//cout << angel[i] << "   ";
//				sprintf(jointAngel, "%s%02d%s%d", "rightpos",i, "_angel_", j);
//				fJoint << jointAngel << angel[j];
//			}
//
//			cout << i <<endl;
//			fs << matrixName << a.forwardkin(3, angel);
//			cout << a.forwardkin(3, angel) << endl;
//
//			aa.get_joint_pos(angel);
//			for (int j = 0; j < 6; j++) {
//				//cout << angel[i] << "   ";
//				sprintf(jointAngel, "%s%02d%s%d", "leftpos", i, "_angel_", j);
//				fJoint << jointAngel << angel[j];
//			}
//
//			sprintf(matrixName, "%s%02d", "leftMatrix", i);
//			fs << matrixName << a.forwardkin(3, angel);
//			cout << aa.forwardkin(3, angel) << endl;
//			
//
//
////
//////			b.movej(-1.05753, -1.16798, -2.1493, 0.216585, 0.967809, -0.031477, 1.6, 0.08);
//////			waitKey(500);
//
//		}
//		if (input == 'x') {
////			b.send_urscript("gripper_close.txt");
//			b.stopl(1.3);
//			break;
//		}
//
//	}
		
	fs.release();
	fJoint.release();

//	move_ur10();
//	std::system("PAUSE");
	return 1;
	
}