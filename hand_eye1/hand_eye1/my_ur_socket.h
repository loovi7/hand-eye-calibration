#ifndef UR_SOCKET_RECV
#define UR_SOCKET_RECV
#include <stdint.h>
#include <Winsock2.h>
#include <string>
#include <iostream>
#include <sstream>

#include <opencv2\opencv.hpp>
using namespace cv;

//#include <frames.hpp>
//#include <frames_io.hpp>
//#include "filter.hpp"
#pragma comment(lib, "ws2_32.lib")
//#pragma comment(lib, "orocos-kdld.lib")
#define M_PI   3.1415926535
#define M_PI_2 1.5707963268
#define ur3 3
#define ur5 5
#define ur10 10
//using namespace KDL;
using namespace std;
const int fix_len = 1116;
const int force_len = 512;
class ur_socket
{
private:
	//HANDLE m_hMutex;
	//HANDLE m_hRecv;
	SOCKET m_client;
	SOCKET m_client2;
	SOCKADDR_IN client;
	SOCKADDR_IN client2;
	char buf[fix_len];
	char f_buf[force_len];
	string int2str(const float &int_temp);	
	int unpack_16(char buf[], int &offset_len);
	int unpack_32(char buf[], int &offset_len);
	double unpack_64(char buf[], int &offset_len);
	void unpack_vector(char buf[], int &offset_len, double val[]);
	void run_recv_joint();
//	void run_recv_force();
	static void Call_recv_joint(void *_this)
	{
		ur_socket *P = (ur_socket*)_this;
		P->run_recv_joint();
	}
	//static void Call_recv_force(void *_this)
	//{
	//	ur_socket *P = (ur_socket*)_this;
	//	P->run_recv_force();
	//}
public:
	ur_socket(int port, const char* server_address);//接口号 IP地址
	~ur_socket();
	const char * split;
	char  *p, *pNext = NULL;
	int robot_type;
	double rotangle ;//旋转角
	double joint[6];//机械臂关节角度
	double sensor_data[6];//力传感器数据
	double f_sensor_data[6];//滤波后力传感器数据
//	Vector force_base;//机械臂力数据
//	Vector torque_base;//机械臂力矩数据
/*Frame*/ Mat DhToFrame(double alpha_, double a_, double d_, double theta_);
/*Frame*/ Mat forwardkin(int robot_type, double joint_angle[6]);
//	void convert_force(int robot_type,double sensor_data[]);
	
	void get_force(double pos[]);
	void get_joint_pos(double pos[]);
	void get_tcp_pos(double pos[]);
	void get_tcp_vel(double pos[]);
	void get_tcp_force(double pos[]);
	//Mat translation(Mat T);
	//Mat rotZ(double theta);
	//Mat rotX(double theta);
};

#endif // UR_SOCKET_RECV


Mat translation(Mat T);
Mat rotZ(double theta);
Mat rotX(double theta);