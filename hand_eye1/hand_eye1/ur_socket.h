#ifndef UR_SOCKET
#define UR_SOCKET
#include <stdio.h>
#include <string>
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <WINSOCK2.H>
#include <iostream>
#include <sstream>
#include <fstream>  
#pragma comment(lib, "ws2_32.lib")
using namespace std;
class SocketThread
{
private:
	u_short PORT;//端口
	const char* SERVER_ADDRESS;//ip地址

public:
	explicit SocketThread(int port,const char* server_adress);
	string int2str(const float &int_temp);
	SOCKET client_steup();
	SOCKET server_steup();
	void send_urscript(string  str);//发送指令
	void send_urscript(char * fileName);
	void resv_urscript();
	void recv_tcp_pose();//接受末端位姿
	void movej(float q1, float q2, float q3, float q4, float q5, float q6, float a, float v);
	void speedj(float v1, float v2, float v3, float v4, float v5, float v6, float a, float t);
	void speedl(float v1, float v2, float v3, float v4, float v5, float v6, float a, float t);
	void stopl(float a);
	void set_digital_out(int n, string b);//设置I/O口信号
	float a;//加速度
	float v;//速度
	float t;//时间
};
#endif // UR_SOCKET

