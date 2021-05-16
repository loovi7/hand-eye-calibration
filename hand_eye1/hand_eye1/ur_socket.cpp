#include "ur_socket.h"
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <stdio.h>
#include <string>
#include <WINSOCK2.H>
#include <iostream>
#include <sstream>
#include <fstream>  
//定义程序中使用的常量
#pragma comment(lib, "ws2_32.lib")
using namespace std;
SocketThread::SocketThread(int port, const char* server_address) 
{	
	PORT = port;//端口
	SERVER_ADDRESS=server_address; //ip地址
	a = 1.2;//加速度
	v = 0.05;//速度
	t = 10;//时间
}
string SocketThread::int2str(const float &int_temp)
{
    string string_temp;
    stringstream stream;
    stream << int_temp;
    string_temp = stream.str();   //此处也可以用 stream>>string_temp
    return string_temp;
}
SOCKET SocketThread::client_steup()//客户端IP地址,客户端的端口号
{
	#define MSGSIZE        1024        
    WSADATA wsaData;
    //连接所用套节字
    SOCKET sClient;
    SOCKADDR_IN client;
    WSAStartup(0x0202, &wsaData);
    // 创建客户端套节字
    sClient = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP); //AF_INET指明使用TCP/IP协议族；
                                                         //SOCK_STREAM, IPPROTO_TCP具体指明使用TCP协议
                                                         // 指明远程服务器的地址信息(端口号、IP地址等)
    memset(&client, 0, sizeof(SOCKADDR_IN)); //先将保存地址的server置为全0
	client.sin_family = PF_INET; //声明地址格式是TCP/IP地址格式
	client.sin_port = htons(PORT); //指明连接服务器的端口号，htons()用于 converts values between the host and network byte order
	client.sin_addr.s_addr = inet_addr(SERVER_ADDRESS);
    connect(sClient, (struct sockaddr *) &client, sizeof(SOCKADDR_IN));
    return sClient;
}
SOCKET SocketThread::server_steup()//客户端IP地址,客户端的端口号
{
	WSADATA wsaData;
	SOCKET sServer, connfd;
	SOCKADDR_IN server;
	WSAStartup(0x0202, &wsaData);
	sServer = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	memset(&server, 0, sizeof(SOCKADDR_IN)); //先将保存地址的server置为全0
	server.sin_family = PF_INET; //声明地址格式是TCP/IP地址格式
	server.sin_port = htons(30000); //指明连接服务器的端口号，htons()用于 converts values between the host and network byte order
	server.sin_addr.s_addr = inet_addr("192.168.12.253");
	bind(sServer, (struct sockaddr *) &server, sizeof(SOCKADDR_IN));
	listen(sServer, 10);
	connfd = accept(sServer, (struct sockaddr*)NULL, NULL);
	return connfd;
}

void SocketThread::send_urscript(string  str)//发送urscript语句
{
	SOCKET connfd;
    SOCKET sClient = client_steup();
    send(sClient, str.c_str(), strlen(str.c_str()), 0);
    closesocket(sClient);
    WSACleanup();
}

void SocketThread::send_urscript(char * fileName)//发送urscript文本
{
	SOCKET sClient = client_steup();
	fstream   fs;
	string    line, str;
	/*fs.open("urscript.txt");*/
	fs.open(fileName);
	if(!fs){		
		cout << "cannot open the file" << endl;
	}
	while (getline(fs, line)) {
		line += '\n';
		str += line;		
	}
	//cout << str;
	send(sClient, str.c_str(), strlen(str.c_str()), 0);
	closesocket(sClient);
	WSACleanup();
}

void SocketThread::resv_urscript()//接收
{
	SOCKET connfd;
	SOCKET sClient = client_steup();
	char buf[8192];
	//while(1){
	//listen(sClient, 10);
	//connfd = accept(sClient, (struct sockaddr*)NULL, NULL);
	recv(sClient, buf, strlen(buf), 0);
	cout << buf;
	//Sleep(1000);
	//}
	//closesocket(sClient);
	//WSACleanup();
}

void SocketThread::recv_tcp_pose()//接收机器人位姿
{		
	float tcp_pose[6];
	const char * split = ",p[]";
	//SOCKET sServer = server_steup();	
	WSADATA wsaData;
	SOCKET sServer, connfd;
	SOCKADDR_IN server;
	WSAStartup(0x0202, &wsaData);
	sServer = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	memset(&server, 0, sizeof(SOCKADDR_IN)); //先将保存地址的server置为全0
	server.sin_family = PF_INET; //声明地址格式是TCP/IP地址格式
	server.sin_port = htons(30000); //指明连接服务器的端口号，htons()用于 converts values between the host and network byte order
	server.sin_addr.s_addr = inet_addr("192.168.2.123");//pc端ip
	ofstream outfile;
	while (1) {
		outfile.open("C:\\data.txt", ios::app);
		char  buf[100];
		float tcp_pose[6];
		bind(sServer, (struct sockaddr *) &server, sizeof(SOCKADDR_IN));
		listen(sServer, 10);
		connfd = accept(sServer, (struct sockaddr*)NULL, NULL);
		recv(connfd, buf, strlen(buf), 0);
		//cout << buf<<endl;
		char  *p;
		p = strtok(buf, split);
		for (int i = 0; i < 6; i++) {
			tcp_pose[i] = strtod(p, NULL);
			cout << tcp_pose[i] << '\t';
			outfile << tcp_pose[i] << " ";
			p = strtok(NULL, split);
		}
		outfile << '\n';
		cout << endl;
		for (int i = 0; i < 100; i++) { buf[i] = 1; }//初始化buf
		outfile.close();
	}
	
	closesocket(sServer);
	WSACleanup();
}
void SocketThread::movej(float q1, float q2, float q3, float q4, float q5, float q6, float a, float v) //弧度值
{
    string command = "movej([";
    command += int2str(q1);
    command += ",";
    command += int2str(q2);
    command += ",";
    command += int2str(q3);
    command += ",";
    command += int2str(q4);
    command += ",";
    command += int2str(q5);
    command += ",";
    command += int2str(q6);
    command += "],a=";
    command += int2str(a);
    command += ",v=";
    command += int2str(v);
    command += ")\n";
    send_urscript(command);

}
void SocketThread::speedj(float v1, float v2, float v3, float v4, float v5, float v6, float a, float t)
{
	string command = "speedj([";
	command += int2str(v1);
	command += ",";
	command += int2str(v2);
	command += ",";
	command += int2str(v3);
	command += ",";
	command += int2str(v4);
	command += ",";
	command += int2str(v5);
	command += ",";
	command += int2str(v6);
	command += "],a=";
	command += int2str(a);
	command += ",t=";
	command += int2str(t);
	command += ")\n";
	send_urscript(command);
}
void SocketThread::speedl(float v1, float v2, float v3, float v4, float v5, float v6, float a, float t)
{
	string command = "speedl([";
	command += int2str(v1);
	command += ",";
	command += int2str(v2);
	command += ",";
	command += int2str(v3);
	command += ",";
	command += int2str(v4);
	command += ",";
	command += int2str(v5);
	command += ",";
	command += int2str(v6);
	command += "],a=";
	command += int2str(a);
	command += ",t=";
	command += int2str(t);
	command += ")\n";
	send_urscript(command);
}
void SocketThread::stopl(float a)
{
	string command = "stopl(";
	command += int2str(a);
	command += ")\n";
	send_urscript(command);
}
void SocketThread::set_digital_out(int n, string b)  //n：输出id（0-9）；b：信号等级
{
    string command = "set_digital_out(";
    command += int2str(n);
    command += ",";
    command += b;
    command += ")\n";
    send_urscript(command);
	
}
