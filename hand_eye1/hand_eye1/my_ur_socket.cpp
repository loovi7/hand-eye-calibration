#include "my_ur_socket.h"
using namespace std;
ur_socket::ur_socket(int port, const char* server_address):robot_type(ur3),rotangle(M_PI_2),split(",") //旋转角度
{	
	//m_hMutex = CreateMutex(nullptr, FALSE, TEXT("lcok_1"));
	//m_hRecv = NULL;
	WSADATA wsaData;
	memset(buf, '\0', sizeof(buf));
	memset(f_buf, '\0',sizeof(f_buf));	
	int err = WSAStartup(0x0202, &wsaData);
	if (err != 0)
	{
		cout << "setup socket error!" << endl;
		return;
	}
	// 创建客户端套节字
	// m_client为接收机械臂角度数据用
	m_client = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (m_client == INVALID_SOCKET){
		cout << "创建socket网络失败！！！" << endl;
		return;
	}
	memset(&client, 0, sizeof(SOCKADDR_IN)); //先将保存地址的server置为全0
	client.sin_family = PF_INET; //声明地址格式是TCP/IP地址格式
	client.sin_port = htons(port); //指明连接服务器的端口号，htons()用于 converts values between the host and network byte order
	client.sin_addr.s_addr = inet_addr(server_address);
	int Ret = connect(m_client, (struct sockaddr *) &client, sizeof(SOCKADDR_IN));
	if (Ret == SOCKET_ERROR){
		cout << "连接主机失败！" << endl;
		return;
	}
	// 创建客户端套节字
	// m_client2为接收机械臂力数据用
	m_client2 = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (m_client2 == INVALID_SOCKET){
		cout << "创建socket2网络失败！！！" << endl;
		return;
	}
	memset(&client2, 0, sizeof(SOCKADDR_IN)); //先将保存地址的server置为全0
	client2.sin_family = PF_INET; //声明地址格式是TCP/IP地址格式
	client2.sin_port = htons(63351); //指明连接服务器的端口号，htons()用于 converts values between the host and network byte order
	client2.sin_addr.s_addr = inet_addr(server_address);
	cout << "启动连接" << endl;
	int Ret2 = connect(m_client2, (struct sockaddr *) &client2, sizeof(SOCKADDR_IN));
	if (Ret2 == SOCKET_ERROR){
		cout << "连接主机2失败！" << endl;
		return;
	}

	cout << "连接成功！" << endl;
	cout << "开启接收线程..." << endl;
	CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)Call_recv_joint, this, 0, NULL);
	//CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)Call_recv_force, this, 0, NULL);
}
ur_socket::~ur_socket()
{
	closesocket(m_client);
	WSACleanup();
}
Mat ur_socket::DhToFrame(double alpha_, double a_, double d_, double theta_)
{
	Mat T1 = Mat::zeros(3, 1, CV_64F);
	T1.at<double>(2, 0) = d_;

	Mat T2 = Mat::zeros(3, 1, CV_64F);
	T2.at<double>(0, 0) = a_;

	return translation(T1) * rotZ(theta_)* translation(T2)* rotX(alpha_);
}

Mat ur_socket::forwardkin(int robot_type, double joint_angle[6])
{
	double *alpha_f, *a_f, *d_f, *theta_f;
	//UR5
	double  alpha_ur5_f[] = { M_PI_2,0.0,0.0,M_PI_2,-M_PI_2,0.0 };
	double  a_ur5_f[] = { 0.0,-0.425,-0.39225,0.0,0.0,0.0 };
	double  d_ur5_f[] = { 0.089159,0.0,0.0,0.10915,0.09465,0.0823 };
	double  theta_ur5_f[] = { 0.0,0.0,0.0,0.0,0.0,0.0 };
	//UR3
	double  alpha_ur3_f[] = { M_PI_2,0.0,0.0,M_PI_2,-M_PI_2,0.0 };
	double  a_ur3_f[] = { 0.0,-0.24365,-0.21325,0.0,0.0,0.0 };
	double  d_ur3_f[] = { 0.1519,0.0,0.0,0.11235,0.08535,0.0819 };
	double  theta_ur3_f[] = { 0.0,0.0,0.0,0.0,0.0,0.0 };
	//UR10
	double  alpha_ur10_f[] = { M_PI_2,0.0,0.0,M_PI_2,-M_PI_2,0.0 };
	double  a_ur10_f[] = { 0.0,-0.612,-0.5723,0.0,0.0,0.0 };
	double  d_ur10_f[] = { 0.1273,0.0,0.0,0.163941,0.1157,0.0922 };
	double  theta_ur10_f[] = { 0.0,0.0,0.0,0.0,0.0,0.0 };
	////DH参数

	//Frame ret_fram4e(Vector(0, 0, 0));
	Mat T = Mat::zeros(3, 1, CV_64F);
	Mat ret_frame = translation(T);
	//基座标系变换
	//ret_frame=ret_frame*Frame(Rotation::RotZ(M_PI));

	if (robot_type == 3)
	{
		alpha_f = alpha_ur3_f;
		a_f = a_ur3_f;
		d_f = d_ur3_f;
		theta_f = theta_ur3_f;
	}
	else if (robot_type == 5)
	{
		alpha_f = alpha_ur5_f;
		a_f = a_ur5_f;
		d_f = d_ur5_f;
		theta_f = theta_ur5_f;
	}
	else if(robot_type == 10)
	{
		alpha_f = alpha_ur10_f;
		a_f = a_ur10_f;
		d_f = d_ur10_f;
		theta_f = theta_ur10_f;
	}
	else {
		alpha_f = 0;
		a_f = 0;
		d_f = 0;
		theta_f = 0;
	}
	for (int i = 0; i < 6; i += 1)
	{
		ret_frame = ret_frame*DhToFrame(alpha_f[i], a_f[i], d_f[i], joint_angle[i]);
	}

	Mat endToTool = Mat::zeros(3, 1, CV_64F);  // 末端-> 工具
	endToTool.at<double>(0, 0) = -0.185;
	endToTool.at<double>(1, 0) = 0.0;
	endToTool.at<double>(2, 0) = 0.096;
	return ret_frame /** translation(endToTool)*/;
}
//void ur_socket::convert_force(int robot_type,double sensor_data[])
//{
//	Frame Forwardkin = forwardkin(robot_type, joint);
//	force_base[0] = sensor_data[0];
//	force_base[1] = sensor_data[1];
//	force_base[2] = sensor_data[2];
//	torque_base[0] = sensor_data[3];
//	torque_base[1] = sensor_data[4];
//	torque_base[2] = sensor_data[5];
//	force_base = Forwardkin.M * force_base;
//	torque_base = Forwardkin.M * torque_base;
//}

string ur_socket::int2str(const float &int_temp)
{
	string string_temp;
	stringstream stream;
	stream << int_temp;
	string_temp = stream.str();   //此处也可以用 stream>>string_temp
	return string_temp;
}

int64_t ntohl_64(int64_t buf)
{
	int64_t ret = 0;
	uint32_t high, low;
	low = buf & 0xFFFFFFFF;
	high = (buf >> 32) & 0xFFFFFFFF;
	low = ntohl(low);
	high = ntohl(high);
	ret = low;
	ret <<= 32;
	ret |= high;
	return ret;
}

int ur_socket::unpack_16(char buf[], int &offset_len)
{
	int16_t tmp;
	memcpy(&tmp, buf + offset_len, 2);
	offset_len += 2;
	tmp = ntohs(tmp);
	return tmp;
}
int ur_socket::unpack_32(char buf[], int &offset_len)
{
	int32_t tmp = 0;
	memcpy(&tmp, buf + offset_len, 4);
	offset_len += 4;
	tmp = ntohl(tmp);
	return tmp;
}
double ur_socket::unpack_64(char buf[], int &offset_len)
{
	int64_t tmp = 0;
	double val = 0;
	memcpy(&tmp, buf + offset_len, 8);
	offset_len += 8;
	tmp = ntohl_64(tmp);
	memcpy(&val, &tmp, 8);
	return val;
}
void ur_socket::unpack_vector(char buf[], int &offset_len, double val[])
{
	for (int i = 0; i<6; i++){
		val[i] = unpack_64(buf, offset_len);
	}
}

void ur_socket::get_force(double pos[])
{
	p = strtok_s(f_buf, split, &pNext);
	for (int i = 0; i < 6; i++) {
		pos[i] = strtod(p, NULL);
		//cout << pos[i] << '\t';
		p = strtok_s(NULL, split, &pNext);
	}
	//cout << endl;
}
void ur_socket::get_joint_pos(double pos[])
{
	int len_mid = 4 + 8 + 48 * 5;
	unpack_vector(buf, len_mid, pos);
}
void ur_socket::get_tcp_pos(double pos[])
{
	int len_mid = 4 + 8 + 48 * 9;
	unpack_vector(buf, len_mid, pos);
}
void ur_socket::get_tcp_vel(double pos[])
{
	int len_mid = 4 + 8 + 48 * 10;
	unpack_vector(buf, len_mid, pos);
}
void ur_socket::get_tcp_force(double pos[])
{
	int len_mid = 4 + 8 + 48 * 11;
	unpack_vector(buf, len_mid, pos);
}
void ur_socket::run_recv_joint()
{
	int t = 0;
	int len = 0;
	int temp = 0;
	char data_buf[4000];
	//设置阻塞方式下的延时溢出1s
	//int recvTimeout = 1000;
	//setsockopt(m_client, SOL_SOCKET, SO_RCVTIMEO, (char *)&recvTimeout, sizeof(int));
	while (1){
		t = fix_len - len;
		temp = recv(m_client, data_buf + len, t, 0);
		//TODO 
		if (temp == 0)
		{
			cout << "主机中断";
			cout << "连接失败或中断，重新尝试连接......";
			cout << "启动连接" << endl;

			int Ret = connect(m_client, (struct sockaddr *) &client2, sizeof(SOCKADDR_IN));
			if (Ret == SOCKET_ERROR)
			{
				while (Ret == SOCKET_ERROR){
					cout << "连接主机失败！" << endl;
					cout << "再次连接......" << endl;
					Ret = connect(m_client, (struct sockaddr *) &client2, sizeof(SOCKADDR_IN));
				}
			}
			cout << "连接成功！" << endl;

			len = 0;
			temp = 0;
			continue;
		}
		else if (temp < 0)
		{
			cout << "新建立连接" << endl;
			closesocket(m_client);
			m_client = socket(AF_INET, SOCK_STREAM, 0);
			if (m_client == INVALID_SOCKET){
				cout << "创建socket网络失败！！！" << endl;
				return;
			}

			cout << "启动连接" << endl;
			int Ret = connect(m_client, (struct sockaddr *) &client, sizeof(SOCKADDR_IN));
			if (Ret == SOCKET_ERROR)
			{
				while (Ret == SOCKET_ERROR)
				{
					cout << "连接主机失败！" << endl;
					cout << "再次连接......" << endl;
					Ret = connect(m_client, (struct sockaddr *) &client, sizeof(SOCKADDR_IN));
				}
			}
			cout << "连接成功！" << endl;

			//setsockopt(m_client, SOL_SOCKET, SO_RCVTIMEO, (char *)&recvTimeout, sizeof(int));
			len = 0;
			temp = 0;
			continue;
		}

		len += temp;
		if (len < fix_len){
			temp = 0;
			continue;
		}
		//WaitForSingleObject(m_hMutex, INFINITE);
		memcpy(buf, data_buf, fix_len);
		memcpy(data_buf, data_buf + fix_len, len - fix_len);
		//ReleaseMutex(m_hMutex);
		len -= fix_len;
		temp = 0;
		get_joint_pos(joint);
	}
}

//void ur_socket::run_recv_force()
//{
//	int t = 0;
//	int len = 0;
//	int temp = 0;//接收的字节数
//	char data_buf[force_len];
//	int left_pare;
//	int right_pare;
//	//滤波器参数
//	double old_data[6];
//	int old_flag[6];
//	int new_flag[6];
//	int num[6];
//	float k[6];
//	bool once = true;
//	while (1) {
//		memset(f_buf, 0, sizeof(f_buf));
//		temp = recv(m_client2, data_buf, force_len, 0);
//		for (int i = 0; i < force_len; i++) {
//			if (data_buf[i] == '(') {
//				left_pare = i;
//				break;
//			}
//		}
//		for (int j = left_pare; j < force_len; j++) {
//			if (data_buf[j] == ')') {
//				right_pare = j;
//				break;
//			}
//		}
//		memcpy(f_buf, data_buf + left_pare + 1, right_pare - left_pare - 1);
//		//解析字节流
//		get_force(sensor_data);
//		if (once) {
//			for (int i = 0; i < 6; i++)
//				old_data[i] = sensor_data[i];
//			once = false;
//		}
//		for (int i = 0; i < 6; i++)
//			lowpass_filter(old_data[i], sensor_data[i], f_sensor_data[i], old_flag[i], new_flag[i], num[i], k[i]);
//		//将力传感器转换到机器人坐标系
//		convert_force(robot_type, sensor_data);
//	}
//}


Mat translation(Mat T) {
	if (T.size() != Size(1, 3)) {
		cout << "Error In Tranlation Matrix" << endl;
		system("PAUSE");
			exit(0);
	}
	Mat ret = Mat::eye(4, 4, CV_64F);
	ret.at<double>(0, 3) = T.at<double>(0, 0);
	ret.at<double>(1, 3) = T.at<double>(1, 0);
	ret.at<double>(2, 3) = T.at<double>(2, 0);
	return ret;
}

Mat rotZ(double theta) {
	Mat ret = Mat::eye(4, 4, CV_64F);
	ret.at<double>(0, 0) = cos(theta);
	ret.at<double>(0, 1) = -sin(theta);
	ret.at<double>(1, 0) = sin(theta);
	ret.at<double>(1, 1) = cos(theta);
	return ret;
}


Mat rotX(double theta) {	
	Mat ret = Mat::eye(4, 4, CV_64F);
	ret.at<double>(1, 1) = cos(theta);
	ret.at<double>(1, 2) = -sin(theta);
	ret.at<double>(2, 1) = sin(theta);
	ret.at<double>(2, 2) = cos(theta);
	return ret;
}