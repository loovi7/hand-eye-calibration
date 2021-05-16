#include "ur_socket.h"
#include "Joystick2.h"
#include <math.h>
#include <iostream>
#include <sstream>
#include <fstream>  
#include <Windows.h>
using namespace std;
//#define Joy1_IP "192.168.1.113"
//#define Joy2_IP "192.168.1.110"
#define Joy1_PORT 30003		
#define Joy2_PORT 30003
#define vel vv/65535*0.075//速度

JoystickThread::JoystickThread() :
	m_xThreshold(12000),
	m_yThreshold(12000),
	m_zThreshold(12000),
	m_rThreshold(12000),
	m_run(true),
	m_valid(false),
	m_signal(true),
	m_xy_signal(true),
	m_z_signal(true),
	m_r_signal(true),
	m_pov_signal(true),
	m_button_signal(true),
	m_monitor_interval(50)//循环延时
{
	for (int i = 0; i < 3; i++)
	{
		old_xPos[i] = 0;
		old_yPos[i] = 0;
		old_zPos[i] = 0;
		old_rPos[i] = 0;
		old_povPos[i] = 65535;
	}
	JOYINFO joyinfo;
	if (joyGetNumDevs() > 0 && joyGetPos(JOYSTICKID1, &joyinfo) != JOYERR_UNPLUGGED && joyGetPos(JOYSTICKID2, &joyinfo) != JOYERR_UNPLUGGED)
	{
		m_valid = true;
		//cout << "m_valid = true";
	}
	for (int j = 0; j < 3; j++)
		for (int i = 0; i < 12; i++)
		{
			m_button[j][i] = 0;
		}
}

void JoystickThread::stop()
{
	m_run = 0;
}

void JoystickThread::setXYThreshold(int x, int y)
{
	m_xThreshold = abs(x);
	m_yThreshold = abs(y);
}

void JoystickThread::setZThreshold(int z)
{
    m_zThreshold = abs(z);
}

void JoystickThread::state_machine(JOYINFOEX &joyinfoex, int n)
{
	joyinfoex.dwSize = sizeof(JOYINFOEX);
	joyinfoex.dwFlags = JOY_RETURNALL;
/*
	char Joy1_ip[30];
	ifstream myFile1;
	myFile1.open("Joy1_IP.txt");
	if (myFile1.is_open())
	{
		while (!myFile1.eof())
		{
			myFile1 >> Joy1_ip;
		}
	}
	myFile1.close();
	Joy1_IP = Joy1_ip;
	

	char Joy2_ip[30];
	ifstream myFile2;
	myFile2.open("Joy2_IP.txt");
	if (myFile2.is_open())
	{
		while (!myFile2.eof())
		{
			myFile2 >> Joy2_ip;
		}
	}
	myFile2.close();
	Joy2_IP = Joy2_ip;
	
*/	
	Joy1_IP = "192.168.1.100";
	Joy2_IP = "192.168.1.101";
	if (n == 1) {
		port = Joy1_PORT;
		ip = Joy1_IP;
	}
	else if (n == 2) {
		port = Joy2_PORT;
		ip = Joy2_IP;
	}
	SocketThread aa(port, ip);
	while (1)
	{
		if (!m_signal || !m_xy_signal) return;
		if (n == 1) {
			joyGetPosEx(JOYSTICKID1, &joyinfoex);
		}
		else if (n == 2) {
			joyGetPosEx(JOYSTICKID2, &joyinfoex);
		}
		AxisX_StateMachine(joyinfoex.dwXpos, n, aa, 65535 - joyinfoex.dwZpos);
		AxisY_StateMachine(joyinfoex.dwYpos, n, aa, 65535 - joyinfoex.dwZpos);
		AxisR_StateMachine(joyinfoex.dwRpos, n, aa, 65535 - joyinfoex.dwZpos);
		AxisPOV_StateMachine(joyinfoex.dwPOV, n, aa, 65535 - joyinfoex.dwZpos);
		Button_StateMachine(joyinfoex.dwButtons, n, aa, 65535 - joyinfoex.dwZpos);
		Sleep(m_monitor_interval);
	}
}

void JoystickThread::run()
{
	JOYINFOEX joyinfoex1;
	joyinfoex1.dwSize = sizeof(JOYINFOEX);
	joyinfoex1.dwFlags = JOY_RETURNALL;
	JOYINFOEX joyinfoex2;
	joyinfoex2.dwSize = sizeof(JOYINFOEX);
	joyinfoex2.dwFlags = JOY_RETURNALL;
	while (m_valid && m_run)
	{
	/*	if ((joyGetPosEx(JOYSTICKID1, &joyinfoex1) == JOYERR_NOERROR) && (joyGetPosEx(JOYSTICKID2, &joyinfoex2) == JOYERR_NOERROR))
		{
			AxisX_StateMachine(joyinfoex1.dwXpos, 1, joyinfoex1.dwZpos);
			AxisY_StateMachine(joyinfoex1.dwYpos, 1, joyinfoex1.dwZpos);
			//AxisZ_StateMachine(joyinfoex1.dwZpos,1，joyinfoex1.dwZpos);
			AxisR_StateMachine(joyinfoex1.dwRpos, 1, joyinfoex1.dwZpos);
			AxisPOV_StateMachine(joyinfoex1.dwPOV, 1, joyinfoex1.dwZpos);
			Button_StateMachine(joyinfoex1.dwButtons, 1, joyinfoex1.dwZpos);

			AxisX_StateMachine(joyinfoex2.dwXpos, 2, joyinfoex2.dwZpos);
			AxisY_StateMachine(joyinfoex2.dwYpos, 2, joyinfoex2.dwZpos);
			//AxisZ_StateMachine(joyinfoex2.dwZpos,2, joyinfoex2.dwZpos);
			AxisR_StateMachine(joyinfoex2.dwRpos, 2, joyinfoex2.dwZpos);
			AxisPOV_StateMachine(joyinfoex2.dwPOV, 2, joyinfoex2.dwZpos);
			Button_StateMachine(joyinfoex2.dwButtons, 2, joyinfoex2.dwZpos);
		}
		Sleep(m_monitor_interval);
		*/
	}
}

void JoystickThread::setMonitorInterval(int interval)
{
    m_monitor_interval = interval;
}

//检测轴是否移动以及移动方向
int JoystickThread::Axis_stateMachine(int pos, int threshold, int &old_pos)
{
	pos = pos - 32767;
	if (abs(pos) <= threshold)//在阈值范围内
	{
		pos = 0;
	}
	else if (pos > threshold)//向右移动
	{
		pos = 1;
	}
	else                      //向左移动
	{
		pos = -1;
	}
	if (pos == old_pos) return 0;
	int ret;
	switch (pos)
	{
	case -1:
		ret = -1;    //左移
		break;
	case 0:
		if (old_pos == -1)
		{
			ret = -2;//从左边回来的
		}
		else
		{
			ret = 2;//从右边回来的
		}
		break;
	case 1:
		ret = 1;    //右移
		break;
	}
	old_pos = pos;
	return ret;
}

void JoystickThread::AxisX_StateMachine(int xPos, int n, SocketThread aa,  float vv)
{
	int ret = Axis_stateMachine(xPos, m_xThreshold, old_xPos[n-1]);
	switch (ret)
	{
	case -1:
		//Joy_MoveLeft();
		aa.speedl(0, 0, 0, 0, vel * 2, 0, aa.a, aa.t);
		//cout << "Joy" << n << "_MoveLeft";
		break;
	case -2:
		//Joy_MoveLeftStop();
		aa.stopl(aa.a);
		//cout << "Joy" << n << "_MoveLeftStop";
		break;
	case 2:
		//Joy_MoveRightStop();
		aa.stopl(aa.a);
		//cout << "Joy" << n << "_MoveRightStop";
		break;
	case 1:
		//Joy_MoveRight();
		aa.speedl(0, 0, 0, 0, -vel * 3, 0, aa.a, aa.t);
		//cout << "Joy" << n << "_MoveRight";
		break;
	default:
		break;
	}
}

void JoystickThread::AxisY_StateMachine(int yPos, int n, SocketThread aa, float vv)
{
	if (!m_signal || !m_xy_signal) return;
	int ret = Axis_stateMachine(yPos, m_yThreshold, old_yPos[n - 1]);
	switch (ret)
	{
	case -1:
		//Joy_MoveForward();
		aa.speedl(0, 0, 0, vel * 3, 0, 0, aa.a, aa.t);
		//cout << "Joy" << n << "_MoveForward";
		break;
	case -2:
		//Joy_MoveForwardStop();
		aa.stopl(aa.a);
		//cout << "Joy" << n << "_MoveForwardStop";
		break;
	case 2:
		//Joy_MoveBackwardStop();
		aa.stopl(aa.a);
		//cout << "Joy" << n << "_MoveBackwardStop";
		break;
	case 1:
		//Joy_MoveBackward();
		aa.speedl(0, 0, 0, -vel * 3, 0, 0, aa.a, aa.t);
		//cout << "Joy" << n << "_MoveBackward";
		break;
	default:
		break;
	}
}

void JoystickThread::AxisZ_StateMachine(int zPos, int n, SocketThread aa, float vv)
{

}

void JoystickThread::AxisR_StateMachine(int rPos, int n, SocketThread aa, float vv)
{
	if (!m_signal || !m_r_signal) return;
	int ret = Axis_stateMachine(rPos, m_rThreshold, old_rPos[n - 1]);
	if (ret == 0) return;
	switch (ret)
	{
	case 1:
		//Joy_MoveClockWise();
		aa.speedl(0, 0, 0, 0, 0, -vel * 3, aa.a, aa.t);
		//cout << "Joy" << n << "_MoveClockWise()";
		break;
	case 2:
		//Joy_MoveClockWiseStop();
		aa.stopl(aa.a);
		//cout << "Joy" << n << "_MoveClockWiseStop()";
		break;
	case -2:
		//Joy_MoveCCWStop();
		aa.stopl(aa.a);
		//cout << "Joy" << n << "_MoveCCWStop()";
		break;
	case -1:
		//Joy_MoveCCW();
		aa.speedl(0, 0, 0, 0, 0, vel * 3, aa.a, aa.t);
		//cout << "Joy" << n << "_MoveCCW()";
		break;
	default:
		break;
	}
}

void JoystickThread::AxisPOV_StateMachine(int povPos, int n, SocketThread aa, float vv)
{
	
	if (!m_signal || !m_pov_signal) return;
	if (povPos != old_povPos[n - 1])
	{
		//switch (povPos)
		//{
		//case 0:
		//	//Joy_MoveNorth();
		//	aa.speedl(0, -vel, 0, 0, 0, 0, aa.a, aa.t);
		//	//cout << "Joy" << n << "_MoveNorth()";
		//	break;
		//case 9000:
		//	//Joy_MoveEast();
		//	aa.speedl(-vel, 0, 0, 0, 0, 0, aa.a, aa.t);
		//	//cout << "Joy" << n << "_MoveEast()";
		//	break;
		//case 18000:
		//	//Joy_MoveSouth();
		//	aa.speedl(0, vel, 0, 0, 0, 0, aa.a, aa.t);
		//	//cout << "Joy" << n << "_MoveSouth()";
		//	break;
		//case 27000:
		//	//Joy_MoveWest();
		//	aa.speedl(vel, 0, 0, 0, 0, 0, aa.a, aa.t);
		//	//cout << "Joy" << n << "_MoveWest()";
		//	break;
		//case 65535:
		//	//Joy_MoveCenter();
		//	aa.stopl(aa.a);
		//	//cout << "Joy" << n << "_MoveCenter()";
		//	break;
		//default:
		//	break;
		//}
		if (povPos == 65535) {
			aa.stopl(aa.a);
		}
		else if (povPos != 65535) {
			double x = vel*sin((27000 - (double)povPos) / 18000 * 3.141);
			double y = vel*cos((27000 - (double)povPos) / 18000 * 3.141);
			aa.speedl( y, x, 0, 0, 0, 0, aa.a, aa.t);
		}
	}
	old_povPos[n - 1] = povPos;
}

void JoystickThread::Button_StateMachine(int button, int nn, SocketThread aa, float vv)
{
	const int mask[12] = { 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048 };
	SocketThread bb(29999, ip);
	for (int i = 0; i < 12; i++)
	{
		int n = button & mask[i];
		if (n ^ m_button[nn - 1][i]) // 如果当前 button 与以前的 button 不同
		{
			m_button[nn - 1][i] = n;
			if (!m_signal || !m_button_signal) continue;
			if (n)
			{
				//cout << "Joy" << nn << "_Button" << i + 1 << "Pressed()";
				switch (i + 1)
				{
				case 1:
					fflag += 1;
					if (fflag % 2 != 0)
					{
						switch (flag[nn])
						{
						case 7:aa.set_digital_out(0, "True"); break;
						case 8:aa.set_digital_out(1, "True"); break;
						case 9:aa.set_digital_out(2, "True"); break;
						case 10:aa.set_digital_out(3, "True"); break;
						case 11:aa.set_digital_out(4, "True"); break;
						case 12:aa.set_digital_out(5, "True"); break;
						}
					}
					else if (fflag % 2 == 0)
					{
						switch (flag[nn])
						{
						case 7:aa.set_digital_out(0, "False"); break;
						case 8:aa.set_digital_out(1, "False"); break;
						case 9:aa.set_digital_out(2, "False"); break;
						case 10:aa.set_digital_out(3, "False"); break;
						case 11:aa.set_digital_out(4, "False"); break;
						case 12:aa.set_digital_out(5, "False"); break;
						}
					}
					//Joy_Button1Pressed();			
					break;
				case 2:
					//Joy_Button2Pressed();
					flag[nn] = 0;
					bb.send_urscript("unlock protective stop\n");
					//bb.send_urscript("Gripper Open(1)\n");
					break;
				case 3:
					//Joy_Button3Pressed();
					aa.speedl(0, 0, -vel, 0, 0, 0, aa.a, aa.t);
					break;
				case 4:
					//Joy_Button4Pressed();
					switch (flag[nn])
					{
					case 7:aa.speedj(-vel * 3, 0, 0, 0, 0, 0, aa.a, aa.t); break;
					case 8:aa.speedj(0, -vel * 3, 0, 0, 0, 0, aa.a, aa.t); break;
					case 9:aa.speedj(0, 0, -vel * 3, 0, 0, 0, aa.a, aa.t); break;
					case 10:aa.speedj(0, 0, 0, -vel * 3, 0, 0, aa.a, aa.t); break;
					case 11:aa.speedj(0, 0, 0, 0, -vel * 3, 0, aa.a, aa.t); break;
					case 12:aa.speedj(0, 0, 0, 0, 0, -vel * 3, aa.a, aa.t); break;
					}
					break;
				case 5:
					//Joy_Button5Pressed();
					aa.speedl(0, 0, vel, 0, 0, 0, aa.a, aa.t);
					break;
				case 6:
					//Joy_Button6Pressed();
					switch (flag[nn])
					{
					case 7:aa.speedj(vel * 3, 0, 0, 0, 0, 0, aa.a, aa.t); break;
					case 8:aa.speedj(0, vel * 3, 0, 0, 0, 0, aa.a, aa.t); break;
					case 9:aa.speedj(0, 0, vel * 3, 0, 0, 0, aa.a, aa.t); break;
					case 10:aa.speedj(0, 0, 0, vel * 3, 0, 0, aa.a, aa.t); break;
					case 11:aa.speedj(0, 0, 0, 0, vel * 3, 0, aa.a, aa.t); break;
					case 12:aa.speedj(0, 0, 0, 0, 0, vel * 3, aa.a, aa.t); break;
					}
					break;
				case 7:
					//Joy_Button7Pressed();
					flag[nn] = 7;
					break;
				case 8:
					//Joy_Button8Pressed();
					flag[nn] = 8;
					break;
				case 9:
					//Joy_Button9Pressed();
					flag[nn] = 9;
					break;
				case 10:
					//Joy_Button10Pressed();
					flag[nn] = 10;
					break;
				case 11:
					//Joy_Button11Pressed();
					flag[nn] = 11;
					break;
				case 12:
					//Joy_Button12Pressed();
					flag[nn] = 12;
					break;

				default:
					break;
				}
			}
			else
			{
				//cout << "Joy" << nn << "_Button" << i + 1 << "Released()";
				switch (i + 1)
				{
				case 1:
					//Joy_Button1Released();
					
					break;
				case 2:
					//Joy_Button2Released();
					
					break;
				case 3:
					//Joy_Button3Released();
					aa.stopl(aa.a);
					break;
				case 4:
					//Joy_Button4Released();
					aa.stopl(aa.a);
					break;
				case 5:
					//Joy_Button5Released();
					aa.stopl(aa.a);
					break;
				case 6:
					//Joy_Button6Released();
					aa.stopl(aa.a);
					break;
				case 7:
					//Joy_Button7Released();
					break;
				case 8:
					//Joy_Button8Released();
					break;
				case 9:
					//Joy_Button9Released();
					break;
				case 10:
					//Joy_Button10Released();
					break;
				case 11:
					//Joy_Button11Released();
					break;
				case 12:
					//Joy_Button12Released();
					break;

				default:
					break;
				}
			}
		}
	}
}