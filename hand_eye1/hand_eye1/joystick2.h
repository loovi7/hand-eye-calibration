#ifndef JOYSTICK_H
#define JOYSTICK_H
#include "ur_socket.h"
#include <Windows.h>

/**
  * 4 轴 12 按键Logitech 3D Exetrem Joystick 的驱动程序。
  * 使用是微软 multimedia joystick API。
 */
class JoystickThread 
{
public:
    explicit JoystickThread();
    /**
     * @brief run 启动监听线程
     */
	//void state_machine(JOYINFOEX joyinfoex, int n);
    void run();
    /**
     * @brief stop 停止监听线程
     */
    void stop();
	void read();
/*
    /// 手柄 XY 轴的输出信号
    /// 需 enableSignal(true), enableXYSignal(true) 才有效
    void Joy_MoveForward();
    void Joy_MoveBackward();
    void Joy_MoveForwardStop();
    void Joy_MoveBackwardStop();
    void Joy_MoveLeft();
    void Joy_MoveRight();
    void Joy_MoveLeftStop();
    void Joy_MoveRightStop();

    /// 手柄 Z 轴的输出信号
    /// 需 enableSignal(true), enableZSignal(true) 才有效
    void Joy_MovePositive();
    void Joy_MoveNegeative();
    void Joy_MovePositiveStop();
    void Joy_MoveNegeativeStop();

    /// 手柄 R 轴的输出信号
    /// 需 enableSignal(true), enableRSignal(true) 才有效
    void Joy_MoveClockWise();
    void Joy_MoveCCW();
    void Joy_MoveClockWiseStop();
    void Joy_MoveCCWStop();

    /// 手柄 POV 的输出信号
    /// 需 enableSignal(true), enablePOVSignal(true) 才有效
    void Joy_MoveNorth();
    void Joy_MoveSouth();
    void Joy_MoveWest();
    void Joy_MoveEast();

    /// 手柄按键的输出信号
    /// 需 enableSignal(true), enableButtionSignal(true) 才有效
    void Joy_Button1Pressed();
    void Joy_Button2Pressed();
    void Joy_Button3Pressed();
    void Joy_Button4Pressed();
    void Joy_Button5Pressed();
    void Joy_Button6Pressed();
    void Joy_Button7Pressed();
    void Joy_Button8Pressed();
    void Joy_Button9Pressed();
    void Joy_Button10Pressed();
    void Joy_Button11Pressed();
    void Joy_Button12Pressed();

    void Joy_Button1Released();
    void Joy_Button2Released();
    void Joy_Button3Released();
    void Joy_Button4Released();
    void Joy_Button5Released();
    void Joy_Button6Released();
    void Joy_Button7Released();
    void Joy_Button8Released();
    void Joy_Button9Released();
    void Joy_Button10Released();
    void Joy_Button11Released();
    void Joy_Button12Released();

*/
    /**
     * @brief setXYThreshold 设置 XY 手柄的开启阈值。
     * 由于手柄使用时间长了会由于机械问题在不停留在原点。所以需要有个开启阈值。
     * @param x  x 阈值
     * @param y  y 阈值
     */
    void setXYThreshold(int x, int y);
    /**
     * @brief setZThreshold 设置 Z 手柄的开启阈值。
     * @param z  z 阈值
     */
    void setZThreshold(int z);
    /**
     * @brief enableSignal 可以用这个函数关闭所有的信号
     * @param on
     */
    void enableSignal(bool on) {m_signal = on;}
    /**
     * @brief enableXYSignal XY 手柄的使能控制，只有enableSignal(true)时才有效。
     * 相当于对手柄的信号有两级控制，第一级是全局使能，第二级是这里的使能。
     * @param on
     */

    void enableXYSignal(bool on) {m_xy_signal = on;}
    void enableZSignal(bool on) {m_z_signal = on;}
    void enableRSignal(bool on) {m_r_signal = on;}
    void enablePOVSignal(bool on) {m_pov_signal = on;}
    void enableButtionSignal(bool on ){m_button_signal = on;}
    /**
     * @brief setMonitorInterval 设置内部轮询的时间间隔
     * @param interval 单位 ms，范围 20 - 10000。
     */
    void setMonitorInterval(int interval);
    /**
     * @brief 夹爪开与关
    void gripper_open();
    void gripper_close();
    */
	/**
	* 主程序段，包含检测joystick的键值并发送urscript
	*/
	void state_machine(JOYINFOEX &joyinfoex, int n);
    int Axis_stateMachine(int pos, int threshold, int &old_pos);
    void AxisX_StateMachine(int xPos, int n, SocketThread aa,float vv);
    void AxisY_StateMachine(int yPos, int n, SocketThread aa, float vv);
    void AxisZ_StateMachine(int zPos, int n, SocketThread aa, float vv);
    void AxisR_StateMachine(int rPos, int n, SocketThread aa, float vv);
    void AxisPOV_StateMachine(int povPos, int n, SocketThread aa, float vv);
    void Button_StateMachine(int button, int nn, SocketThread aa, float vv);

private:
    int m_xThreshold;
    int m_yThreshold;
    int m_zThreshold;
    int m_rThreshold;

    int old_xPos[3];
    int old_yPos[3];
    int old_zPos[3];
    int old_rPos[3];
	int old_povPos[3];
    int m_button[3][12];

    bool m_run;
    bool m_valid;
    bool m_signal;
    bool m_xy_signal;
    bool m_z_signal;
    bool m_r_signal;
    bool m_pov_signal;
    bool m_button_signal;//循环频率时间

    int m_monitor_interval;
	int flag[3];
	int fflag=0;
	const char* Joy1_IP;
	const char* Joy2_IP;

	int port;
	const char* ip = 0;
};

#endif // JOYSTICK_H
