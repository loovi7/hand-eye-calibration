#include "ur_socket.h"
#include "Joystick2.h"
#include <thread>
int main(int argc, char *argv[])
{
	
	JoystickThread c;
	thread t1([&c](){
		JOYINFOEX joyinfoex1;
		c.state_machine(joyinfoex1, 1);
	}
	);
	//thread t2([&c](){
	//	JOYINFOEX joyinfoex2;
	//	c.state_machine(joyinfoex2, 2);
	//}
	//);
	t1.join();
	//t2.join();
	return 0;  
}