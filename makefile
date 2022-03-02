all:  mavlink_control

mavlink_control: mavlink_control.cpp
	g++ -w -I mavlink/include/mavlink/v2.0 joystick.cpp mavlink_control.cpp serial_port.cpp gimbal_interface.cpp -o gSDK -lpthread -ljsoncpp

clean:
	 rm -rf *o gSDK
