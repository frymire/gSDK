all:  mavlink_control

main: main.cpp
	g++ -std=c++11 -I mavlink/include/mavlink/v2.0 main.cpp serial_port.cpp gimbal_interface.cpp -o bin/gSDK -lpthread

git_submodule:
	git submodule update --init --recursive

clean:
	 rm -rf *o gSDK
