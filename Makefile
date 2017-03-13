all:
	g++ -g main_Config.cpp ./lidar_lite.cpp -I/usr/include/python2.7 -o main_Config -lpigpio -lpthread -lwiringPi -lpython2.7