all:
	g++ main_Config.cpp ./lidar_lite.cpp -I.. -o main_Config -lpigpio -lpthread -lwiringPi