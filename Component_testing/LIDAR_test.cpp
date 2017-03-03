#include <include/lidar_lite.h>
#include <cstdio>
#include <unistd.h>

#define DEBUG

int dist; //Range measured (in cm)

int main(){
	//Initalize Lidar-Lite
	Lidar_Lite l1(1);
	if (l1.connect() < 0){	//If error connect, exit
		#ifdef DEBUG
			printf("Error connecting: %d\n", l1.err);
		#endif
		return -1;
	}
	//Else continue

	//Continuously get distance measurement from Lidar
	while(l1.err >= 0){
		dist = l1.getDistance();
		#ifdef DEBUG
			printf("Range: %d cm\n", dist);
		#endif
		usleep(100000);
	}
	return 0;
}
