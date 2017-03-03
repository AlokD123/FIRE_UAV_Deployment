#include "lidar_lite.h"
#include <cstdio>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <pthread.h>
#include <pigpio.h>
#include <wiringPi.h>
#include <wiringSerial.h>
//#include "/usr/include/linux/i2c.h"


/*Servo Constants*/
//#define DEAD_TIME_SWEEP_PROD	500*180					//As sweep angle decreases, dead time should increase??
#define DEAD_TIME				500					 	//Minimum servo refresh time (in us).
#define FREQUENCY				50 //=1/20000us 		//Frequency of PWM for servo (NOT same as width at 180deg position)
#define MIN_POS_WIDTH			1000 					//Minimum pulse width (us)
#define MAX_POS_WIDTH			2000 					//Maximum pulse width (us)
#define WIDTH_CHANGE_RATE		(MAX_POS_WIDTH-MIN_POS_WIDTH)/(180-0)	//Rate of change of marking width of PWM with position (angle)

#define MIN_POS			0					//Minimum position of orientation
#define MAX_POS			10					//Maximum position of orientation

/*Macros*/
//Convert servo position to scaled duty cycle
#define PWM_MARK_WIDTH(POS)		(MIN_POS_WIDTH + WIDTH_CHANGE_RATE*POS)	//Get pulse width (us) for a position
#define PWM_MARK_WIDTH_SEC(POS)	PWM_MARK_WIDTH(POS) //*1/1000000	//Pulse width (sec)
#define DUTY_CYCLE(POS)			PWM_MARK_WIDTH_SEC(POS)*FREQUENCY	//Convert to duty cycle (decimal)
#define SCALED_DUTY_CYCLE(POS)	DUTY_CYCLE(POS) //*1000000		//Scale duty cycle by 1000000(1M) (as required)

/*Pin definitions*/
#define SERVO_PIN	18
#define ULTRA_PIN	SERVO_PIN


/*Errors*/
#define GPIO_NO_INITIALIZE	-1
#define LIDAR_NO_INITIALIZE	-2
#define LIDAR_ERROR			-3
#define OTHER_ERROR			-4 //See errno

/*States*/
//#define DEBUG
//#define QUIT
#define EXIT_COND1	1						//Continuous run
#define EXIT_COND2	k<2550					//Alternate exit condition: finite time
#define EXIT_COND	EXIT_COND1
//#define PROCESS_TIMING


/*Functionalities*/
#define REPLACE_CENTER						//Replace data point in center with ultrasonic when ultra_dist < ____ cm AND 
#ifdef REPLACE_CENTER
	#define USE_ULTRA_BEST_GUESS			//When big discrepancy, use value of ultrasonic sensor to determine whether too close rather than the LIDAR (i.e. when too smokey and all reflecting)
#endif
#define REMOVE_MAG_OFFSET					//Subtract off the offset due to the magnetometer
#define TOGGLE_LED							//Toggle LED ON when object too close, and OFF when not

#define W_LIDAR								//If LIDAR connected...
/*
#ifdef W_LIDAR
	#define LIDAR_WHILE_COND ()
#else
	#define LIDAR_WHILE_COND 1
#endif
*/
#define W_ULTRA_AND_GAS							//If ultrasonic and gas sensors connected...
/*
#ifdef W_ULTRA_AND_GAS
	#define ULTRA_WHILE_COND _____ //<------- Condition to stop if 
#else
	#define LIDAR_WHILE_COND 1
#endif
*/
#define DATA_SEPARATION 					//General separator to allow for parsing <---- Gas sensor data goes here


/* Variables */
int pos=0;				//Position of servo (angle in degrees 0->180)
int lidar_dist=0;		//Lidar measured range (in cm)
int ultra_dist=0;		//Ultrasonic measured range (in cm)
int dist=0;			//Best estimate range of objects in front, accounting for smoke (combination of LIDAR and ultrasonic) (in cm)
int gas_density=0;		//Estimate of gas density (%)
bool turnCW=1;			//Holds direction of rotation (1=CW, 0=CCW)
uint32_t startTime, endTime, processTime; //For timing process

Lidar_Lite LIDAR1=Lidar_Lite(1);
int SERIAL_ID=0;
bool GPIO_STARTED=0;
bool LIDAR_STARTED=0;

//Routine to quit, closing comm channels
int quit(int error){
	if(GPIO_STARTED){
		gpioWrite(SERVO_PIN,0);
		gpioTerminate();
	}
	if(LIDAR_STARTED){
		LIDAR1.~Lidar_Lite();
		//LIDAR1=NULL;
	}
	if(SERIAL_ID!=0){
		serialClose(SERIAL_ID);
	}
	#ifdef DEBUG
		printf("\n\n");
	#endif
	return error;
}

//Initialization of comm channels
void initializeSerial(){
	if( (SERIAL_ID=serialOpen("ttyAMA0",115200))<0 ){
		#ifdef DEBUG
			printf("Unable to open serial port, %s", strerror(errno));
		#endif
		quit(OTHER_ERROR);
	}
}
int connectLidar(Lidar_Lite lidar){
	int x=lidar.connect();
	LIDAR_STARTED=0;
	if (x< 0){		//If error connect, exit
		#ifdef DEBUG
			printf("Error connecting: %d\n", lidar.err);
			quit(LIDAR_NO_INITIALIZE);
		#endif
	}else{
		LIDAR_STARTED=1;
	}
	return x; //Continue w/ or w/o LIDAR
}
void intializeGPIO(){
	if( gpioInitialise()<0 ){
		#ifdef DEBUG
			fprintf(stderr,"pigpio failed\n");
		#endif
		quit(GPIO_NO_INITIALIZE);
	}
	GPIO_STARTED=1;
}

//Sensor fusion algorithm
int fuseData(int lidar_dist,int ultra_dist){
	
}



int main(){
	#ifdef QUIT
		printf("Preemptive quit\n");
		quit(0);
	#endif	


	//Start serial
		initializeSerial();
	//Initalize Lidar-Lite
		int lidar_connection= connectLidar(LIDAR1);
		if(lidar_connection<0){				//Else continue w/o
			printf("No lidar, error: %d",lidar_connection);
		};
		

	#ifdef PROCESS_TIMING
		startTime=gpioTick();
	#endif

	//Initialize GPIO
		intializeGPIO();

	int k=0;
	pos=0; //Start at 0
	
	//Continuously get distance measurement and sweep Lidar
	while(EXIT_COND){
		//Move motor... if LIDAR present
		#ifdef W_LIDAR
			gpioHardwarePWM(SERVO_PIN,FREQUENCY,SCALED_DUTY_CYCLE(pos));
		#endif

		//Get LIDAR distance (polling) if no errors
		#ifdef W_LIDAR
			if(LIDAR1.err >= 0){
				lidar_dist = LIDAR1.getDistance();
				dist=lidar_dist;					//Use LIDAR range by default
			}
		#endif
		//Get ultrasonic distance (polling)
		#ifdef W_ULTRA_AND_GAS
				ultra_dist = -1;					//<--------- GET measurements from gas and ultrasonic here
		#endif
		
		#ifdef W_ULTRA_AND_GAS
			if(dist==0){			//i.e. if no LIDAR... use ultrasonic range
				dist=ultra_dist;
			}
			//Sensor fusion if BOTH present
			#ifdef W_LIDAR
				dist=fuseData(lidar_dist,ultra_dist);
			#endif
		#endif
		
		
		#ifdef DEBUG
			printf("Range: %d cm	DC: %d\n", dist,gpioGetPWMdutycycle(SERVO_PIN));
		#endif
		serialPrintf(SERIAL_ID,dist+"\n");
		printf("%d,%d\n", dist,gpioGetPWMdutycycle(SERVO_PIN));  // <---------- Must remove


		//Delay to allow motor to finish turning (us)
		gpioDelay(DEAD_TIME*10);

		//Change direction upon reaching end
		//Also send gas sensor data
		if(pos==MAX_POS){
			turnCW=0;
			#ifdef DATA_SEPARATION
				#ifdef DEBUG
					printf("-1\n");	
				#endif
				#ifdef W_ULTRA_AND_GAS
					serialPrintf(SERIAL_ID,"gas\n");		//<---------- Replace with gas data
				#else
					serialPrintf(SERIAL_ID,"-3\n");		//<-------------- Send data separator -3 (max position)
				#endif
			#endif
		}
		else if(pos==MIN_POS){
			turnCW=1;
			#ifdef DATA_SEPARATION
				#ifdef DEBUG
					printf("-1\n");	
				#endif
				#ifdef W_ULTRA_AND_GAS
					serialPrintf(SERIAL_ID,"gas\n");		//<---------- Replace with gas data
				#else
					serialPrintf(SERIAL_ID,"-4\n");		//<-------------- Send data separator -4 (min position)
				#endif
			#endif
		}
		//Update next position
		if(turnCW==0) pos--;
		else pos++;
		k++;
	}
	#ifdef PROCESS_TIMING
		endTime=gpioTick();
		processTime=endTime-startTime;
		printf("Process time, with Lidar: %zu (us)\n\n",processTime);
	#endif

	gpioTerminate();
	quit(LIDAR_ERROR);
}