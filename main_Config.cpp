#include <include/lidar_lite.h>
#include <cstdio>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <pthread.h>
#include <pigpio.h>
#include <wiringPi.h>
#include <wiringSerial.h>


/*Servo Constants*/
//#define DEAD_TIME_SWEEP_PROD	500*180					//As sweep angle decreases, dead time should increase??
#define DEAD_TIME		500				 	//Minimum servo refresh time (in us).
#define FREQUENCY		50 //=1/20000us 			//Frequency of PWM for servo (NOT same as width at 180deg position)
#define MIN_POS_WIDTH		1000 					//Minimum pulse width (us)
#define MAX_POS_WIDTH		2000 					//Maximum pulse width (us)
#define WIDTH_CHANGE_RATE	(MAX_POS_WIDTH-MIN_POS_WIDTH)/(180-0)	//Rate of change of marking width of PWM with position (angle)

#define MIN_POS			0					//Minimum position of orientation
#define MAX_POS			10					//Maximum position of orientation

/*Macros*/
//Convert servo position to scaled duty cycle
#define PWM_MARK_WIDTH(POS)	(MIN_POS_WIDTH + WIDTH_CHANGE_RATE*POS)	//Get pulse width (us) for a position
#define PWM_MARK_WIDTH_SEC(POS)	PWM_MARK_WIDTH(POS) //*1/1000000	//Pulse width (sec)
#define DUTY_CYCLE(POS)		PWM_MARK_WIDTH_SEC(POS)*FREQUENCY	//Convert to duty cycle (decimal)
#define SCALED_DUTY_CYCLE(POS)	DUTY_CYCLE(POS) //*1000000		//Scale duty cycle by 1000000(1M) (as required)

/*Pin definitions*/
#define SERVO_PIN	18
#define ULTRA_PIN	SERVO_PIN


/*Errors*/
#define GPIO_NO_INITIALIZE	-1
#define LIDAR_NO_INITIALIZE	-2
#define LIDAR_ERROR		-3
#define OTHER_ERROR		-4 //See errno

/*States*/
//#define DEBUG
#define DATA_SEPARATION 	//<---- Gas sensor and ultrasonic data goes here
//#define QUIT
#define EXIT_COND1	k<2550
#define EXIT_COND2	1	
#define EXIT_COND	EXIT_COND2
//#define PROCESS_TIMING

#define W_LIDAR
#ifdef W_LIDAR
	#define LIDAR_WHILE_COND (LIDAR1.err >= 0)
#else
	#define LIDAR_WHILE_COND 1
#endif
#define W_ULTRA
#define W_GAS


/* Variables */
int pos=0;	//Position of servo (angle in degrees 0->180)
int dist=0;	//Lidar measured range (in cm)
bool turnCW=1;	//Holds direction of rotation (1=CW, 0=CCW)
uint32_t startTime, endTime, processTime; //For timing process

Lidar_Lite LIDAR1=Lidar_Lite(1);
int SERIAL_ID=0;
bool GPIO_STARTED=0;
bool LIDAR_STARTED=0;

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
	
	//Continuously get distance measurement from Lidar
	while(LIDAR_WHILE_COND && EXIT_COND){
		//Move motor
		gpioHardwarePWM(SERVO_PIN,FREQUENCY,SCALED_DUTY_CYCLE(pos));

		//Get LIDAR distance (polling)
		#ifdef W_LIDAR 
			dist = LIDAR1.getDistance();
		#endif
		#ifdef DEBUG
			printf("Range: %d cm	DC: %d\n", dist,gpioGetPWMdutycycle(SERVO_PIN));
		#endif
		serialPrintf(SERIAL_ID,dist+"\n");
		printf("%d,%d\n", dist,gpioGetPWMdutycycle(SERVO_PIN));


		//Delay to allow motor to finish turning (us)
		gpioDelay(DEAD_TIME*10);

		//Change direction upon reaching end
		if(pos==MAX_POS){
			turnCW=0;
			#ifdef DATA_SEPARATION
				#ifdef DEBUG
					printf("-1\n");	
				#endif
				serialPrintf(SERIAL_ID,"X\n");
			#endif
		}
		else if(pos==MIN_POS){
			turnCW=1;
			#ifdef DATA_SEPARATION
				#ifdef DEBUG
					printf("-1\n");	
				#endif
				serialPrintf(SERIAL_ID,"Y\n");
				
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