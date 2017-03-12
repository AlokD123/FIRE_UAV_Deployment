#include "lidar_lite.h"
#include <cstdio>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <pthread.h>
#include <pigpio.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <Python.h>
#include <stdlib.h>
//#include "/home/pi/Desktop/git/cpython/Include/Python.h"
//#include "/usr/include/linux/i2c.h"

/*Python script*/
//#define PYTHON_SCRIPT_NAME	"GetData_Ultra+Gas"	//NOT A DEFINITION. This must be changed manually.
//#define PYTHON_SCRIPT_PATH	'/home/pi/Desktop'	//NOT A DEFINITION. This must be changed manually.


/*Servo Constants*/
//#define DEAD_TIME_SWEEP_PROD	500*180					//As sweep angle decreases, dead time should increase??
#define DEAD_TIME				500					 	//Minimum servo refresh time (in us).
#define FREQUENCY				50 //=1/20000us 		//Frequency of PWM for servo (NOT same as width at 180deg position)
#define MIN_POS_WIDTH			1000 					//Minimum pulse width (us)
#define MAX_POS_WIDTH			2000 					//Maximum pulse width (us)
#define WIDTH_CHANGE_RATE		(MAX_POS_WIDTH-MIN_POS_WIDTH)/(180-0)	//Rate of change of marking width of PWM with position (angle)

#define MIN_POS			0					//Minimum position of orientation
#define MAX_POS			10					//Maximum position of orientation

/*Other Sensor Constants*/
#define GAS_THRESHOLD_PRCNT		20			//% value above which gas (smoke) defined to be detected
#define ULTRA_MAX_DIST			500			//Experimentally determined max reliable distance for the ultrasonic sensor (cm)

/*Macros*/
//Convert servo position to scaled duty cycle
#define PWM_MARK_WIDTH(POS)		(MIN_POS_WIDTH + WIDTH_CHANGE_RATE*POS)	//Get pulse width (us) for a position
#define PWM_MARK_WIDTH_SEC(POS)	PWM_MARK_WIDTH(POS) //*1/1000000	//Pulse width (sec)
#define DUTY_CYCLE(POS)			PWM_MARK_WIDTH_SEC(POS)*FREQUENCY	//Convert to duty cycle (decimal)
#define SCALED_DUTY_CYCLE(POS)	DUTY_CYCLE(POS) //*1000000		//Scale duty cycle by 1000000(1M) (as required)

/*Pin definitions*/
#define SERVO_PIN	18


/*Errors*/
#define GPIO_NO_INITIALIZE	-1
#define LIDAR_NO_INITIALIZE	-2
#define LIDAR_ERROR			-3
#define NO_PYTHON_SCRIPT	-4
#define PYTHON_SCRIPT_ERR	-5
#define OTHER_ERROR			-6 //See errno

/*States*/
#define DEBUG
//#define QUIT							//<---- Keep this (usually) commented!
#define EXIT_COND1	1						//Continuous run
#define EXIT_COND2	k<2550					//Alternate exit condition: finite time
#define EXIT_COND	EXIT_COND1
//#define PROCESS_TIMING
#define STDOUT_SEPARATOR


/*Functionalities*/
#define WEIGHTED_REPLACE_CENTER				//Replace data point in center with weighted sum of ultrasonic and LIDAR (depending on smoke density) when ultra_dist < ULTRA_MAX_DIST && at half-way angle
#define USE_ULTRA_BEST_GUESS			//When big discrepancy between ultrasonic and LIDAR, use value of ultrasonic sensor to determine whether too close rather than the LIDAR (i.e. when too smokey and all reflecting)
#define ADD_MAG_OFFSET						//Add offset in UAV orientation, as measured by the magnetometer  //****TO DECIDE!!
#define TOGGLE_LED							//Toggle LED ON when object too close, and OFF when not

#define W_LIDAR								//If LIDAR connected...
//#define W_ULTRA_AND_GAS						//If ultrasonic and gas sensors connected...
#define DATA_SEPARATION 					//General separator to allow for parsing <---- Gas sensor data goes here
#ifdef DATA_SEPARATION
	//By default, send binary value for whether gas (smoke) is detected (-2) or not (-1).
	//#define RAW_GAS_DENSITY				//Otherwise, send raw gas sensor data
#endif


/* Variables */
int pos=0;				//Position of servo (angle in degrees 0->180)
int lidar_dist=0;		//Lidar measured range (in cm)
int ultra_dist=-1;		//Ultrasonic measured range (in cm)
int dist=0;				//Best estimate range of objects in front, accounting for smoke (combination of LIDAR and ultrasonic) (in cm)
int gas_density=-2;		//Estimate of gas density (%)
bool obst_det=0;		//Estimate of whether obstacle detected (1) or not (0)
bool turnCW=1;			//Holds direction of rotation (1=CW, 0=CCW)
uint32_t startTime, endTime, processTime; //For timing process

Lidar_Lite LIDAR1=Lidar_Lite(1);
int SERIAL_ID=0;
bool GPIO_STARTED=0;
bool LIDAR_STARTED=0;

//Routine to quit, closing comm channels
void quit(int error){
	if(GPIO_STARTED){
		gpioWrite(SERVO_PIN,0);
		gpioTerminate();
	}
	if(LIDAR_STARTED){
		LIDAR1.~Lidar_Lite();
	}
	if(SERIAL_ID!=0){
		serialClose(SERIAL_ID);
	}
	#ifdef DEBUG
		printf("Error Code = %d. Quitting\n\n",error);
	#endif
	exit(error);
}

//Initialization of comm channels
void initializeSerial(){
	if( (SERIAL_ID=serialOpen("/dev/ttyAMA0",115200))<0 ){
		#ifdef DEBUG
			printf("Unable to open serial port. %s", strerror(errno));
		#endif
		quit(OTHER_ERROR);
	}
}
int connectLidar(){
	int x=LIDAR1.connect();
	LIDAR_STARTED=0;
	if (x< 0){		//If error connect, exit
		#ifdef DEBUG
			printf("Error connecting: %d\n", LIDAR1.getError());
			quit(LIDAR_NO_INITIALIZE);
		#endif
	}else{
		LIDAR_STARTED=1;
		#ifdef DEBUG
			printf("Connected to Lidar\n");
		#endif
	}
	return x; //Continue w/ or w/o LIDAR
}
void intializeGPIO(){
	if( gpioInitialise()<0 ){
		#ifdef DEBUG
			printf("pigpio failed to initialize.\n");
		#endif
		quit(GPIO_NO_INITIALIZE);
	}
	GPIO_STARTED=1;
}

//Sensor fusion algorithm
int fuseData(int lidar_dist,int ultra_dist,int gas_density,int pos){
	//At half-way angle...
	//Send fused sensor data
	/*
	a) for ultra_dist<ULTRA_MAX_DIST, replace center pt of sweep with...
		-> #ifdef USE_ULTRA_BEST_GUESS: ultrasonic value (when heavy smoke present)	--> reason: to account for large discrepancy
		-> LIDAR (smoke not present)												--> reason: ultrasonic is more variable
		-> #ifdef WEIGHTED_REPLACE_CENTER: weighted average (when some smoke present) --> reason: LIDAR is affected by smoke
	b) for ultra_dist>ULTRA_MAX_DIST, all Lidar (NO CHANGE)
	*/
}

void getUltraNGasData(){
	#ifdef DEBUG
		printf("Running Python script to get ultrasonic and gas sensor measurements.\n");
	#endif
	// Initialize the Python interpreter.
	Py_Initialize();
	// Create some Python objects that will later be assigned values.
	PyObject *pName, *pModule, *pDict, *pFunc, *pArgs, *pValue;
	// Set Python path
	PyRun_SimpleString("import sys");
	PyRun_SimpleString("sys.path.append('/home/pi/Desktop')");

	// Convert the file name to a Python string.
	pName = PyString_FromString("GetData_Ultra+Gas");
	// Import the file as a Python module.
	pModule = PyImport_Import(pName);
	if(pModule == NULL){
		#ifdef DEBUG
			printf("No Python Script!\n");
		#endif
		quit(NO_PYTHON_SCRIPT);
	}
	// Create a dictionary for the contents of the module.
	pDict = PyModule_GetDict(pModule);
	// Get the add method from the dictionary.
	pFunc = PyDict_GetItemString(pDict, "main");
	

	// Call the function with the arguments.
	PyObject* pResult = PyObject_CallObject(pFunc, NULL);
	// Print a message if calling the method failed.
	if(pResult == NULL)   printf("Method failed.\n");

	//Parse values
	PyObject* tupleItem1 = PyTuple_GetItem(pResult,0);
	if(tupleItem1 == NULL){
		#ifdef DEBUG
			printf("No ultrasonic distance return value from Python script.\n");
		#endif
		quit(PYTHON_SCRIPT_ERR);
	}else{
		ultra_dist=(int) PyInt_AsLong(tupleItem1);
	}
	PyObject* tupleItem2 = PyTuple_GetItem(pResult,1);
	if(tupleItem2 == NULL){
		#ifdef DEBUG
			printf("No gas sensor measurement return value from Python script.\n");
		#endif
		quit(PYTHON_SCRIPT_ERR);
	}else{
		gas_density=(int) PyInt_AsLong(tupleItem2);
	}

	// Destroy the Python interpreter.
	Py_DECREF(pModule);
	Py_DECREF(pName);
	Py_Finalize();
	#ifdef DEBUG
		printf("Ultra_dist = %d, Gas_density = %d\n",ultra_dist,gas_density);
	#endif
    return;
}



int main(){
	#ifdef QUIT
		printf("Quit right away\n");
		quit(0);
	#endif	


	//Start serial
		initializeSerial();

	#ifdef W_LIDAR
	//Initalize Lidar-Lite
		int lidar_connection= connectLidar();		//Currently only set up for one Lidar module (LIDAR1)
		if(lidar_connection<0){				//Else continue w/o
			printf("No lidar, error: %d",lidar_connection);
		};
	#endif		

	#ifdef PROCESS_TIMING
		startTime=gpioTick();
	#endif

	//Initialize GPIO
		intializeGPIO();

	int k=0;
	pos=0; //Start at 0
	
	//Continuously get distance measurement and sweep Lidar
	while(EXIT_COND){
		//#ifdef ADD_MAG_OFFSET: add UAV orientation (pos) offset as detected by the magnetometer
		
		
		//Move motor... if LIDAR present
		#ifdef W_LIDAR
			gpioHardwarePWM(SERVO_PIN,FREQUENCY,SCALED_DUTY_CYCLE(pos));
		#endif

		//Get LIDAR distance (polling) if no errors
		#ifdef W_LIDAR
			if(LIDAR1.getError() == 0){
				lidar_dist = LIDAR1.getDistance();
				dist=lidar_dist;					//Use LIDAR range by default
				#ifdef DEBUG
					//printf("LIDAR distance: %d\n",lidar_dist);
				#endif
			}else{
				quit(LIDAR_ERROR);
			}
		#endif

		//Get ultrasonic distance (polling)
		#ifdef W_ULTRA_AND_GAS
			getUltraNGasData();	//Get measurements from gas and ultrasonic by calling Python script
		#endif
		
		#ifdef W_ULTRA_AND_GAS
			if(dist==0){			//i.e. if no LIDAR... use ultrasonic range
				dist=ultra_dist;
			}
			//Sensor fusion if BOTH present
			#ifdef W_LIDAR
				dist=fuseData(lidar_dist,ultra_dist,gas_density,pos);
			#endif
		#endif
		
		//Print data to serial port for transmission (system) or plotting (individual)
		#ifdef DEBUG
			#ifdef W_LIDAR
				printf("Range: %d cm	Motor Duty Cycle: %d\n", dist,gpioGetPWMdutycycle(SERVO_PIN));
			#endif
		#endif
		serialPrintf(SERIAL_ID,dist+"\n");

		//Delay to allow motor to finish turning (us)
		gpioDelay(DEAD_TIME*10);

		//Change direction upon reaching end
		//Also send gas sensor data (BINARY value)
		if(pos==MAX_POS) turnCW=0;
		if(pos==MIN_POS) turnCW=1;
		if((pos==MIN_POS) || (pos==MAX_POS)){
			#ifdef DATA_SEPARATION
				#ifdef STDOUT_SEPARATOR
					printf("-1\n");	
				#endif
				#ifdef W_ULTRA_AND_GAS
					#ifdef RAW_GAS_DENSITY
						serialPrintf(SERIAL_ID,gas_density+"\n");		//Option raw value (not default)
					#else
						if(gas_density <= GAS_THRESHOLD_PRCNT) serialPrintf(SERIAL_ID,"-1\n");	//Provide binary value indicator of smoke
						else if(gas_density > GAS_THRESHOLD_PRCNT) serialPrintf(SERIAL_ID,"-2\n");
					#endif
				#else
					serialPrintf(SERIAL_ID,"-3\n");					// Send data separator -3 (end position)
				#endif
			#endif
		}
		
		//Determine if object too close, based on threshold range. Stored in boolean. Will be later used to direct UAV.
		
		//#ifdef TOGGLE_LED: toggle LED if too close
		
		
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
	quit(OTHER_ERROR);
}