#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <pigpio.h>


#define DEAD_TIME	10 //minmum servo refresh time (in ms). TO CONFIRM if same as dead time
//PWM write
#define PWM_RANGE	255 //default 255, change using gpioSetPWMRange
#define FREQUENCY	50 //=1/20000us //Frequency of PWM for servo (equal to maximum width so PWM_RANGE=2400us=100%)
//Servo
#define SERVO_CCW	1000 // minimum pulse width (us)
#define SERVO_CW	2000 // maximum pulse width (us)

//PWM read
#define RESOLUTION	147/1000 			//Ultrasonic range resolution in ms/in
#define NUM_STEPS	340 //(int)(50/RESOLUTION)  	//resoution in terms of number of steps out of 50ms
#define ULTRA_PWM_PERIOD	0.050 			//Period of 50ms
#define ULTRA_PWM_FREQUENCY	(int) 1/ULTRA_PWM_PERIOD		
//PWM read 2: above plus...
#define INTERRUPT_TIMEOUT	100			//Interrupt, in ms, due to timeout (no level change in PWM signal). Pick 100ms for 50ms period
#define PWM_MARK_TIME_MS	pwmMarkTime/1000	//Duty cycle time in ms

#define SERVO_PIN	18
#define ULTRA_PIN	SERVO_PIN

#define SOFTWARE_PWM_TEST
//#define HARDWARE_PWM_TEST
//#define SERVO_TEST
//#define READ_ULTRA_TEST1
//#define READ_ULTRA_TEST2

#define DEBUG
#define DEBUG_ULTRA_READ

/* Variables */
int i, j;
int servoPWMread; //To confirm sending correct duty cycle (number out of PWM_RANGE)
float ultraRange; //Range read by ultrasonic sensor
int pwmSet; //Return value from setting ULTRA_PIN to PWM

volatile uint32_t startTime, endTime; //Starting time (us), corresponding to PWM rising edge
volatile uint32_t pwmMarkTime; //PWM mark time (us), giving duty cycle


/* ISR Handlers */
//ISR to get rising edge time
void pwmRisingEdgeISR(int gpio, int level, uint32_t tick){
	if(level==PI_TIMEOUT){
		#ifdef DEBUG_ULTRA_READ
			printf("Timeout\n");
		#endif
	}else{
		startTime=gpioTick();
		#ifdef DEBUG_ULTRA_READ
			printf("GotRise @ %zu\n",startTime);
		#endif
	}
}

//ISR to get falling edge time, and hence PWM duty cycle
void pwmFallingEdgeISR(int gpio, int level, uint32_t tick){
	if(level==PI_TIMEOUT){
		#ifdef DEBUG_ULTRA_READ
			printf("Timeout\n");
		#endif
	}else{
		endTime=gpioTick();		
		#ifdef DEBUG_ULTRA_READ
			printf("GotFall @ %zu\n",endTime);
		#endif
	}
}



/*Main: 4 tests... 2 servo, 2 ultrasonic*/
int main(){


if( gpioInitialise()<0 ){
	fprintf(stderr,"pigpio failed\n");
	return -1;
}


#ifdef SOFTWARE_PWM_TEST

//Start PWM using gpioPWM
gpioSetMode(SERVO_PIN,PI_OUTPUT);
gpioSetPWMfrequency(SERVO_PIN,FREQUENCY);

//Do PWM "revving" 10 times
for(j=0;j<10;j++){
	#ifdef DEBUG
		printf("\n\nThe %d-th revving\n\n",j);
	#endif
	for(i=0;i<PWM_RANGE;i++){
		gpioPWM(SERVO_PIN,i);
		#ifdef DEBUG
			servoPWMread = gpioGetPWMdutycycle(SERVO_PIN);
			printf("DutyCycle: %d\n",servoPWMread);
		#endif
		
		//gpioSleep(PI_TIME_RELATIVE,1,0);
		gpioDelay(DEAD_TIME); //10 us
	}

gpioWrite(SERVO_PIN,0);
}
#endif


/* Using hardware instead of software. Limited test (no revving) */
#ifdef HARDWARE_PWM_TEST

gpioSetMode(SERVO_PIN,PI_OUTPUT);
#define DUTY_CYCLE	1000000//=100000*0.0024*FREQUENCY = 100% //Value out of 1000000(1M)
#define SC		1000000
int x;

gpioHardwarePWM(SERVO_PIN,FREQUENCY,(1+0/100)*0.001*FREQUENCY*SC);
gpioDelay(10000);
while(1){
for(x=0;x<100;x++){
gpioHardwarePWM(SERVO_PIN,FREQUENCY,(1+x/100)*0.001*FREQUENCY*SC);
gpioDelay(10000);
}
for(x=100;x>0;x--){
gpioHardwarePWM(SERVO_PIN,FREQUENCY,(1+x/100)*0.001*FREQUENCY*SC);
gpioDelay(10000);
}
}

gpioWrite(SERVO_PIN,0);
#endif




#ifdef SERVO_TEST

//Start servo using gpioServo
gpioSetMode(SERVO_PIN,PI_OUTPUT);

//Do revving 10 times
for(j=0;j<10;j++){
	#ifdef DEBUG
		printf("\n\nThe %d-th revving\n\n",j);
	#endif
	for(i=SERVO_CCW;i<SERVO_CW;i++){
		gpioServo(SERVO_PIN,i);
		//printf("DC: %d",gpioGetPWMdutycycle(SERVO_PIN));
		gpioDelay(DEAD_TIME);
	}

gpioWrite(SERVO_PIN,0);
}
#endif







/* NOT FUNCTIONAL... only "gets", does NOT "read" */
#ifdef READ_ULTRA_TEST1

//Read from ultrasonic PWM
pwmSet = gpioSetMode(ULTRA_PIN,PI_ALT5)
printf("Success = %d\n\n",pwmSet);
//Set the expected number of steps
//Actual (real) number of steps is given in associated table (see pigpio C interface docs)
gpioSetPWMrange(ULTRA_PIN,NUM_STEPS);

//Read in 10 values
for(j=0;j<10;j++){
	ultraRange = (gpioGetPWMdutycycle(ULTRA_PIN)/NUM_STEPS) * ULTRA_PWM_PERIOD / RESOLUTION; //Range = duty cycle time / resolution = range in inches 
	#ifdef DEBUG
		printf("\nRange: %f\n",ultraRange);
	#endif
}
printf("\n");


#endif


#ifdef READ_ULTRA_TEST2
//Attach ISRs
printf("Rise ISR attach success = %d\n",!gpioSetISRFunc(ULTRA_PIN,RISING_EDGE,INTERRUPT_TIMEOUT,pwmRisingEdgeISR));
//printf("Fall ISR attach success = %d\n",!gpioSetISRFunc(ULTRA_PIN,FALLING_EDGE,INTERRUPT_TIMEOUT,pwmFallingEdgeISR));

//Start PWM read
gpioSetMode(ULTRA_PIN,PI_INPUT);
//Read in 10 values
for(j=0;j<10;j++){
	pwmMarkTime = endTime - startTime;
	ultraRange = PWM_MARK_TIME_MS / RESOLUTION; //Range (inches) = duty cycle time(ms) / resolution(ms/in)
	#ifdef DEBUG
		printf("\nstartTime: %zu us.... endTime: %zu us.... pwmMarkTime: %zu us....Range: %f inches\n",startTime,endTime,pwmMarkTime,ultraRange);
	#endif
}

//Try infinite loop to catch rising edges (falling edges are OK)
//PROBLEM HERE: can't catch rise fast enough, likely because gpioTick() takes too long
while(1);

gpioWrite(SERVO_PIN,0);
#endif

gpioTerminate();

return 0;

}
