#include "lidar_lite.h"
#define SLEEP_TIME		100
//Mode definition
#define DEFAULT_ACQ_MODE 	0x08 //Default: Only disabled quick termination
//Masks
#define OR_DISABLE_CALLER 	0x40 //OR-mask to block calling process (i.e. polling)
//#define OR_DISABLE_FILTER 	0x10 //OR-mask to disable measurement filter
#define AND_ENABLE_QUICK_TERM 	0xF7 //AND-mask to enable quick termination of measurement
#define OR_ALT_REF_CNT 		0x04 //OR-mask to choose alternate number of acquisitions per measurement

#define SINGLE_ACQ		0x01

#define DEBUG

using namespace std;

int i=0;

Lidar_Lite::Lidar_Lite (int bus){
  err = 0;
  adapter_num = bus;
  snprintf(filename, 19, "/dev/i2c-%d", adapter_num);
}

Lidar_Lite::~Lidar_Lite(void){
  #ifdef DEBUG
      printf("Ending Lidar-Lite Session\n");
  #endif
  if (i2c_bus > 0){
   int e = close(i2c_bus);
  }
}



/*Connect and Configure Lidar*/

int Lidar_Lite::connect( void ) {
  #ifdef DEBUG
      printf("Connecting to %s", filename);
  #endif
  i2c_bus = open(filename, O_RDWR);
  if (i2c_bus < 0){
    err = errno;
    #ifdef DEBUG
        fprintf(stderr,"Connect Error: %d", err);
    #endif
    return -1;
  }
  if (ioctl(i2c_bus, I2C_SLAVE, 0x62) < 0) {
    err = errno;
    #ifdef DEBUG
       printf("Bus Error: %d", err);
    #endif
    return -1;
  }

  //Additional configurations for LIDAR
  int e;
  //Delay between measurements=0.01s (default max 100Hz)
  e = writeAndWait(0x45,0x14);
  if (e < 0){
    return e;
  }
  //Setting for LIDAR to be polled for measurement
  e = writeAndWait(0x04,DEFAULT_ACQ_MODE|OR_DISABLE_CALLER);
  if (e < 0){
    return e;
  }
  //Enable quick termination too
  e = writeAndWait(0x04,(DEFAULT_ACQ_MODE|OR_DISABLE_CALLER) & AND_ENABLE_QUICK_TERM);
  if (e < 0){
    return e;
  }


  //Alternate number of acquisitions per measurement
  e = writeAndWait(0x04, ( (DEFAULT_ACQ_MODE|OR_DISABLE_CALLER) & AND_ENABLE_QUICK_TERM ) |OR_ALT_REF_CNT );
  if (e < 0){
    return e;
  }
  //Set single acq/meas
  e = writeAndWait(0x12,SINGLE_ACQ);
  if (e < 0){
    return e;
  }


//Not helping...
  //Disable measurement filter too
  //e = writeAndWait(0x04,DEFAULT_ACQ_MODE|OR_DISABLE_CALLER|OR_DISABLE_FILTER);
  //if (e < 0){
  //  return e;
  //}
  

  return 0;
}





int Lidar_Lite::writeAndWait(int writeRegister, int value){
  res = i2c_smbus_write_byte_data(i2c_bus, writeRegister, value);
  //usleep(SLEEP_TIME);
  if (res < 0){
    err = errno;
    printf("Write Error %d", err);
    return -1;
  } else {
    return 0;
  }
}

int Lidar_Lite::readAndWait(int readRegister){
  res = i2c_smbus_read_byte_data(i2c_bus, readRegister);
  //usleep(SLEEP_TIME);
  if (res < 0){
    err = errno;
    #ifdef DEBUG
       printf("Read Error: %d", err);
    #endif
    return -1;
  } else {
    return 0;
  }
}

int Lidar_Lite::getDistance( void ){
  int buf[2];
  int e = 0;

  	//if(i>=99){ //Every so often...
  	//  i=0;
  e = writeAndWait(0x00,0x04); //Set receiver bias correction
  	//}
  	//else{
  	//  i++;
  	//  e = writeAndWait(0x00,0x03); //Set no receiver bias correction
  	//}
  if (e < 0){
      return e;
  }  

  e = readAndWait(0x8f); //Get data HIGH byte
  if (e < 0){
    return e;
  } else {
    buf[0] = res;
  }
  e = readAndWait(0x10); //Get data LOW byte
  if (e < 0){
    return e;
  } else {
    buf[1] = res;
  }
  return (buf[0] << 8) + buf[1];

}

int Lidar_Lite::getError(void){
  return err;
}
