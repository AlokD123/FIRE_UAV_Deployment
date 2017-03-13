#!/usr/bin/env python2.7
# adapted from script by Alex Eames http://RasPi.tv
import time
import os
import subprocess
import smtplib
import string
import RPi.GPIO as GPIO
from time import gmtime, strftime

GPIO.setmode(GPIO.BCM)

#DEFINITIONS
#ADC channels
ULTRA_ADC_CH = 0
GAS_ADC_CH = 1

#Other definitions
adc_ch = [ULTRA_ADC_CH,GAS_ADC_CH] # Particular ADC channel to read                                 <------ MUST add second
reps = 10 # how many times to take each measurement for averaging
time_between_readings = 1 # seconds between clusters of readings
V_REF = 5.0 #reference voltage
ultra_conv_factor=1./(0.0098/2.54)      #Ultrasonic factor: cm/mV


# Define Pins/Ports
SPICLK = 22           # FOUR SPI ports on the ADC 
SPIMISO = 5
SPIMOSI = 6
SPICS = 26
SPICS2 = 16


# read SPI data from MCP3002 chip, 2 possible adc channels (0 & 1)
# this uses a bitbang method rather than Pi hardware spi
# modified code based on an adafruit example for mcp3008
def readadc(adc_ch, clockpin, mosipin, misopin, cspin):
    if ((adc_ch > 1) or (adc_ch < 0)):
        return -1
    if (adc_ch == 0):
        commandout = 0x6
    else:
        commandout = 0x7

    GPIO.output(cspin, True)

    GPIO.output(clockpin, False)  # start clock low
    GPIO.output(cspin, False)     # bring CS low

    commandout <<= 5    # we only need to send 3 bits here
    for i in range(3):
        if (commandout & 0x80):
            GPIO.output(mosipin, True)
        else:   
            GPIO.output(mosipin, False)
        commandout <<= 1
        GPIO.output(clockpin, True)
        GPIO.output(clockpin, False)

    adcout = 0
    # read in one empty bit, one null bit and 10 ADC bits
    for i in range(12):
        GPIO.output(clockpin, True)
        GPIO.output(clockpin, False)
        adcout <<= 1
        if (GPIO.input(misopin)):
            adcout |= 0x1

    GPIO.output(cspin, True)

    adcout /= 2       # first bit is 'null' so drop it
    return adcout

    
    
#MAIN FUNCTION: to get ultrasonic and gas sensor data

def main():

    reps=10
    #Set up ports
    GPIO.setup(SPIMOSI, GPIO.OUT)       # set up the SPI interface pins
    GPIO.setup(SPIMISO, GPIO.IN)
    GPIO.setup(SPICLK, GPIO.OUT)
    GPIO.setup(SPICS, GPIO.OUT)

    try:
            for adc_channel in adc_ch:      #adc_ch is the channel number
                adctot = 0
                # read the analog value
                for i in range(reps):       #Read same ADC repeatedly for # REPS
                    read_adc = readadc(adc_channel, SPICLK, SPIMOSI, SPIMISO, SPICS)
                    adctot += read_adc
                    #time.sleep(0.01)            #Minimum 11.5us limit for acquisition & conversion
                read_adc = adctot / reps / 1.0 # Take average value
                # print read_adc
    
                # convert analog reading to Volts = ADC * ( V_REF / 1024 )
                volts = read_adc * ( V_REF / 1024.0)
                # convert voltage to measurement
                if (adc_channel==ULTRA_ADC_CH): 
                    ultra_dist = volts * ultra_conv_factor
                    if ultra_dist < 50:         # Filtering to reduce effect of noise below 50cm
                        reps = 100
                    else:
                        reps = 10
                    print "\nUltrasonic distance: %d" %ultra_dist
                elif (adc_channel==GAS_ADC_CH): 
                    gas_val_percent = volts / V_REF *100
                    print "Gas density: %d\n" %gas_val_percent

            #time.sleep(time_between_readings)
        
    except KeyboardInterrupt:             # trap a CTRL+C keyboard interrupt
        GPIO.cleanup()
    GPIO.cleanup()

    return (ultra_dist, gas_val_percent)

# To call main function
if __name__ == '__main__':
        main()
