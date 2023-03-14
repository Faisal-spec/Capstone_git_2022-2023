/**
 * @file LineVision.ino
 * @author Rylan Moore (rylan.moore@colorado.edu)
 * @brief 
 * @version 0.1
 * @date 2023-01-31
 * 
 * @copyright Copyright (c) 2023
 * 
 */

/**
 * @note
 * Board config:
 * MattairTech MT-D21E revB
 * Print & String use auto-promoted
 * config.h diabled
 * Internal oscillator 
 * 732.4Hz
 * SAMD21E18A
 * 8KB_BOOTLOADER
 * TWO_UART_ONE_WIRE_ONE_SPI
 * CDC_ONLY
 */
//TODO
/*
 * Make a macro for selecting which sensors are enabled in code 
 * 
 * Add different states for setup and normal operation and timeout after a defined value
 * 
 * Setup a string format for sending data over the radio (called p_out)
 * 
 */

/**
 * @brief
 * Defines below
*/
#include <Wire.h>
#include <math.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_ZeroFFT.h"
#include <Adafruit_BNO055.h>
#include <string.h>

//for testing hardware
#define test


// Hardware UART layout. Serial0/serial is the native USB
#define GPSSerial   Serial1
#define XBEESerial  Serial2

#ifdef test //this will change the default output port based on if hardware testing is running with USB connected. 
    #define p_out Serial
#endif 
#ifndef test
    #define p_out XBEESerial
#endif

struct gpsout{
  int set;
  int fixx;
  int fixq;
  int latt;
  int lonn;
};

gpsout outss;
// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis();

//Start the i2c and analog devices. 
Adafruit_AHTX0 aht;

double xPos = 0, yPos = 0, headingVel = 0;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board
uint16_t PRINT_DELAY_MS = 500; // how often to print the data
uint16_t printCount = 0; //counter to avoid printing every 10MS sample

//velocity = accel*dt (dt in seconds)
//position = 0.5*accel*dt^2
double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

#define X_in  2
#define Y_in  3
#define Z_in  4
#define ADC_bits 12
#define N_samples 2048
#define sample_rate 512
float mg_mv = 1 / .553; //mG per step in the 12 bits
//short ADXL_data[3][N_samples];
q15_t mag_data[N_samples];

//functions
int ADXL_collect(bool enable); //Collect data from the ADXL and run fft

int AHT_collect(bool enable); //Collect data from the AHT 

void GPS_collect(bool enable, bool power); //collect data from the gps, have a tag that says if gps is running or not. Once per day.

void BNO_collect(bool enable); //collect orientation data from the BNO055 sensor 

void DATA_output(bool enable); //this function will output all packaged data over the raadio (p-out)


#define AHT_EN true
#define BNO_EN true
#define GPS_EN true
#define ADXL_EN true


/**
 * @brief 
 * Code to setup all the sensors and communication devices
 */
void setup(void){
  Serial.begin(115200); //Start USB communication
  #ifdef test 
      while(!p_out); //wait for usb connection
  #endif

  if(GPS_EN){
    GPSSerial.begin(115200);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
    // Request updates on antenna status, comment out to keep quiet
    GPS.sendCommand(PGCMD_ANTENNA); 
  }
  XBEESerial.begin(9600);
  p_out.println("START");

  //setup the ADC for ADXL
  analogReadResolution(ADC_bits);
  //setup i2c for AHT
  if(!aht.begin() && AHT_EN){
    p_out.print("Could not find AHT20");
  }

  if(!bno.begin() && BNO_EN){
    p_out.print("Could not find BNO");
    while(1){}
  }
  //GPS setup


  pinMode(5, OUTPUT);

  delay(1000);
  p_out.println("End of setup");



}

/**
 * @brief 
 * 
 */
void loop(){
  ADXL_collect(ADXL_EN);
  AHT_collect(AHT_EN);
  BNO_collect(BNO_EN);
  GPS_collect(GPS_EN, /*Need to add the boolean for powering gps based on time*/);
  
  digitalWrite(5, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(5, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}

/**
 * @brief Get_temp function for AHT 
 * 
 * @param r_temp 
 * @param r_humid 
 * @return int 
 */
int AHT_collect(bool enable){
    long start = micros();
    //int tempp[4];
    //int hum[4];
    float temp_1 = 0;
    float hum_1 = 0;
    ///
    sensors_event_t humidity, temp;
    bool ret = aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
    //tempp[0] = temp.temperature;  //populate tempp with the current temperature data
    //hum[0] = humidity.relative_humidity; //populate humidity with the current data
    temp_1 = temp_1 + temp.temperature;
    hum_1 = hum_1 + humidity.relative_humidity;
    ///
    delay(500);
    //sensors_event_t humidity, temp;
    ret = aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
    //tempp[1] = temp.temperature;
    //hum[1] = humidity.relative_humidity;
    temp_1 = temp_1 + temp.temperature;
    hum_1 = hum_1 + humidity.relative_humidity;
    ///
    delay(500);
    //sensors_event_t humidity, temp;
    ret = aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
    //tempp[2] = temp.temperature;
    //hum[2] = humidity.relative_humidity;
    temp_1 = temp_1 + temp.temperature;
    hum_1 = hum_1 + humidity.relative_humidity;
    ///
    delay(500);
    //sensors_event_t humidity, temp;
    ret = aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
    //tempp[3] = temp.temperature;
    //hum[3] = humidity.relative_humidity;
    temp_1 = temp_1 + temp.temperature;
    hum_1 = hum_1 + humidity.relative_humidity;
    ///
    delay(500);
    //sensors_event_t humidity, temp;
    ret = aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
    //tempp[4] = temp.temperature;
    //hum[4] = humidity.relative_humidity;
    temp_1 = temp_1 + temp.temperature;
    hum_1 = hum_1 + humidity.relative_humidity;
    ///
    //temp_1 = (tempp[0]+tempp[1]+tempp[2]+tempp[3]+tempp[4])/5;
    //hum_1 = (hum[0]+hum[1]+hum[2]+hum[3]+hum[4])/5;
    temp_1 = temp_1/5 ;
    hum_1 = hum_1/5 ; 
    long end = micros();
    return temp_1,hum_1;
}

/**
 * @brief ADXL_collect function for ADXL327
 * 
 * @param num_samples 
 * @param sample_rate 
 * @return int 
 */
int ADXL_collect(bool enable){
  Serial.println("ADXL data");
  int start = micros(); //take start time for analysis
  int ref = 2048;
  int delay_us = 1000000 / sample_rate; //number of us between samples to align the data
  int32_t dp[3] = {0,0,0};
  for(int i = 0; i < N_samples; i++){ //fill the entire data array
      int current = micros();
      while(micros() - current < delay_us){} //collect data at the proper sampling rate 
      for(int j = 0; j<3; j++){
          dp[0] += analogRead(X_in);
          dp[1] += analogRead(Y_in);
          dp[2] += analogRead(Z_in);
      }
      for(int j = 0; j<3; j++){
          dp[j] = dp[j] / 3;
      }

      mag_data[i] = int16_t (ref - sqrt(sq(dp[0])+sq(dp[1])+sq(dp[2])));
  }
    
  // for(int i = 0; i<N_samples; i++){
  //     mag_data[i] = sqrt((ADXL_data[0][i])^2+(ADXL_data[1][i])^2+(ADXL_data[2][i])^2);
  // }
  ZeroFFT(mag_data, N_samples);
  return micros() - start;
}


/**
 * @brief 
 * 
 */
void BNO_collect(bool enable){
  unsigned long tStart = micros();
  sensors_event_t orientationData , linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  //  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  xPos = xPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
  yPos = yPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;

  // velocity of sensor in the direction it's facing
  headingVel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);

  if (printCount * BNO055_SAMPLERATE_DELAY_MS >= PRINT_DELAY_MS) {
    //enough iterations have passed that we can print the latest data
    p_out.print("Heading: ");
    p_out.println(orientationData.orientation.x);
    p_out.print("Position: ");
    p_out.print(xPos);
    p_out.print(" , ");
    p_out.println(yPos);
    p_out.print("Speed: ");
    p_out.println(headingVel);
    p_out.println("-------");

    printCount = 0;
  }
  else {
    printCount = printCount + 1;
  }



  while ((micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000))
  {
    //poll until the next sample is ready
  }
}

/**
 * @brief GPS_collect function for parsing GPS data
 * 
 */
void GPS_collect(bool enable, bool power){
  if(enable == 1){
    //float outs[3];
    if(power == 1){
      outss.set = 1;
      if(GPS.fix){
        //outs[0] = GPS.fix;
        //outs[1] = GPS.fixquality;
        //outs[2] = GPS.latitudeDegrees;
        //outs[3] = GPS.longitudeDegrees;
        outss.fixx = GPS.fix;
        outss.fixq = GPS.fixquality;
        outss.latt = GPS.latitudeDegrees;
        outss.lonn = GPS.longitudeDegrees;
      }
      return outss;
    }
    //power is not on so return previous GPS fix
    if(outss.set = 1){
      return outss;
    }
    return -1;
  }
}

/**
 * @brief Function to print all data out over the XBEE radio
 * 
 */
void DATA_output(bool enable){
  //This funciton needs to package all of the data from sensors and put it onto the p_out serial port. 
}