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
int ADXL_collect(void);
int Get_temp(float * r_temp, float * r_humid);
void GPS_collect(void);
void ADXL_print(void);



/**
 * @brief 
 * Code to setup all the sensors and communication devices
 */
void setup(void){
    Serial.begin(115200); //Start USB communication
    #ifdef test 
        while(!p_out); //wait for usb connection
    #endif

    GPSSerial.begin(115200);
    XBEESerial.begin(9600);
    p_out.println("START");

    //setup the ADC for ADXL
    analogReadResolution(ADC_bits);
    //setup i2c for AHT
    // if(!aht.begin()){
    //     p_out.print("Could not find AHT20");
    // }

    // if(!bno.begin()){
    //     p_out.print("Could not find BNO");
    //     while(1){}
    // }
    //GPS setup
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
    // Request updates on antenna status, comment out to keep quiet
    GPS.sendCommand(PGCMD_ANTENNA); 
    pinMode(5, OUTPUT);

  delay(1000);
  p_out.println("End of setup");



}

/**
 * @brief 
 * 
 */
void loop(){
  // float * temp;
  // float *humidity; //pointers for AHT output
  // digitalWrite(5, HIGH);
  // delay(15000);
  // digitalWrite(5, LOW);
  // int temp_succ = Get_temp(temp, humidity);
  // String data = "Temp: " + String(*temp, 3) + " Humid: " + String(*humidity, 3);
  // Serial.println();
  // ADXL_collect();

  // // //ADXL_print();
  GPS_collect();
  
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
int Get_temp(float * r_temp, float * r_humid){
    long start = micros();
    sensors_event_t humidity, temp;
    bool ret = aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
    *r_temp = temp.temperature;
    *r_humid = humidity.relative_humidity;
    long end = micros();
    return end - start;
}

/**
 * @brief ADXL_collect function for ADXL327
 * 
 * @param num_samples 
 * @param sample_rate 
 * @return int 
 */
int ADXL_collect(void){
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
  ADXL_print();
  return micros() - start;
}

/**
 * @brief 
 * 
 */
void ADXL_print(void){
  for(int i=0; i<N_samples/2; i++){
    
    //print the frequency
    p_out.print(FFT_BIN(i, sample_rate, N_samples));
    p_out.print(",");

    //print the corresponding FFT output
    p_out.println(mag_data[i]);
  }
  return;
}

/**
 * @brief 
 * 
 */
void BNO_collect(void){
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

// void printEvent(sensors_event_t* event) { //adafruit function for BNO. Not sure if this will be helpful
//   Serial.print(event->type);
//   double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
//   if (event->type == SENSOR_TYPE_ACCELEROMETER) {
//     x = event->acceleration.x;
//     y = event->acceleration.y;
//     z = event->acceleration.z;
//   }
//   else if (event->type == SENSOR_TYPE_ORIENTATION) {
//     x = event->orientation.x;
//     y = event->orientation.y;
//     z = event->orientation.z;
//   }
//   else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
//     x = event->magnetic.x;
//     y = event->magnetic.y;
//     z = event->magnetic.z;
//   }
//   else if ((event->type == SENSOR_TYPE_GYROSCOPE) || (event->type == SENSOR_TYPE_ROTATION_VECTOR)) {
//     x = event->gyro.x;
//     y = event->gyro.y;
//     z = event->gyro.z;
//   }

//   Serial.print(": x= ");
//   Serial.print(x);
//   Serial.print(" | y= ");
//   Serial.print(y);
//   Serial.print(" | z= ");
//   Serial.println(z);
// }

/**
 * @brief GPS_collect function for parsing GPS data
 * 
 */
void GPS_collect(void){
      // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) p_out.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    p_out.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000 && GPS.fix != 0) {
    timer = millis(); // reset the timer
    p_out.print("\nTime: ");
    if (GPS.hour < 10) { p_out.print('0'); }
    p_out.print(GPS.hour, DEC); p_out.print(':');
    if (GPS.minute < 10) { p_out.print('0'); }
    p_out.print(GPS.minute, DEC); p_out.print(':');
    if (GPS.seconds < 10) { p_out.print('0'); }
    p_out.print(GPS.seconds, DEC); p_out.print('.');
    if (GPS.milliseconds < 10) {
      p_out.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      p_out.print("0");
    }
    p_out.println(GPS.milliseconds);
    p_out.print("Date: ");
    p_out.print(GPS.day, DEC); p_out.print('/');
    p_out.print(GPS.month, DEC); p_out.print("/20");
    p_out.println(GPS.year, DEC);
    p_out.print("Fix: "); p_out.print((int)GPS.fix);
    p_out.print(" quality: "); p_out.println((int)GPS.fixquality);
    if (GPS.fix) {
      p_out.print("Location: ");
      p_out.print(GPS.latitude, 4); p_out.print(GPS.lat);
      p_out.print(", ");
      p_out.print(GPS.longitude, 4); p_out.println(GPS.lon);
      p_out.print("Speed (knots): "); p_out.println(GPS.speed);
      p_out.print("Angle: "); p_out.println(GPS.angle);
      p_out.print("Altitude: "); p_out.println(GPS.altitude);
      p_out.print("Satellites: "); p_out.println((int)GPS.satellites);
      p_out.print("Antenna status: "); p_out.println((int)GPS.antenna);
    }
  }
}
