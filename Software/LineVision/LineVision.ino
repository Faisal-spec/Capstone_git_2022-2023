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
  int fix;
  int fixq;
  int lat;
  int lon;
}
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

void BNO_collect(bool enable, bool power); //collect orientation data from the BNO055 sensor 

void DATA_output(bool enable); //this function will output all packaged data over the raadio (p-out)

void POW_watchdog(bool* power_good, bool* charge_good); //this will check the status of the charge controller. 

#define AHT_EN true
#define BNO_EN true
#define GPS_EN true
#define ADXL_EN true
#define XBEE_EN true

//mos control
bool gps_power = true;
bool bno_power = true;
bool xbee_power = true;

//Power data pins 
#define PGOOD 19 //to have an integer pin number
#define CHG 18

//Power mos pins
#define GPS_MOS 7
#define BNO_MOS 8
#define XBEE_MOS 6



/**
 * @brief 
 * Code to setup all the sensors and communication devices
 */
void setup(void){
  Serial.begin(115200); //Start USB communication
  #ifdef test 
      while(!p_out); //wait for usb connection
  #endif
  if(XBEE_EN){
    pinMode(XBEE_MOS, OUTPUT);
    digitalWrite(XBEE_MOS, HIGH); //this will turn the XBEE on before initialization 
  }
  if(GPS_EN){
    pinMode(GPS_MOS, OUTPUT);
    digitalWrite(GPS_MOS, HIGH);
    delay(200);
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
    pinMode(BNO_MOS, OUTPUT);
    digitalWrite(BNO_MOS, HIGH);
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
  //need to read if there is input over the radio, 
  String in; //used for serial input
  if (p_out.available()){
    in = p_out.read(); //this will contain commands. 
  }
  else{
    in = "";
  }


  if(in == ""){ //this is the main loop that will run most of the time. 
    ADXL_collect(ADXL_EN);
    AHT_collect(AHT_EN);
    BNO_collect(BNO_EN, bno_power);
    GPS_collect(GPS_EN, gps_power);
  }
  else if(in == "SETUP"){

  }
  else{
    p_out.write("Critical exception");
    while(1);
  }
  ADXL_collect(ADXL_EN);
  AHT_collect(AHT_EN);
  BNO_collect(BNO_EN, bno_power);
  //GPS_collect(GPS_EN, /*Need to add the boolean for powering gps based on time*/);
  


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
    *r_temp = temp.temperature;
    *r_humid = humidity.relative_humidity;
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
void BNO_collect(bool enable, bool power){
  unsigned long tStart = micros();
  sensors_event_t orientationData , linearAccelData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  //  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  xPos = xPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
  yPos = yPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;

  // velocity of sensor in the direction it's facing
  headingVel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);
  //populating struct phase
  bbo.head = orientationData.orientation.x; //populate struct with heading
  bbo.speed = headingVel; //populate struct with speed
  bbo.xpo = xPos; //populate struct with xPosition
  bbo.ypo = yPos; //populate struct with yPosition

 /* if (printCount * BNO055_SAMPLERATE_DELAY_MS >= PRINT_DELAY_MS) {
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
  }*/


  while ((micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000))
  {
    //poll until the next sample is ready
  }
  return bbo; //i am assuming we are returing the struct to print later.
}

/**
 * @brief GPS_collect function for parsing GPS data
 * 
 */
void GPS_collect(bool enable, bool power){
  if(enable == 1){
    //float outs[3];
    gpsout outss;
    if(power == 1){
      outss.set = 1;
      if(GPS.fix){
        //outs[0] = GPS.fix;
        //outs[1] = GPS.fixquality;
        //outs[2] = GPS.latitudeDegrees;
        //outs[3] = GPS.longitudeDegrees;
        outss.fix = GPS.fix;
        outss.fixq = GPS.fixquality;
        outss.lat = GPS.latitudeDegrees;
        outss.lon = GPS.longitudeDegrees;
      }
      return outss;
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



/**
 * @brief Function to print all data out over the XBEE radio
 * 
 */
void DATA_output(bool enable){
  //This funciton needs to package all of the data from sensors and put it onto the p_out serial port. 
}

/**
 * @brief Function to check the status of the charger
 * 
 */
void POW_watchdog(bool* power_good, bool* charge_good){

}