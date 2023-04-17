/**
 * @file LineVision.ino
 * @author Rylan Moore (rylan.moore@colorado.edu)
 * @brief Final software for the LineVision companion system. 
 * @version 1.0
 * @date 2023-04-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <Wire.h>
#include <Adafruit_BNO055.h>
//#include <utility/imumaths.h>

#include "Adafruit_ZeroFFT.h"
//#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_GPS.h>

#define GPSSerial Serial1 
#define XBEE Serial2

#define p_out Serial2
#define acc_en false

#define FFT_BUFFER_SIZE 10
#define MAG_THRESHOLD 10

#define GPSECHO false

//digital pins
#define X_in  2 //digital pins that the analog inputs are on
#define Y_in  3
#define Z_in  4
#define V_ref 3
#define ADC_bits 12 //SAMD21 ADC resolution

//Power mos pins
#define GPS_MOS 7
#define BNO_MOS 8
#define XBEE_MOS 6

#define PGOOD 19
#define CHARGE 18

//stuff for fft
#define N_samples 2048//4096
#define sample_rate 1024

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);
Adafruit_BNO055 bno = Adafruit_BNO055(55);
int IMU_delay = 100;
Adafruit_AHTX0 aht;


long timer = 0;
long timer2 = 0;
bool config = true;

void check_rx_buffer();
void collect_aht();
void collect_imu();
void collect_gps(bool setup);
void collect_pwr();
void collect_fft();
void transmit_pkt();

/**
 * @brief This struct will be used globally to store data from the sensors between functions and loops
 * 
 */
struct sensor_data{
    float gps_longitude;
    char gps_long_char;
    char gps_lat_char;
    float gps_latitude;
    float gps_elevation;
    float aht_temp;
    float imu_x;
    float imu_y;
    float imu_z;
    bool bq_charge;
    bool bq_power;
    float fft_f[FFT_BUFFER_SIZE];
    float fft_m[FFT_BUFFER_SIZE];
};

/**
 * @brief This struct will be used to hold global statuses for the system. 
 * 
 */
struct status{
    bool setup;
    int interval = 10; //default 10 minutes
    char cmd_buf[10];
    int timer;
    int timer2;
    int timer3;
    int timer4;
};

// what's the name of the hardware p_out port?


//420 mv / g 
// +/- 2G is +/- 840mV around 2048
float mg_mv = 1/.553; //constant for the accelerometer 
int done = 0;
int16_t mag_data_td[N_samples];
q15_t mag_data[N_samples];
// q15_t z_only[N_samples];
int sample_time[N_samples];
int last = 0;



sensor_data smart; //declare the data node
status control; //declare the control node

/**
 * @brief The setup function configures the ADC, enables high power devices, and starts communications 
 * 
 */
void setup(){
    analogReadResolution(12);

    REG_ADC_REFCTRL = ADC_REFCTRL_REFSEL_INTVCC1; //set vref for ADC to VCC
    // Average control 1 sample, no right-shift
    REG_ADC_AVGCTRL |= ADC_AVGCTRL_SAMPLENUM_1;
    // Sampling time, no extra sampling half clock-cycles
    REG_ADC_SAMPCTRL = ADC_SAMPCTRL_SAMPLEN(0);
    ADC->CTRLA.reg |= ADC_CTRLA_ENABLE; //set ADC to run in standby

    Serial.begin(115200);
    pinMode(GPS_MOS, OUTPUT);
    pinMode(XBEE_MOS,OUTPUT);
    pinMode(BNO_MOS, OUTPUT);
    pinMode(CHARGE,INPUT); //for charge
    pinMode(PGOOD,INPUT); //for pgood 
    digitalWrite(GPS_MOS, HIGH);
    digitalWrite(XBEE_MOS, HIGH);
    digitalWrite(BNO_MOS, HIGH);
    delay(200);
    GPS.begin(9600);
    XBEE.begin(115200);
    pinMode(5, OUTPUT);
    //Serial.println("IMU Unity test"); Serial.println("");
    //Init sensor

    //XBEE.println("Hello");  
    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // uncomment this line to turn on only the "minimum recommended" data
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
    // the parser doesn't care about other sentences at this time
    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
    // For the parsing code to work nicely and have time to sort thru the data, and
    // print it out we don't suggest using anything higher than 1 Hz

    // Request updates on antenna status, comment out to keep quiet
    GPS.sendCommand(PGCMD_ANTENNA);

    delay(1000);

    // Ask for firmware version
    GPSSerial.println(PMTK_Q_RELEASE);

    if(!aht.begin()){
        while(1);
    }
    if(!bno.begin()){
        while(1);
    }
    XBEE.println("END of setup");    
}

/**
 * @brief The loop function holds all info on the state of the system and calls all of the individual sensors in sub routines. 
 * 
 */
void loop(){
    //check_rx_buffer();
    if(config || millis() - timer < 2000000){
        if (config){ //on the first run of the loop
            timer = millis();
        }
        digitalWrite(XBEE_MOS, HIGH);
        digitalWrite(GPS_MOS, HIGH);
        digitalWrite(BNO_MOS, HIGH);
        config = false;
        collect_aht(); //bad
        collect_imu(); //ok
        collect_pwr(); //ok
        if(millis() - control.timer4 > 200){
            transmit_pkt();
            control.timer4 = millis();
        }

    }
    if(millis() - timer > 2000000 && smart.gps_longitude == 0){
       // while(smart.gps_longitude == 0.0){
        collect_gps(1);
        //if(millis() - control.timer4 > 200){
            transmit_pkt();
        //     control.timer4 = millis();
        // }
        
    }

    else if ((millis() - timer2) > control.interval*60*1000 && false){
        digitalWrite(BNO_MOS, HIGH);
        digitalWrite(GPS_MOS, HIGH);
        collect_aht();
        collect_imu();
        collect_gps(0);
        collect_pwr();
        collect_fft();
        transmit_pkt();
        digitalWrite(GPS_MOS, LOW);
        digitalWrite(BNO_MOS, LOW);
        timer2 = millis();
    }
}

/**
 * @brief check_rx_buffer checks to see if any external commands have come into the system over the radio that need to be processed. 
 * 
 */
void check_rx_buffer(){

}

/**
 * @brief collect_aht asks the AHT20 for a new datapoint that is then stored in the storage struct. 
 * 
 */
void collect_aht(){
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
    smart.aht_temp = temp.temperature;
    //XBEE.print(temp.temperature); XBEE.print(";");
    return;
}

/**
 * @brief collect_imu asks the BNO055 for a new datapoint on the absolute orientation. This data is then converted to an Euler vector and stored. 
 * 
 */
void collect_imu(){
    imu::Quaternion quat = bno.getQuat();

    double sqw = quat.w() * quat.w();
    double sqx = quat.x() * quat.x();
    double sqy = quat.y() * quat.y();
    double sqz = quat.z() * quat.z();

    double _x = quat.x();
    double _y = quat.y();
    double _z = quat.z();
    double _w = quat.w();

    smart.imu_x = 57.3* atan2(2.0 * (_x * _y + _z * _w), (sqx - sqy - sqz + sqw));
    smart.imu_y  = 57.3* asin(-2.0 * (_x * _z - _y * _w) / (sqx + sqy + sqz + sqw));
    smart.imu_z = 57.3* atan2(2.0 * (_y * _z + _x * _w), (-sqx - sqy + sqz + sqw));

    return;
}

/**
 * @brief The collect_gps function collects NEMA sentences from the GPS for long enough that the GPS location can be parsed and stored in the data struct.
 * 
 * @param setup 
 */
void collect_gps(bool setup){
    // if you want to debug, this is a good time to do it!
//     for(int i = 0; i<7; i++){
//         GPS.read();
//     }
//   // if you want to debug, this is a good time to do it!
// //   if (GPSECHO)
//     // if (c) p_out.print(c);
//   // if a sentence is received, we can check the checksum, parse it...
//   if (GPS.newNMEAreceived()) {
//     // a tricky thing here is if we print the NMEA sentence, or data
//     // we end up not listening and catching other sentences!
//     // so be very wary if using OUTPUT_ALLDATA and trying to print out data
//     //XBEE.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
//     if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
//       return; // we can fail to parse a sentence in which case we should just wait for another
//   }
    while(!GPS.fix){
        GPS.read();
        if (GPS.newNMEAreceived()) {
        // a tricky thing here is if we print the NMEA sentence, or data
        // we end up not listening and catching other sentences!
        // so be very wary if using OUTPUT_ALLDATA and trying to print out data
        //p_out.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
        if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
            return; // we can fail to parse a sentence in which case we should just wait for another
        }
    }
     

    
    // approximately every 2 seconds or so, print out the current stats
    if (millis() - control.timer3 > 2000) {
        control.timer3 = millis();
        // XBEE.print("LOng:");
        // XBEE.println(GPS.longitude);
        smart.gps_latitude = GPS.seconds;//GPS.latitudeDegrees;
        smart.gps_long_char = GPS.lon;
        smart.gps_lat_char = GPS.lat;
        smart.gps_longitude = GPS.longitudeDegrees;
        smart.gps_elevation = GPS.altitude;
    }
    return;
}

/**
 * @brief collect_pwr checks the two status pins on the charge controller and sets them to the data struct. These are digital singals. 
 * 
 */
void collect_pwr(){
  if (!digitalRead(CHARGE)){ //charge 
    smart.bq_charge = true;
  }
  else{
    smart.bq_charge = false;
  }

  if(!digitalRead(PGOOD)){ //pgood
    smart.bq_power = true;
  }
  else{
    smart.bq_power = false;
  }
}

/**
 * @brief collect_fft begins the process of collecting data from the ADXL327. The data is then run through the fft funciton and the output is loaded into 
 * the data struct, with peak frequencies and magnitudes. 
 * 
 */
void collect_fft(){
    int start = micros(); //take start time for analysis
    int16_t ref;
    int32_t dp [3] = {0,0,0};
    int delay_us = 1000000 / sample_rate; //number of us between samples to align the data
    for(int i = 0; i < N_samples; i++){ //fill the entire data array
        int current = micros();
        while(micros() - current < delay_us){} //collect data at the proper sampling rate 
        ref = 2048;//analogRead(V_ref) / 2; //This collects the center voltage
        int avg_curr = micros();
        for(int j = 0; j<3; j++){
            dp[0] += analogRead(X_in);
            dp[1] += analogRead(Y_in);
            dp[2] += analogRead(Z_in);
        }
        sample_time[i] = micros() - avg_curr; //this shows how long data collection off the pins takes 9 samples
        // z_only[i] = ref - (dp[2] / 3);
        for(int j = 0; j<3; j++){
            dp[j] = dp[j] / 3;
        }

        mag_data[i] = int16_t (ref - sqrt(sq(dp[0])+sq(dp[1])+sq(dp[2])));
        mag_data_td[i] = mag_data[i];
    }
    
    // for(int i = 0; i<N_samples; i++){
    //     mag_data[i] = sqrt((ADXL_data[0][i])^2+(ADXL_data[1][i])^2+(ADXL_data[2][i])^2);
    // }
    ZeroFFT(mag_data, N_samples);
    // ZeroFFT(z_only, N_samples);
    long tt = micros() - start;

    int current_buffer_index = 0;
    for(int i=2; i<N_samples/2; i++){
        if(mag_data[i] > MAG_THRESHOLD && current_buffer_index < FFT_BUFFER_SIZE){
            //print the frequency
            //Serial2.print(FFT_BIN(i, sample_rate, N_samples));
            smart.fft_f[current_buffer_index] = FFT_BIN(i, sample_rate, N_samples);
            //Serial.print(" Hz, Magnitude: ");
            //Serial2.print(",");
            //print the corresponding FFT output
            // Serial2.print(mag_data[i] * mg_mv);
            smart.fft_m[current_buffer_index] = mag_data[i] * mg_mv;
            // Serial2.print(",");
            // Serial2.print(z_only[i] * mg_mv);
            // Serial2.print(",");
            // Serial2.println(mag_data_td[i]);
            current_buffer_index++;
        }

    }
}

/**
 * @brief transmit_pkt takes all of the data from the data struct and prints it out over the XBEE in the format for the Unity simulation. 
 * 
 */
void transmit_pkt(){
    XBEE.print(smart.imu_x);
    XBEE.print(F(", "));
    XBEE.print(smart.imu_y);
    XBEE.print(F(", "));
    XBEE.print(smart.imu_z);
    XBEE.print(F(";"));
    XBEE.print(smart.aht_temp); XBEE.print(";");
    for(int i =0; i< FFT_BUFFER_SIZE; i++){
        if(smart.fft_f[i] > 0){
            XBEE.print(smart.fft_f[i]); XBEE.print('-'); XBEE.print(smart.fft_m[i]); XBEE.print(',');
        }
    }
//fft print
    XBEE.print(";"); //end of the fft part of data output. 

    XBEE.print(smart.gps_latitude, 4); XBEE.print(smart.gps_lat_char); XBEE.print(","); 
    XBEE.print(smart.gps_longitude, 4); XBEE.print(smart.gps_long_char); XBEE.print(",");
    XBEE.print(smart.gps_elevation, 4); XBEE.print(" Meters;");

    XBEE.print(smart.bq_charge);
    XBEE.print(';');
    XBEE.println(smart.bq_power);
}