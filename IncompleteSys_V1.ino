/*------------------Haptic Device Source Code----------*/
/*----------------------By Yazan Saleh-----------------*/
/*-----------------------20/Dec/2023-------------------*/
/*------------------------Libraries--------------------*/

#include "Arduino.h"      //Arduino Standard Library 
#include <Wire.h>
#include <SPI.h>
#include <math.h>         //Standard Math Library
//https://github.com/max22-/ESP32-BLE-MIDI
#include <BLEMidi.h>      //Midi over Bluetooth for ESP32 MCU Library
//https://github.com/aster94/SensorFusion
#include <SensorFusion.h> //Sensor Fusion Library for the IMU
//https://github.com/adafruit/Adafruit_LSM6DS
#include <Adafruit_LSM6DSO32.h> //LSM6DSO32 IMU Signal Acquisition library
#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>
//http://www.adafruit.com/products/2650
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
/*----GPIO PIN Declaration---*/


/*----Global Variables-------*/
float gx, gy, gz, ax, ay, az, mx, my, mz, temp; //IMU Variables
float pitch, roll, yaw; //Euler Angles 
float deltat; //Sensor Fusion Variable

/*System Outputs Sensor Values*/

int ccRoll ;
int ccPitch ;
int ccYaw ;
int ccTemp;
int ccAlt;
int ccHum;

/*---->initialization<-------*/
//FreeRTOS Task Handles:
TaskHandle_t TaskBME;
TaskHandle_t TaskIMU;
TaskHandle_t TaskBLE;

//Initialize Sensor Fusion and LSM6DSO32 Libraries 
SF fusion;
Adafruit_ICM20948 icm;
Adafruit_Sensor *icm_temp, *icm_accel, *icm_gyro, *icm_mag;

#define ICM20948_I2CADDR_DEFAULT 0x68 ///< ICM20948 default i2c address
#define ICM20948_MAG_ID 0x09          ///< The chip ID for the magnetometer
#define ICM20948_UT_PER_LSB 0.15 ///< mag data LSB value (fixed)

//Initialize Sensor BME280 Libraries 
#define SEALEVELPRESSURE_HPA (1013.25)
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
Adafruit_BME280 bme; // I2C
//
unsigned long previousTime,previousTime1,previousTime2,previousTime3,previousTime4;

/*-------Setup--------------*/
void setup() {


  //set the baud rate to 115200
	Serial.begin(115200);
    //Initialize Midi over BLE
  BLEMidiServer.begin("Yazan MIDI Node");


  //Wait until the IMU is connected over serial 
 //while (!Serial){
 //   delay(10); 
 // }

  
    if (!icm.begin_I2C()) {
    // if (!icm.begin_SPI(ICM_CS)) {
    // if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {
    Serial.println("Failed to find ICM20948 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("ICM20948 Found!");
  icm_temp = icm.getTemperatureSensor();
  icm_temp->printSensorDetails();

  icm_accel = icm.getAccelerometerSensor();
  icm_accel->printSensorDetails();

  icm_gyro = icm.getGyroSensor();
  icm_gyro->printSensorDetails();

  icm_mag = icm.getMagnetometerSensor();
  icm_mag->printSensorDetails();

    if (! bme.begin(0x77, &Wire)) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }
  bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,  // temperature
                    Adafruit_BME280::SAMPLING_X16, // pressure
                    Adafruit_BME280::SAMPLING_X1,  // humidity
                    Adafruit_BME280::FILTER_X16,
                    Adafruit_BME280::STANDBY_MS_0_5 );
    
    // suggested rate is 25Hz
    // 1 + (2 * T_ovs) + (2 * P_ovs + 0.5) + (2 * H_ovs + 0.5)
    // T_ovs = 2
    // P_ovs = 16
    // H_ovs = 1
    // = 40ms (25Hz)
    // with standby time that should really be 24.16913... Hz
    //delayTime = 41;


	//create a task on core 1 that will measure EMG signal
  // and execute task1Func() with priority 7
  //	xTaskCreatePinnedToCore(task1Func,"EMG",1000,NULL,1, &TaskEMG, 1); 																															
  //delay(500);
	//create a task on core 1 that will measure PPG signal
  // and execute task2Func() with priority 9
	xTaskCreatePinnedToCore( task2Func,"BME",	10000,NULL, 5, &TaskBME,1); 						
	delay(500);
  //create a task on core 1 that will measure IMU angles
  // and execute task3Func() with priority 8
	xTaskCreatePinnedToCore(task3Func, "IMU", 10000, NULL, 8, &TaskIMU, 1);
	delay(500);
  //create a task on core 0 that will send MIDI over BLE messages
  // and execute task4Func() with priority 10
  xTaskCreatePinnedToCore(task4Func, "BLE", 100000, NULL, 10, &TaskBLE, 1);
	delay(500);

}

void task2Func( void *pvParameters )
{
  for( ;; )
	{
     	previousTime2=millis();
	bme.takeForcedMeasurement(); // has no effect in normal mode
  ccTemp = mapConstrainedToMidi(int(bme.readTemperature()), -10, 50);
  ccAlt = mapConstrainedToMidi(int(bme.readAltitude(SEALEVELPRESSURE_HPA)), 0, 1500);
  ccHum= mapConstrainedToMidi(int(bme.readHumidity()), 0, 100);
  delay(50);//42

	}
}

void task3Func( void *pvParameters )
{
	for( ;; )
	{
  		previousTime3=millis();
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sensors_event_t mag;
  icm_temp->getEvent(&temp);
  icm_accel->getEvent(&accel);
  icm_gyro->getEvent(&gyro);
  icm_mag->getEvent(&mag);
    
      //write the accelerometer and gyroscope angles onto the a and g variables
      ax = accel.acceleration.x;
      ay = accel.acceleration.y;
      az = accel.acceleration.z;
      gx = gyro.gyro.x;
      gy = gyro.gyro.y;
      gz = gyro.gyro.z;
      mx = mag.magnetic.x;
      my = mag.magnetic.y;
      mz = mag.magnetic.z;
      deltat = fusion.deltatUpdate(); //update the sensor values into the sensor fusion algorithm 
    //  Serial.println(ax);
    //        Serial.println(gx);
    //              Serial.println(deltat);

      //mahony filtered is updated with acc/gyro values  
      fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);
      roll = fusion.getRoll(); //Calculate Roll angle
      pitch = fusion.getPitch(); //Calculate Pitch angle
      yaw = fusion.getYaw(); //Calculate Yaw angle
      //Serial.println(roll);

      //Map the euler angles to MIDI       
      ccRoll = mapConstrainedToMidi(roll, -180, 180);
      ccYaw = mapConstrainedToMidi(yaw, 0, 360);
      ccPitch= mapConstrainedToMidi(pitch, -90, 90);
      //    Serial.println(ccPitch);

delay(25);//15
    //delay(1000 / 208); //sampling period 208Hz or ~4.80769ms

	}
}

void task4Func( void *pvParameters )
{
	for( ;; )
	{
     	previousTime4=millis();
      if(BLEMidiServer.isConnected()) { 
        //control change (cc) with Midi over BLE
        //first parameter is channel, then controller and value
        //BLEMidiServer.controlChange(0, 6, ccEMG);
        //BLEMidiServer.controlChange(0, 2, ccPPG);
        BLEMidiServer.controlChange(0, 103, ccRoll);
        BLEMidiServer.controlChange(0, 104, ccPitch);       
        BLEMidiServer.controlChange(0, 105, ccYaw);
        BLEMidiServer.controlChange(0, 106, ccTemp);
        BLEMidiServer.controlChange(0, 107, ccAlt);       
        BLEMidiServer.controlChange(0, 108, ccHum);
       // Serial.print(ccRoll);
      }

    delay(25);//25
	}
}
void loop() {

}

/*----Sensor Value Mapping to Midi-----*/
int mapConstrainedToMidi(int value, int min, int max) {
  int mapped = map(value, min, max, 0, 127);
  return constrain(mapped, 0,127);      
}

