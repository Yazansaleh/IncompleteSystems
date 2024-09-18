/*------------------Haptic Device Source Code----------*/
/*----------------------By Yazan Saleh-----------------*/
/*-----------------------20/June/2024-------------------*/
/*------------------------Libraries--------------------*/

#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <SensorFusion.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BLEMIDI_Transport.h>
#include "Adafruit_MAX1704X.h"
#include <hardware/BLEMIDI_ESP32_NimBLE.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_task_wdt.h"
#include "esp_sleep.h"

#define DEBUG_MODE true

// Constants
#define ICM20948_I2CADDR_DEFAULT 0x68
#define SEALEVELPRESSURE_HPA (1013.25)
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define MIDI_CHANNEL 1
#define MIDI_DEVICE_NAME "Sensor 4"
#define BUTTON_PIN_1 42
#define BUTTON_PIN_2 45
#define BUTTON_PIN_3 6
#define BUTTON_PIN_4 1
#define WAKEUP_PIN_BITMASK 0X42  //(2^6 + 2^1) IN HEX

// Task Handles
TaskHandle_t TaskSOC, TaskBME, TaskIMU, TaskBLE, TaskSys, TaskBlink;
SemaphoreHandle_t blinkSemaphore = NULL;

// Sensor Objects
Adafruit_ICM20948 icm;
Adafruit_BME280 bme;  // I2C
Adafruit_Sensor *icm_temp, *icm_accel, *icm_gyro, *icm_mag;
SF fusion;
Adafruit_MAX17048 maxlipo;

// BLE MIDI Instance
BLEMIDI_CREATE_INSTANCE(MIDI_DEVICE_NAME, MIDI)
bool isConnected = false;

// Global Variables
float gx, gy, gz, ax, ay, az, mx, my, mz, temp;  //IMU Variables
float pitch, roll, yaw;                          //Euler Angles
float accelerationX = 0.0;
float accelerationY = 0.0;
float accelerationZ = 0.0;
float deltat;  //Sensor Fusion Variable
int batSOC;
// System Outputs Sensor Values
int ccRoll;
int ccPitch;
int ccYaw;
int ccAX;
int ccAY;
int ccAZ;
int ccTemp;
int ccAlt;
int ccHum;
int ccBatSOC;

//Variables for button toggling
const int buttonPins[] = { BUTTON_PIN_1, BUTTON_PIN_2, BUTTON_PIN_3, BUTTON_PIN_4 };  // Pin numbers for the four buttons
bool toggleStates[] = { false, false, false, false };                                 // Toggle states for each button

int buttonStates[] = { HIGH, HIGH, HIGH, HIGH };      // Current button states
int lastButtonStates[] = { HIGH, HIGH, HIGH, HIGH };  // Last button states

unsigned long lastDebounceTimes[] = { 0, 0, 0, 0 };  // Debounce timers for each button
unsigned long debounceDelay = 50;                    // Debounce delay (same for all buttons)



// Function Declarations
void initializeSensors();
void taskSOC(void *pvParameters);
void taskBME(void *pvParameters);
void taskIMU(void *pvParameters);
void taskBLE(void *pvParameters);
void taskSys(void *pvParameters);
void taskBlink(void *pvParameters);

int mapConstrainedToMidi(int value, int min, int max);

/*-------Setup--------------*/
void setup() {
  if (DEBUG_MODE) {
    Serial.begin(115200);
  }
  initializeSensors();
  MIDI.begin();

  // Initialize GPIOs for button input
  pinMode(BUTTON_PIN_1, INPUT);
  pinMode(BUTTON_PIN_2, INPUT);
  pinMode(BUTTON_PIN_3, INPUT);
  pinMode(BUTTON_PIN_4, INPUT);
  blinkSemaphore = xSemaphoreCreateBinary();

  // Enable wakeup by multiple buttons (all need to be pressed)
  vTaskDelay(1000);
  esp_sleep_enable_ext1_wakeup(WAKEUP_PIN_BITMASK, ESP_EXT1_WAKEUP_ALL_LOW);
  xSemaphoreGive(blinkSemaphore);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  BLEMIDI.setHandleConnected([]() {
    isConnected = true;
    digitalWrite(LED_BUILTIN, HIGH);
  });

  BLEMIDI.setHandleDisconnected([]() {
    isConnected = false;
    digitalWrite(LED_BUILTIN, LOW);
  });

  //esp_task_wdt_init(5, true); // Enable panic so system restarts if WDT is not fed
  xTaskCreatePinnedToCore(taskSOC, "SOC", 2048, NULL, 1, &TaskSOC, 1);
  xTaskCreatePinnedToCore(taskBME, "BME", 10000, NULL, 2, &TaskBME, 1);
  xTaskCreatePinnedToCore(taskIMU, "IMU", 20000, NULL, 3, &TaskIMU, 1);
  xTaskCreatePinnedToCore(taskBLE, "BLE", 50000, NULL, 4, &TaskBLE, 0);
  xTaskCreatePinnedToCore(taskSys, "Sys", 2048, NULL, 5, &TaskSys, 0);
  xTaskCreatePinnedToCore(taskBlink, "Sys", 2048, NULL, 6, &TaskBlink, 0);
}

void initializeSensors() {
  if (!icm.begin_I2C(ICM20948_I2CADDR_DEFAULT)) {
    handleError("ICM20948 Initialization Failed");
  }
  if (DEBUG_MODE) {
    Serial.println("ICM20948 Found!");
  }
  icm.enableAccelDLPF(true, ICM20X_ACCEL_FREQ_5_7_HZ);
  icm.setAccelRange(ICM20948_ACCEL_RANGE_2_G);
  /*icm.setAccelRateDivisor(4095);
      uint16_t accel_divisor = icm.getAccelRateDivisor();
      float accel_rate = 1125 / (1.0 + accel_divisor);
    */
  icm.setGyroRange(ICM20948_GYRO_RANGE_250_DPS);
  /*icm.setGyroRateDivisor(255);
      uint8_t gyro_divisor = icm.getGyroRateDivisor();
      float gyro_rate = 1100 / (1.0 + gyro_divisor);
    */
  icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);

  icm_temp = icm.getTemperatureSensor();
  icm_accel = icm.getAccelerometerSensor();
  icm_gyro = icm.getGyroSensor();
  icm_mag = icm.getMagnetometerSensor();
  /* Available cutoff frequencies:
      ICM20X_ACCEL_FREQ_246_0_HZ
      ICM20X_ACCEL_FREQ_111_4_HZ
      ICM20X_ACCEL_FREQ_50_4_HZ
      ICM20X_ACCEL_FREQ_23_9_HZ
      ICM20X_ACCEL_FREQ_11_5_HZ
      ICM20X_ACCEL_FREQ_5_7_HZ
      ICM20X_ACCEL_FREQ_473_HZ
    */

  if (!bme.begin(0x77, &Wire)) {
    handleError("BME280 Initialization Failed");
  }
  bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                  Adafruit_BME280::SAMPLING_X2,   // temperature
                  Adafruit_BME280::SAMPLING_X16,  // pressure
                  Adafruit_BME280::SAMPLING_X1,   // humidity
                  Adafruit_BME280::FILTER_X16,
                  Adafruit_BME280::STANDBY_MS_0_5);

  if (!maxlipo.begin()) {
    handleError("maxlipo Initialization Failed");
  }
  if (DEBUG_MODE) {
    Serial.print(F("Found MAX17048"));
    Serial.print(F(" with Chip ID: 0x"));
    Serial.println(maxlipo.getChipID(), HEX);
  }
}

void loop() {}

void taskSOC(void *pvParameters) {
  for (;;) {
    ccBatSOC = mapConstrainedToMidi(int(maxlipo.cellPercent()), 0, 127);
    if (DEBUG_MODE) {
      Serial.print(F("Batt Percent: "));
      Serial.print(maxlipo.cellPercent());
      Serial.println(" %");
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void taskBME(void *pvParameters) {
  for (;;) {
    //previousTime2=millis();
    bme.takeForcedMeasurement();  // has no effect in normal mode
    ccTemp = mapConstrainedToMidi(int(bme.readTemperature()), -10, 50);
    ccAlt = mapConstrainedToMidi(int(bme.readAltitude(SEALEVELPRESSURE_HPA)), 700, 1200);
    ccHum = mapConstrainedToMidi(int(bme.readHumidity()), 10, 80);
    ccBatSOC = mapConstrainedToMidi(int(maxlipo.cellPercent()), 0, 100);
    if (DEBUG_MODE) {
      Serial.print(F("Batt Percent: "));
      Serial.print(maxlipo.cellPercent());
      Serial.println(" %");
    }
    //delay(42);//42
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void taskIMU(void *pvParameters) {
  for (;;) {
    //previousTime3=millis();
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
    deltat = fusion.deltatUpdate();  //update the sensor values into the sensor fusion algorithm



    //mahony filtered is updated with acc/gyro values
    fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //else use the magwick, it is slower but more accurate
    roll = fusion.getRoll();                                            //Calculate Roll angle
    pitch = fusion.getPitch();                                          //Calculate Pitch angle
    yaw = fusion.getYaw();                                              //Calculate Yaw angle

    //Map the euler angles to MIDI
    ccRoll = mapConstrainedToMidi(roll, -180, 180);
    ccYaw = mapConstrainedToMidi(yaw, 0, 360);
    ccPitch = mapConstrainedToMidi(pitch, -90, 90);

    ccAX = mapConstrainedToMidi(ax, -100, 100);
    ccAY = mapConstrainedToMidi(ay, -100, 100);
    ccAZ = mapConstrainedToMidi(az, -100, 100);
    if (DEBUG_MODE) {
      // Serial.print("Accelerometer RAW: ");
      // Serial.print(ax);
      // Serial.print(", ");
      // Serial.print(ay);
      // Serial.print(", ");
      // Serial.println(az);

      // Serial.print("Gyro RAW: ");
      // Serial.print(gx);
      // Serial.print(", ");
      // Serial.print(gy);
      // Serial.print(", ");
      // Serial.println(gz);

      // Serial.print("Magno RAW: ");
      // Serial.print(mx);
      // Serial.print(", ");
      // Serial.print(my);
      // Serial.print(", ");
      // Serial.println(mz);

      // Serial.print("Roll: ");
      // Serial.print(roll);
      // Serial.print(", ");
      // Serial.print("Pitch: ");
      // Serial.print(pitch);
      // Serial.print(", ");
      // Serial.print("Yaw: ");
      // Serial.print(yaw);
      // Serial.print(", ");
      // Serial.print("Deltat: ");
      // Serial.println(deltat);

      // Serial.print("ccRoll: ");
      // Serial.print(ccRoll);
      // Serial.print(", ");
      // Serial.print("ccPitch: ");
      // Serial.print(ccPitch);
      // Serial.print(", ");
      // Serial.print("ccYaw: ");
      // Serial.println(ccYaw);
    }
    //delay(15);//15
    //delay(1000 / 208); //sampling period 208Hz or ~4.80769ms
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}


void taskBLE(void *pvParameters) {
  for (;;) {
    MIDI.sendControlChange(103, ccRoll, MIDI_CHANNEL);
    MIDI.sendControlChange(104, ccPitch, MIDI_CHANNEL);
    MIDI.sendControlChange(105, ccYaw, MIDI_CHANNEL);
    MIDI.sendControlChange(106, ccAX, MIDI_CHANNEL);
    MIDI.sendControlChange(107, ccAY, MIDI_CHANNEL);
    MIDI.sendControlChange(108, ccAZ, MIDI_CHANNEL);
    MIDI.sendControlChange(109, ccTemp, MIDI_CHANNEL);
    MIDI.sendControlChange(110, ccAlt, MIDI_CHANNEL);
    MIDI.sendControlChange(111, ccHum, MIDI_CHANNEL);
    MIDI.sendControlChange(112, ccBatSOC, MIDI_CHANNEL);

    //Button toggle handling
    for (int i = 0; i < 4; i++) {
      int reading = digitalRead(buttonPins[i]);  // Read the button

      // If the button state has changed
      if (reading != lastButtonStates[i]) {
        lastDebounceTimes[i] = millis();  // Reset the debounce timer for this button
      }

      // If the debounce time has passed
      if ((millis() - lastDebounceTimes[i]) > debounceDelay) {
        // If the button state has changed and is now pressed
        if (reading != buttonStates[i] && reading == LOW) {
          toggleStates[i] = !toggleStates[i];  // Toggle the state for the button
          Serial.print("Button ");
          Serial.print(i);
          Serial.print(" is now ");
          Serial.println(toggleStates[i] ? "ON" : "OFF");
        }

        buttonStates[i] = reading;  // Update the button state
      }

      lastButtonStates[i] = reading;  // Save the current state for the next loop
    }

    // Check each button's toggle state and perform actions
    if (toggleStates[0]) {
      // Button 0 is ON
      Serial.println("Button 0: ON - Performing action for Button 0 ON state");
      MIDI.sendControlChange(113, 127, MIDI_CHANNEL);
      // Perform action when Button 0 is ON
    } else {
      // Button 0 is OFF
      Serial.println("Button 0: OFF - Performing action for Button 0 OFF state");
      MIDI.sendControlChange(113, 0, MIDI_CHANNEL);
      // Perform action when Button 0 is OFF
    }

    if (toggleStates[1]) {
      // Button 1 is ON
      Serial.println("Button 1: ON - Performing action for Button 1 ON state");
      MIDI.sendControlChange(114, 127, MIDI_CHANNEL);
      // Perform action when Button 1 is ON
    } else {
      // Button 1 is OFF
      Serial.println("Button 1: OFF - Performing action for Button 1 OFF state");
      MIDI.sendControlChange(114, 0, MIDI_CHANNEL);
      // Perform action when Button 1 is OFF
    }

    if (toggleStates[2]) {
      // Button 2 is ON
      Serial.println("Button 2: ON - Performing action for Button 2 ON state");
      MIDI.sendControlChange(115, 127, MIDI_CHANNEL);
      // Perform action when Button 2 is ON
    } else {
      // Button 2 is OFF
      Serial.println("Button 2: OFF - Performing action for Button 2 OFF state");
      MIDI.sendControlChange(115, 0, MIDI_CHANNEL);
      // Perform action when Button 2 is OFF
    }

    if (toggleStates[3]) {
      // Button 3 is ON
      Serial.println("Button 3: ON - Performing action for Button 3 ON state");
      MIDI.sendControlChange(116, 127, MIDI_CHANNEL);
      // Perform action when Button 3 is ON
    } else {
      // Button 3 is OFF
      Serial.println("Button 3: OFF - Performing action for Button 3 OFF state");
      MIDI.sendControlChange(116, 0, MIDI_CHANNEL);
      // Perform action when Button 3 is OFF
    }


    // End button togglingg

//If works delete **********************

    // if (digitalRead(BUTTON_PIN_1) == LOW) {
    //   MIDI.sendControlChange(113, 127, MIDI_CHANNEL);
    // } else {
    //   MIDI.sendControlChange(113, 0, MIDI_CHANNEL);
    // }

    // if (digitalRead(BUTTON_PIN_2) == LOW) {
    //   MIDI.sendControlChange(114, 127, MIDI_CHANNEL);
    // } else {
    //   MIDI.sendControlChange(114, 0, MIDI_CHANNEL);
    // }

    // if (digitalRead(BUTTON_PIN_3) == LOW) {
    //   MIDI.sendControlChange(115, 127, MIDI_CHANNEL);
    // } else {
    //   MIDI.sendControlChange(115, 0, MIDI_CHANNEL);
    // }

    // if (digitalRead(BUTTON_PIN_4) == LOW) {
    //   MIDI.sendControlChange(116, 127, MIDI_CHANNEL);
    // } else {
    //   MIDI.sendControlChange(116, 0, MIDI_CHANNEL);
    // }
//***********************************
    vTaskDelay(pdMS_TO_TICKS(30));
  }
}

void taskSys(void *pvParameters) {
  for (;;) {
    if (digitalRead(BUTTON_PIN_3) == LOW && digitalRead(BUTTON_PIN_4) == LOW) {
      xSemaphoreGive(blinkSemaphore);
      vTaskDelay(1200);
      esp_deep_sleep_start();  // Enter deep sleep
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
void taskBlink(void *pvParameters) {
  const TickType_t xDelay = 100 / portTICK_PERIOD_MS;
  while (true) {
    if (xSemaphoreTake(blinkSemaphore, portMAX_DELAY) == pdTRUE) {
      for (int i = 0; i < 5; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        vTaskDelay(xDelay);
        digitalWrite(LED_BUILTIN, LOW);
        vTaskDelay(xDelay);
      }
    }
  }
}
/*----Sensor Value Mapping to Midi-----*/
int mapConstrainedToMidi(int value, int min, int max) {
  return constrain(map(value, min, max, 0, 127), 0, 127);
}
void handleError(const char *message) {
  if (DEBUG_MODE) {
    Serial.println(message);
  }
  ESP.restart();
}