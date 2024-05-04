#include <Wire.h>
#include <ICM20948_WE.h>
#include <BLEMidi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define ICM20948_ADDR 0x68

ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);

// Variables for sensor fusion
float roll = 0.0;   // Roll angle (x-axis)
float pitch = 0.0;  // Pitch angle (y-axis)
float yaw = 0.0;    // Yaw angle (z-axis)
// Global variables for accelerometer data
float accelerationX = 0.0;
float accelerationY = 0.0;
float accelerationZ = 0.0;

void setup() {
  Wire.begin();
// Serial.begin(115200);
//  while (!Serial);

  if (!myIMU.init()) {
   // Serial.println("ICM20948 does not respond");
    while (1);  // Halt if IMU not detected
  } else {
 //   Serial.println("ICM20948 is connected");
  }

  // Calibration and initialization
 // Serial.println("Position your ICM20948 flat and don't move it - calibrating...");
  delay(1000);
  myIMU.autoOffsets();
 // Serial.println("Done!");

  // IMU configuration
  myIMU.setAccRange(ICM20948_ACC_RANGE_8G);
  myIMU.setAccDLPF(ICM20948_DLPF_3);
  myIMU.setAccSampleRateDivider(10);

  // Bluetooth initialization
 // Serial.println("Initializing bluetooth");
  BLEMidiServer.begin("MIDI Controller");
 // Serial.println("Waiting for connections...");

  // Create tasks for IMU sensor reading and Bluetooth communication
  xTaskCreatePinnedToCore(taskIMU, "IMU Task", 10000, NULL, 2, NULL, 1); // IMU task on core 0
  xTaskCreatePinnedToCore(taskAccelerometer, "Accelerometer Task", 10000, NULL, 1, NULL, 1); // Accelerometer task on core 0
  xTaskCreatePinnedToCore(taskBLE, "BLE Task", 50000, NULL, 5, NULL, 0); // Bluetooth task on core 1 with higher priority
}

void loop() {
  // Not used in FreeRTOS, but necessary to satisfy Arduino structure
}
void taskAccelerometer(void *pvParameters) {
  (void)pvParameters; // Unused parameter

  for (;;) {
    myIMU.readSensor();

    // Get raw accelerometer values
    xyzFloat accRaw = myIMU.getAccRawValues();

    // Convert raw values to acceleration in m/s^2
    accelerationX = accRaw.x * 9.81 / 16384; // Convert from LSB to m/s^2
    accelerationY = accRaw.y * 9.81 / 16384;
    accelerationZ = accRaw.z * 9.81 / 16384;

    vTaskDelay(pdMS_TO_TICKS(15)); // Delay for 25ms
  }
}
void taskIMU(void *pvParameters) {
  (void)pvParameters; // Unused parameter

  for (;;) {
    // Read sensor values
    myIMU.readSensor();

    // Calculate angles
    xyzFloat angle = myIMU.getAngles();
    roll = angle.x;
    pitch = angle.y;
    yaw = angle.z;

    vTaskDelay(pdMS_TO_TICKS(20)); // Delay for 25ms
  }
}

void taskBLE(void *pvParameters) {
  (void)pvParameters; // Unused parameter

  for (;;) {
    // Send angles over BLE as MIDI CC messages
    BLEMidiServer.controlChange(0, 103, mapAngleToMIDI(roll));  // Sending roll angle
    BLEMidiServer.controlChange(0, 104, mapAngleToMIDI(pitch)); // Sending pitch angle
    BLEMidiServer.controlChange(0, 105, mapAngleToMIDI(yaw));   // Sending yaw angle
    BLEMidiServer.controlChange(0, 106, mapAccelerationToMIDI(accelerationX)); // Sending accelerationX
    BLEMidiServer.controlChange(0, 107, mapAccelerationToMIDI(accelerationY)); // Sending accelerationY
    BLEMidiServer.controlChange(0, 108, mapAccelerationToMIDI(accelerationZ)); // Sending accelerationZ

    vTaskDelay(pdMS_TO_TICKS(25)); // Delay for 25ms
  }
}

// Function to map angle values to MIDI CC range (0-127)
int mapAngleToMIDI(float angle) {
  // Assuming angle ranges from -90 to 90 degrees
  return map(angle, -90, 90, 0, 127);
}
// Function to map acceleration values to MIDI CC range (0-127)
int mapAccelerationToMIDI(float acceleration) {
  // Assuming acceleration ranges from -9.81 to 9.81 m/s^2
  return map(acceleration, -9.81, 9.81, 0, 127);
}

