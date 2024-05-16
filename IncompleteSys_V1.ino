#include <Wire.h>
#include <ICM20948_WE.h>
#include <BLEMIDI_Transport.h>
#include <hardware/BLEMIDI_ESP32_NimBLE.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
BLEMIDI_CREATE_INSTANCE("Yazan Midi", MIDI)
bool isConnected = false;

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


#define WINDOW_SIZE 2  // Size of the moving average window

float accXBuffer[WINDOW_SIZE] = {0};
float accYBuffer[WINDOW_SIZE] = {0};
float accZBuffer[WINDOW_SIZE] = {0};
int bufferIndex = 0;

float rollBuffer[WINDOW_SIZE] = {0};
float pitchBuffer[WINDOW_SIZE] = {0};
float yawBuffer[WINDOW_SIZE] = {0};
int angleBufferIndex = 0;

void setup() {
  Wire.begin();
  MIDI.begin();
  //Serial.begin(115200); // Consider keeping Serial communication for debugging.
  //while (!Serial); // Ensures Serial console is ready.

  if (!myIMU.init()) {
//    Serial.println("ICM20948 does not respond");
    ESP.restart(); // Instead of hanging the system, perform a restart.
  } else {
//    Serial.println("ICM20948 is connected");
  }

//  Serial.println("Calibrating ICM20948, please wait...");
  delay(1000);
  myIMU.autoOffsets(); // Calibrates the IMU.
//  Serial.println("Calibration complete!");

  myIMU.setAccRange(ICM20948_ACC_RANGE_8G);
  myIMU.setAccDLPF(ICM20948_DLPF_3);
  myIMU.setAccSampleRateDivider(10);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
//  Serial.println("Initializing Bluetooth...");
  BLEMIDI.setHandleConnected([]() {
    isConnected = true;
    digitalWrite(LED_BUILTIN, HIGH);
//      Serial.println("Bluetooth Ready!");

  });
    BLEMIDI.setHandleDisconnected([]() {
    isConnected = false;
    digitalWrite(LED_BUILTIN, LOW);
  });

  xTaskCreatePinnedToCore(taskIMU, "IMU Task", 10000, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskAccelerometer, "Accelerometer Task", 10000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskBLE, "BLE Task", 50000, NULL, 5, NULL, 0);
}

void loop() {
  // Not used in FreeRTOS, but necessary to satisfy Arduino structure
}
void taskAccelerometer(void *pvParameters) {
    for (;;) {
        myIMU.readSensor();
        xyzFloat accRaw = myIMU.getAccRawValues();
        accelerationX = calculateMovingAverage(accXBuffer, accRaw.x * 9.81 / 16384);
        accelerationY = calculateMovingAverage(accYBuffer, accRaw.y * 9.81 / 16384);
        accelerationZ = calculateMovingAverage(accZBuffer, accRaw.z * 9.81 / 16384);
        vTaskDelay(pdMS_TO_TICKS(25));
    }
}
void taskIMU(void *pvParameters) {
    for (;;) {

        myIMU.readSensor();
        xyzFloat angle = myIMU.getAngles();

        // Apply moving average filter to each angle
        roll = calculateMovingAverage(rollBuffer, angle.x);
        pitch = calculateMovingAverage(pitchBuffer, angle.y);
        yaw = calculateMovingAverage(yawBuffer, angle.z);

        vTaskDelay(pdMS_TO_TICKS(25));
    }
}


void taskBLE(void *pvParameters) {
  (void)pvParameters; // Unused parameter

  for (;;) {
    // Send angles over BLE as MIDI CC messages
//MIDI.sendControlChange(cc_number, cc_value, channel);
//void Midi::controlChange(uint8_t channel, uint8_t controller, uint8_t value)

    MIDI.sendControlChange(103, mapAngleToMIDI(roll), 1);  // Sending roll angle
    MIDI.sendControlChange(104, mapAngleToMIDI(pitch), 1); // Sending pitch angle
    MIDI.sendControlChange(105, mapAngleToMIDI(yaw), 1);   // Sending yaw angle
    MIDI.sendControlChange(106, mapAccelerationToMIDI(accelerationX), 1); // Sending accelerationX
    MIDI.sendControlChange(107, mapAccelerationToMIDI(accelerationY), 1); // Sending accelerationY
    MIDI.sendControlChange(108, mapAccelerationToMIDI(accelerationZ), 1); // Sending accelerationZ

    vTaskDelay(pdMS_TO_TICKS(25)); // Delay for 25ms
  }
}

// Function to map angle values to MIDI CC range (0-127)
int mapAngleToMIDI(float angle) {
  // Assuming angle ranges from -120 to 120 degrees
  return map(angle, -120, 120, 0, 127);
}
// Function to map acceleration values to MIDI CC range (0-127)
int mapAccelerationToMIDI(float acceleration) {
  // Assuming acceleration ranges from -9.81 to 9.81 m/s^2
  return map(acceleration, -9.81, 9.81, 0, 127);
}
float calculateMovingAverage(float *buffer, float newValue) {
    buffer[bufferIndex] = newValue;
    float sum = 0.0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        sum += buffer[i];
    }
    bufferIndex = (bufferIndex + 1) % WINDOW_SIZE;
    return sum / WINDOW_SIZE;
}
