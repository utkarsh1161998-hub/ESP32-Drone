#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <esp_now.h>
#include <WiFi.h>

// --- Pin Definitions ---
const int PIN_FL = 12;
const int PIN_FR = 13;
const int PIN_BL = 14;
const int PIN_BR = 27;

// --- PWM Properties ---
const int PWM_FREQ = 20000; // 20kHz for high-frequency coreless motors
const int PWM_RES  = 8;     // 8-bit resolution (0-255)

// --- Structures ---
struct ControlData {
  float throttle; // 0 to 255
  float roll;     // Target angle (-30 to 30 deg)
  float pitch;    // Target angle (-30 to 30 deg)
  float yaw;      // Target rate (-60 to 60 deg/s)
};
ControlData rxData = {0, 0, 0, 0}; // Incoming receiver data

// --- Sensor & Orientation Variables ---
Adafruit_MPU6050 mpu;
float angleRoll = 0, anglePitch = 0;
unsigned long lastTime = 0;

// --- PID Constants & Variables ---
// (Note: These are baseline values and will require tuning for your specific frame)
float kp_roll = 1.2, ki_roll = 0.04, kd_roll = 0.3;
float kp_pitch = 1.2, ki_pitch = 0.04, kd_pitch = 0.3;

float errorRoll, lastErrorRoll, integralRoll, outputRoll;
float errorPitch, lastErrorPitch, integralPitch, outputPitch;

// FreeRTOS Task Handles
TaskHandle_t Core1Task;

// --- ESP-NOW Receiver Callback ---
void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&rxData, incomingData, sizeof(rxData));
}

void setup() {
  Serial.begin(115200);
  
  // Configure PWM Outputs
  ledcAttachChannel(PIN_FL, PWM_FREQ, PWM_RES, 0);
  ledcAttachChannel(PIN_FR, PWM_FREQ, PWM_RES, 1);
  ledcAttachChannel(PIN_BL, PWM_FREQ, PWM_RES, 2);
  ledcAttachChannel(PIN_BR, PWM_FREQ, PWM_RES, 3);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip! System Halted.");
    while (1) { delay(10); }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Initialize ESP-NOW on Core 0 (Wi-Fi Core)
  WiFi.mode(WIFI_STA);
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(esp_now_recv_cb_t(onDataRecv));
    Serial.println("ESP-NOW Initialized Successfully.");
  }

  // Pin the stabilization loop to Core 1
  xTaskCreatePinnedToCore(
    stabilizationLoop, /* Task function */
    "Stabilization",   /* Name of task */
    8192,              /* Stack size */
    NULL,              /* Parameter of the task */
    1,                 /* Priority of the task */
    &Core1Task,        /* Task handle */
    1                  /* Core id */
  );
}

// Core 0 runs background tasks and telemetry natively
void loop() {
  delay(100); 
}

// --- Core 1: Time-Critical Flight Stabilization Loop ---
void stabilizationLoop(void * pvParameters) {
  lastTime = micros();

  while(1) {
    // 1. Maintain rigid 250Hz frequency (4000 microseconds per execution frame)
    unsigned long currentTime = micros();
    float dt = (currentTime - lastTime) / 1000000.0;
    if (dt < 0.004) {
      delayMicroseconds(4000 - (currentTime - lastTime));
      currentTime = micros();
      dt = (currentTime - lastTime) / 1000000.0;
    }
    lastTime = currentTime;

    // 2. Fetch Sensor Data
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Convert raw gyro readings from rad/s to deg/s
    float gyroRateRoll = g.gyro.x * 57.2958;
    float gyroRatePitch = g.gyro.y * 57.2958;

    // Calculate static accelerometer angles
    float accAngleRoll = atan2(a.acceleration.y, a.acceleration.z) * 57.2958;
    float accAnglePitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 57.2958;

    // Complementary Filter: 98% Gyro (Fast/Dynamic) + 2% Accelerometer (Slow/Absolute)
    angleRoll = 0.98 * (angleRoll + gyroRateRoll * dt) + 0.02 * accAngleRoll;
    anglePitch = 0.98 * (anglePitch + gyroRatePitch * dt) + 0.02 * accAnglePitch;

    // 3. Compute PID Controller Outputs
    if (rxData.throttle > 20) { // Only run tracking loops if motors are actively spinning
      // Roll Control Loop
      errorRoll = rxData.roll - angleRoll;
      integralRoll += errorRoll * dt;
      integralRoll = constrain(integralRoll, -50, 50); // Prevent wind-up
      float derivativeRoll = (errorRoll - lastErrorRoll) / dt;
      outputRoll = (kp_roll * errorRoll) + (ki_roll * integralRoll) + (kd_roll * derivativeRoll);
      lastErrorRoll = errorRoll;

      // Pitch Control Loop
      errorPitch = rxData.pitch - anglePitch;
      integralPitch += errorPitch * dt;
      integralPitch = constrain(integralPitch, -50, 50);
      float derivativePitch = (errorPitch - lastErrorPitch) / dt;
      outputPitch = (kp_pitch * errorPitch) + (ki_pitch * integralPitch) + (kd_pitch * derivativePitch);
      lastErrorPitch = errorPitch;
    } else {
      // Clear integral histories if throttled down to prevent instant flips on takeoff
      integralRoll = 0;  lastErrorRoll = 0;  outputRoll = 0;
      integralPitch = 0; lastErrorPitch = 0; outputPitch = 0;
    }

    // 4. X-Configuration Mixer Calculations
    float mixFL = rxData.throttle + outputPitch - outputRoll - rxData.yaw;
    float mixFR = rxData.throttle + outputPitch + outputRoll + rxData.yaw;
    float mixBL = rxData.throttle - outputPitch - outputRoll + rxData.yaw;
    float mixBR = rxData.throttle - outputPitch + outputRoll - rxData.yaw;

    // 5. Direct Hardware Constraints & Writes
    ledcWrite(PIN_FL, constrain(mixFL, 0, 255));
    ledcWrite(PIN_FR, constrain(mixFR, 0, 255));
    ledcWrite(PIN_BL, constrain(mixBL, 0, 255));
    ledcWrite(PIN_BR, constrain(mixBR, 0, 255));
  }
}
