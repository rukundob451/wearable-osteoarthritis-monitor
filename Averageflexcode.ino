#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Constants for step detection
const float stepThreshold = 10.0;  // Adjust this value to set the threshold for step detection
unsigned long stepTimeout = 200;  // Timeout between steps in milliseconds to prevent multiple counts
unsigned long lastStepTime = 0;   // Timestamp of the last detected step

int stepCount = 0;  // Total step count
unsigned long lastUpdateTime = 0; // Timestamp of the last 30-second update

const int flexPin = 36; // ADC1_CH0 on ESP32 DevKit v1

const float VCC = 5.0; // voltage at ESP32 3.3V line
const float R_DIV = 10000.0; // resistor used to create a voltage divider
const float flatResistance = 21000.0; // resistance when flat
const float bendResistance = 100000.0; // resistance at 90 deg

BLEServer* pServer = NULL;
BLECharacteristic* FlexCharacteristic = NULL; // For Flex Sensor Data
BLECharacteristic* stepCharacteristic = NULL; // For MPU6050 Data
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_FLEX "beb5483e-36e1-4688-b7f5-ea07361b26a8" // Flex Sensor Data UUID
#define CHARACTERISTIC_UUID_MPU "12345678-1234-5678-1234-56789abcdef0" // MPU6050 Data UUID

Adafruit_MPU6050 mpu;

// Variables for averaging flex sensor readings
float flexSum = 0;
int flexCount = 0;
const int flexAverageCount = 20;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

void setup() { 
    Serial.begin(115200);
    pinMode(flexPin, INPUT);

    // Initialize MPU6050 sensor
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) {
            delay(10);
        }
    }
    Serial.println("MPU6050 Found!");

    // Setup motion detection
    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu.setMotionDetectionThreshold(1);
    mpu.setMotionDetectionDuration(20);
    mpu.setInterruptPinLatch(true); // Keep it latched. Will turn off when reinitialized.
    mpu.setInterruptPinPolarity(true);
    mpu.setMotionInterrupt(true);


    // Create the BLE Device
    BLEDevice::init("WOM: ");

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic for Flex Sensor
    FlexCharacteristic = pService->createCharacteristic(
                          CHARACTERISTIC_UUID_FLEX,
                          BLECharacteristic::PROPERTY_READ   |
                          BLECharacteristic::PROPERTY_WRITE |
                          BLECharacteristic::PROPERTY_NOTIFY |
                          BLECharacteristic::PROPERTY_INDICATE
                        );

    // Create a BLE Characteristic for MPU6050
    stepCharacteristic = pService->createCharacteristic(
                          CHARACTERISTIC_UUID_MPU,
                          BLECharacteristic::PROPERTY_READ   |
                          BLECharacteristic::PROPERTY_NOTIFY |
                          BLECharacteristic::PROPERTY_INDICATE
                        );

    // Create a BLE Descriptor
    FlexCharacteristic->addDescriptor(new BLE2902());
    stepCharacteristic->addDescriptor(new BLE2902());

    // Start the service
    pService->start();

    // Start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0); // set value to 0x00 to not advertise this parameter
    BLEDevice::startAdvertising();
    Serial.println("Waiting for a client connection to notify...");
}

void loop() {
    // Read flex sensor data
    int ADCflex = analogRead(flexPin);
    float Vflex = ADCflex * VCC / 4095; // ESP32 has a 12-bit ADC, so the maximum value is 4095
    float Rflex = R_DIV * (VCC / Vflex - 1.0);
    float angle = map(Rflex, flatResistance, bendResistance, 0, 90.0);

    // Accumulate flex sensor readings
    flexSum += angle;
    flexCount++;

    // Calculate average every 10 readings
    if (flexCount >= flexAverageCount) {
        int flexAverage = int(flexSum) / int(flexCount);
        flexSum = 0;
        flexCount = 0;

        // Prepare flex angle data
        String flexData = String(flexAverage);
        Serial.print("Angle:");
        Serial.println(flexData);
        const char* flexAngle = flexData.c_str();

        // Notify changed value for Flex Sensor
        if (deviceConnected) {
            char testStr[10]; // Adjust size as needed
            strncpy(testStr, flexAngle, sizeof(testStr)); // Copy the flex angle string
            testStr[sizeof(testStr)-1] = '\0'; // Null-terminate the string
            FlexCharacteristic->setValue(testStr);
            FlexCharacteristic->notify();
        }
    }

    // Perform step detection based on motion
    stepDetection();

    // Notify changed value for MPU6050
    if (deviceConnected) {
        char testStr1[10];
        dtostrf(stepCount, 1, 2, testStr1);
        stepCharacteristic->setValue(testStr1);
        stepCharacteristic->notify();

        delay(3); // Bluetooth stack may congest if too many packets are sent quickly
    }

    // Disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // Give the Bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // Restart advertising
        Serial.println("Start advertising");
        oldDeviceConnected = deviceConnected;
    }

    // Connecting
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }

    // Update step count every 30 seconds
    if (millis() - lastUpdateTime > 30000) {
        lastUpdateTime = millis();
        Serial.print("Steps in the last 30 seconds: ");
        Serial.println(stepCount);
        stepCount = 0; // Reset step count for the next 30-second interval
    }

    delay(100); // Reduced delay to allow more frequent sensor readings
}

void stepDetection() {
  if(mpu.getMotionInterruptStatus()) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Basic step detection algorithm using acceleration magnitude
    float accelMagnitude = sqrt(a.acceleration.x * a.acceleration.x +
                                a.acceleration.y * a.acceleration.y +
                                a.acceleration.z * a.acceleration.z);
    Serial.println(accelMagnitude);

    unsigned long currentTime = millis();
    if (accelMagnitude > stepThreshold && (currentTime - lastStepTime) > stepTimeout) {
        stepCount++;
        lastStepTime = currentTime;
    }
  }
}
