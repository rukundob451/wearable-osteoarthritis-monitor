#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

const int flexPin = 34; // ADC1_CH0 on ESP32 DevKit v1

const float VCC = 3.3; // voltage at ESP32 3.3V line
const float R_DIV = 47000.0; // resistor used to create a voltage divider
const float flatResistance = 215000.0; // resistance when flat
const float bendResistance = 100000.0; // resistance at 90 deg

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

Adafruit_MPU6050 mpu;

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
 BLEDevice::init("ESP32");

 // Create the BLE Server
 pServer = BLEDevice::createServer();
 pServer->setCallbacks(new MyServerCallbacks());

 // Create the BLE Service
 BLEService *pService = pServer->createService(SERVICE_UUID);

 // Create a BLE Characteristic
 pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

 // Create a BLE Descriptor
 pCharacteristic->addDescriptor(new BLE2902());

 // Start the service
 pService->start();

 // Start advertising
 BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
 pAdvertising->addServiceUUID(SERVICE_UUID);
 pAdvertising->setScanResponse(false);
 pAdvertising->setMinPreferred(0x0); // set value to 0x00 to not advertise this parameter
 BLEDevice::startAdvertising();
 Serial.println("Waiting a client connection to notify...");
}

void loop() {
    // Read flex sensor data
    int ADCflex = analogRead(flexPin);
    float Vflex = ADCflex * VCC / 4095.0; // ESP32 has a 12-bit ADC, so the maximum value is 4095
    float Rflex = R_DIV * (VCC / Vflex - 1.0);
    float angle = map(Rflex, flatResistance, bendResistance, 0, 90.0);

    // Read sensor data
    if(mpu.getMotionInterruptStatus()) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        // Format sensor data as a string
        String sensorData = "X:" + String(a.acceleration.x) +
                           //  ",Y:" + String(a.acceleration.y) +
                           //  ",Z:" + String(a.acceleration.z) +
                           //  ",GyroX:" + String(g.gyro.x) +
                          //   ",GyroY:" + String(g.gyro.y) +
                           //  ",GyroZ:" + String(g.gyro.z) +
                             ",FlexAngle:" + String(angle); // Include flex angle in the data

        // Convert Arduino String to std::string
        std::string message = sensorData.c_str();

        // Notify changed value
        if (deviceConnected) {
            pCharacteristic->setValue(message);
            pCharacteristic->notify();
            value++;
            delay(3); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
        }
    }

    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }

    delay(500);
}
