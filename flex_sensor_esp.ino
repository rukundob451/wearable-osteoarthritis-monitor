const int flexPin = 36; // ADC1_CH0 on ESP32 DevKit v1

const float VCC = 3.3; // voltage at ESP32 3.3V line
const float R_DIV = 10000.0; // resistor used to create a voltage divider
const float flatResistance = 30000.0; // resistance when flat
const float bendResistance = 100000.0; // resistance at 90 deg   
float flexSum = 0;
int flexCount = 0;
const int flexAverageCount = 10;

void setup() {
 Serial.begin(115200); // ESP32 uses 115200 baud rate by default
 pinMode(flexPin, INPUT);
}

void loop() {
 // Read the ADC, and calculate voltage and resistance from it
 int ADCflex = analogRead(flexPin);
 float Vflex = ADCflex * VCC / 4095.0; // ESP32 has a 12-bit ADC, so the maximum value is 4095
 float Rflex =  (Vflex / (VCC - Vflex)* R_DIV);
//  Serial.println("Resistance: " + String(Rflex) + " ohms");

 // Use the calculated resistance to estimate the sensor's bend angle:
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
      const char* flexAngle = flexData.c_str();
      Serial.print("Angle: ");
      Serial.println(flexAngle);

 delay(500);
    }
}
