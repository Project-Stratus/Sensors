// Test code to see if servo works, reads temperature

#include <Servo.h>
#include <Wire.h>

// TMP117 default I2C address
#define TMP117_ADDRESS 0x48

// TMP117 temperature register
#define TMP117_TEMP_REGISTER 0x00

Servo servo1; // servo

float temperature; // temperature readings here

int initial = 0; // initial angle set to 0
int target = 45; // final angle, 45 degrees will cover up the valve
int location = initial; // location of the servo
int counts = 0; // count the number of temperature readings, not necessary

void setup() {
  servo1.attach(8); // attach to pwm node
  servo1.write(initial); // reset to initial angle

  Wire.begin(); // Initialize I2C communication
  Serial.begin(115200); // Initialize serial communication for debugging

  // Check if the TMP117 is connected
  Wire.beginTransmission(TMP117_ADDRESS);
  if (Wire.endTransmission() != 0) {
    Serial.println("TMP117 not detected. Check connections!");
    while (1); // Stay here if the sensor is not found
  }

  Serial.println("TMP117 detected!");
}

void loop() {
  temperature = readTemperature(); // read temperature from sensor
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");

  if (location == initial) {
    if (temperature > 30) { // if temperature exceeds limit specified, servo moves to target
      servo1.write(target);
      location = target;
      Serial.print("moving to target position");
    }
  } 
  else if (location == target) {
    if (temperature < 26) { // if temperature drops below limit specified, servo moves back to initial position
      servo1.write(initial);
      location = initial;
      Serial.print("moving back to initial position");
    }
  } 
  
  delay(2000); // time delay before reading again
  counts += 1; // add count

  if (counts > 100) { // shuts down after certain counts so it stops beeping the whole time
    servo1.write(initial);
    servo1.detach();
    Serial.write("terminating...");
    Serial.end();
  }
}

// Function to read temperature from TMP117
float readTemperature() {
  Wire.beginTransmission(TMP117_ADDRESS);
  Wire.write(TMP117_TEMP_REGISTER); // Point to the temperature register
  Wire.endTransmission();

  Wire.requestFrom(TMP117_ADDRESS, 2); // Request 2 bytes of data
  if (Wire.available() == 2) {
    uint16_t rawData = (Wire.read() << 8) | Wire.read(); // Combine MSB and LSB
    return rawData * 0.0078125; // Convert to Celsius (TMP117 resolution: 0.0078125°C/LSB)
  } else {
    Serial.println("Error reading temperature!");
    return NAN; // Return "Not a Number" if the read failed
  }
}
