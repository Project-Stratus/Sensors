#include <Wire.h>
#include <SparkFun_TMP117.h>
#include <SparkFun_TMP117_Registers.h>
#include <SparkFun_MicroPressure.h>
#include <SparkFun_VEML6075_Arduino_Library.h>
#include "SparkFun_ISM330DHCX.h"


// TMP117 (Temperature Sensor)
TMP117 tempSensor;

// VEML6075 (UV Sensor)
VEML6075 uvSensor;

// MicroPressure Sensor
SparkFun_MicroPressure microPressure;

// ISM330DHCX (6-Axis Sensor)
SparkFun_ISM330DHCX ism330;



// Structs for accelerometer and gyroscope data
sfe_ism_data_t accelData; 
sfe_ism_data_t gyroData; 

// I²C Addresses
const uint8_t TMP117_ADDRESS = 0x48;   // Default address for TMP117
const uint8_t MICRO_PRESSURE_ADDRESS = 0x18; // Default address for MicroPressure sensor
const uint8_t ISM330DHCX_ADDRESS = 0x6B; // Address depends on hardware config

void setup() {
  Serial.begin(115200);

  // Initialize Wire1 for Qwiic
  Wire1.begin();


  // Initialize VEML6075 (UV Sensor)
  if (uvSensor.begin(Wire1) != VEML6075_SUCCESS) {
    Serial.println("VEML6075 failed to initialize.");
    while (1);
  } else {
    Serial.println("VEML6075 initialized successfully.");
  }



  // Initialize TMP117 (Temperature Sensor)
  if (tempSensor.begin(TMP117_ADDRESS, Wire1)) {
    Serial.println("TMP117 initialized successfully.");
  } else {
    Serial.println("TMP117 failed to initialize.");
    while (1);
  }


  // Initialize MicroPressure Sensor
  if (!microPressure.begin(MICRO_PRESSURE_ADDRESS, Wire1)) {
    Serial.println("MicroPressure sensor failed to initialize.");
    while (1);
  } else {
    Serial.println("MicroPressure sensor initialized successfully.");
  }


  // Initialize ISM330DHCX (6-Axis Sensor)
  if (!ism330.begin(Wire1, ISM330DHCX_ADDRESS)) {
    Serial.println("ISM330DHCX failed to initialize.");
    while (1);
  } else {
    Serial.println("ISM330DHCX initialized successfully.");
  }


  // Reset ISM330DHCX and configure settings
  ism330.deviceReset();
  while (!ism330.getDeviceReset()) {
    delay(1);
  }
  Serial.println("ISM330DHCX reset.");
  Serial.println("Applying settings...");



  ism330.setDeviceConfig();
  ism330.setBlockDataUpdate();
  ism330.setAccelDataRate(ISM_XL_ODR_104Hz);
  ism330.setAccelFullScale(ISM_4g);
  ism330.setAccelFilterLP2();
  ism330.setAccelSlopeFilter(ISM_LP_ODR_DIV_100);
  ism330.setGyroDataRate(ISM_GY_ODR_104Hz);
  ism330.setGyroFullScale(ISM_500dps);
  ism330.setGyroFilterLP1();
  ism330.setGyroLP1Bandwidth(ISM_MEDIUM);
}

// void loop() {

//   // VEML6075 UV Sensor Data
//   Serial.print("UVA: ");
//   Serial.print(uvSensor.uva());
//   Serial.print(", UVB: ");
//   Serial.print(uvSensor.uvb());
//   Serial.print(", Index: ");
//   Serial.println(uvSensor.index());


//   // TMP117 Temperature Sensor Data
//   if (tempSensor.dataReady()) {
//     float tempC = tempSensor.readTempC();
//     float tempF = tempSensor.readTempF();
//     Serial.print("Temperature (C): ");
//     Serial.println(tempC);
//     Serial.print("Temperature (F): ");
//     Serial.println(tempF);
//   }

//   // MicroPressure Sensor Data
//   Serial.print("Pressure (PSI): ");
//   Serial.println(microPressure.readPressure(), 4);
//   Serial.print("Pressure (Pa): ");
//   Serial.println(microPressure.readPressure(PA), 1);

//   // ISM330DHCX Sensor Data
//   if (ism330.checkStatus()) {
//     ism330.getAccel(&accelData);
//     ism330.getGyro(&gyroData);

//     Serial.print("Accelerometer X: ");
//     Serial.print(accelData.xData);
//     Serial.print(", Y: ");
//     Serial.print(accelData.yData);
//     Serial.print(", Z: ");
//     Serial.println(accelData.zData);

//     Serial.print("Gyroscope X: ");
//     Serial.print(gyroData.xData);
//     Serial.print(", Y: ");
//     Serial.print(gyroData.yData);
//     Serial.print(", Z: ");
//     Serial.println(gyroData.zData);
//   }

//   delay(250);
// }




void printSensorReadings(float uva, float uvb, float uvIndex, 
                         float tempC, float tempF,
                         float pressurePSI, float pressurePa,
                         float accelX, float accelY, float accelZ,
                         float gyroX, float gyroY, float gyroZ) {
  Serial.println("=============== Sensor Readings ===============");

  // UV Data
  Serial.println("UV Data:");
  Serial.print("    UVA:        "); Serial.println(uva, 2);
  Serial.print("    UVB:        "); Serial.println(uvb, 2);
  Serial.print("    UV Index:   "); Serial.println(uvIndex, 2);
  Serial.println();

  // Temperature Data
  Serial.println("Temperature Data:");
  Serial.print("    Celsius:    "); Serial.println(tempC, 2);
  Serial.print("    Fahrenheit: "); Serial.println(tempF, 2);
  Serial.println();

  // Pressure Data
  Serial.println("Pressure Data:");
  Serial.print("    PSI:        "); Serial.println(pressurePSI, 4);
  Serial.print("    Pa:         "); Serial.println(pressurePa, 1);
  Serial.println();

  // 6-Axis Sensor Data
  Serial.println("6-Axis Sensor Data (ISM330DHCX):");
  Serial.println("  Accelerometer:");
  Serial.print("      X:       "); Serial.println(accelX, 2);
  Serial.print("      Y:       "); Serial.println(accelY, 2);
  Serial.print("      Z:       "); Serial.println(accelZ, 2);

  Serial.println("  Gyroscope:");
  Serial.print("      X:       "); Serial.println(gyroX, 2);
  Serial.print("      Y:       "); Serial.println(gyroY, 2);
  Serial.print("      Z:       "); Serial.println(gyroZ, 2);
  Serial.println("==============================================");
}

void loop() {

  // UV Sensor Data
  float uva = uvSensor.uva();
  float uvb = uvSensor.uvb();
  float uvIndex = uvSensor.index();

  // Temperature Data
  float tempC = tempSensor.readTempC();
  float tempF = tempSensor.readTempF();

  // Pressure Data
  float pressurePSI = microPressure.readPressure();
  float pressurePa = microPressure.readPressure(PA);

  // 6-Axis Sensor Data
  ism330.getAccel(&accelData);
  ism330.getGyro(&gyroData);
  float accelX = accelData.xData;
  float accelY = accelData.yData;
  float accelZ = accelData.zData;
  float gyroX = gyroData.xData;
  float gyroY = gyroData.yData;
  float gyroZ = gyroData.zData;

  // Print all sensor readings
  printSensorReadings(uva, uvb, uvIndex, tempC, tempF, 
                      pressurePSI, pressurePa,
                      accelX, accelY, accelZ, 
                      gyroX, gyroY, gyroZ);

  delay(1000);
}


