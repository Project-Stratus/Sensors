#include <Wire.h>
#include <SparkFun_TMP117.h>
#include <SparkFun_TMP117_Registers.h>
#include <SparkFun_MicroPressure.h>
#include <SparkFun_VEML6075_Arduino_Library.h>
#include "SparkFun_ISM330DHCX.h"
#include <SPI.h>
#include <SD.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

// Sensor and SD Card Objects
TMP117 tempSensor;
VEML6075 uvSensor;
SparkFun_MicroPressure microPressure;
SparkFun_ISM330DHCX ism330;
SFE_UBLOX_GNSS myGNSS;
Sd2Card card;
SdVolume volume;
SdFile root;

// SD Card Chip Select Pin
const int chipSelect = 10;

// Structs for accelerometer and gyroscope data
sfe_ism_data_t accelData; 
sfe_ism_data_t gyroData; 

// I²C Addresses
const uint8_t TMP117_ADDRESS = 0x48;           // TMP117 Default Address
const uint8_t MICRO_PRESSURE_ADDRESS = 0x18;  // MicroPressure Default Address
const uint8_t ISM330DHCX_ADDRESS = 0x6B;      // ISM330DHCX Default Address
const uint8_t GNSS_ADDRESS = 0x42;            // GNSS Default Address

File myFile;

void setup() {
  Serial.begin(115200);

  // Initialize I2C
  Wire1.begin();

  // Initialize Sensors
  initializeUVSensor();
  initializeGNSS();
  initializeTemperatureSensor();
  initializePressureSensor();
  initializeIMUSensor();

  // Initialize SD Card
  initializeSDCard();
}

void initializeUVSensor() {
  if (uvSensor.begin(Wire1) != VEML6075_SUCCESS) {
    Serial.println("VEML6075 failed to initialize.");
    while (1);
  }
  Serial.println("VEML6075 initialized successfully.");
}

void initializeGNSS() {
  if (!myGNSS.begin()) {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Freezing."));
    while (1);
  }
  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA);
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
  myGNSS.setNMEAOutputPort(Serial);
  Serial.println("GNSS initialized successfully.");
}

void initializeTemperatureSensor() {
  if (tempSensor.begin(TMP117_ADDRESS, Wire1)) {
    Serial.println("TMP117 initialized successfully.");
  } else {
    Serial.println("TMP117 failed to initialize.");
    while (1);
  }
}

void initializePressureSensor() {
  if (!microPressure.begin(MICRO_PRESSURE_ADDRESS, Wire1)) {
    Serial.println("MicroPressure sensor failed to initialize.");
    while (1);
  }
  Serial.println("MicroPressure sensor initialized successfully.");
}

void initializeIMUSensor() {
  if (!ism330.begin(Wire1, ISM330DHCX_ADDRESS)) {
    Serial.println("ISM330DHCX failed to initialize.");
    while (1);
  }
  Serial.println("ISM330DHCX initialized successfully.");

  ism330.deviceReset();
  while (!ism330.getDeviceReset()) {
    delay(1);
  }
  Serial.println("ISM330DHCX reset. Applying settings...");
  
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

void initializeSDCard() {
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed.");
    while (1);
  }
  Serial.println("SD card initialized successfully.");
  if (SD.exists("test.txt")) {
    Serial.println("test.txt exists. Deleting...");
    SD.remove("test.txt");
  } else {
    Serial.println("test.txt doesn't exist.");
  }
}

void loop() {
  // Fetch Sensor Data
  long latitude = myGNSS.getLatitude();
  long longitude = myGNSS.getLongitude();
  long altitude = myGNSS.getAltitude();
  float uva = uvSensor.uva();
  float uvb = uvSensor.uvb();
  float uvIndex = uvSensor.index();
  float tempC = tempSensor.readTempC();
  float pressurePa = microPressure.readPressure(PA);
  
  ism330.getAccel(&accelData);
  ism330.getGyro(&gyroData);
  float accelX = accelData.xData;
  float accelY = accelData.yData;
  float accelZ = accelData.zData;
  float gyroX = gyroData.xData;
  float gyroY = gyroData.yData;
  float gyroZ = gyroData.zData;

  // Write Data to File
  writeSensorReadings(latitude, longitude, altitude, uva, uvb, uvIndex, tempC, 
                      pressurePa, accelX, accelY, accelZ, gyroX, gyroY, gyroZ);

  // Read File Content
  readSDFile();

  delay(10000);
}

void writeSensorReadings(long lat, long lon, long alt, float uva, float uvb, float uvIndex, float tempC,
                         float pressurePa, float accelX, float accelY, float accelZ,
                         float gyroX, float gyroY, float gyroZ) {
  myFile = SD.open("test.txt", FILE_WRITE);
  if (myFile) {
    myFile.print(lat); myFile.print(" "); myFile.print(lon); myFile.print(" "); myFile.print(alt); myFile.print(" ");
    myFile.print(uva); myFile.print(" "); myFile.print(uvb); myFile.print(" "); myFile.print(uvIndex); myFile.print(" ");
    myFile.print(tempC); myFile.print(" "); myFile.print(pressurePa); myFile.print(" ");
    myFile.print(accelX); myFile.print(" "); myFile.print(accelY); myFile.print(" "); myFile.print(accelZ); myFile.print(" ");
    myFile.print(gyroX); myFile.print(" "); myFile.print(gyroY); myFile.println(" "); myFile.println(gyroZ);
    myFile.close();
    Serial.println("Data written to test.txt.");
  } else {
    Serial.println("Error opening test.txt");
  }
}

void readSDFile() {
  myFile = SD.open("test.txt");
  if (myFile) {
    Serial.println("Contents of test.txt:");
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    myFile.close();
  } else {
    Serial.println("Error opening test.txt");
  }
}
