#include <Servo.h>

//Rocket Computer Full Code

//-----Library Initialization-----

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Servo.h>
#include <SD.h>
#include <SPI.h>

//-----Address Definitions-----

#define BMP280_I2C_ADDRESS 0x76

//-----Variable Initialization-----

Adafruit_BMP280 bmp280;   // Defines the name of the Barometer Sensor
const int MPU_addr=0x68;  // I2C address of the Gyro/Accel sensor (MPU-6050)
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
const int chipSelect = 7; //Initializes the chip select pin:
                          // (Simpler to define a variable than change all references)
int localpressure = 1000; //Set to forecasted pressure for the day for altitude calculations

//-----Setup-----

void setup() {
  //---Gyro Setup---
  
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600); //Opens serial communication @9600 Baud Rate
  
  //---Baro Setup---
  
  if (!bmp280.begin(BMP280_I2C_ADDRESS))
  {  
    Serial.println("BMP280 not found.");
    while (1);
  //---SD Setup---
  
  if (!SD.begin(chipSelect)) {
    Serial.println("SD Card not found.");
    // don't do anything more:
    while (1);}
    Serial.println("SD card initialized.");
    
  }}

void loop() {

float sensordata[10];

 //---Gyro---

Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);
  sensordata[0] = AcX; 
  sensordata[1] = AcY; 
  sensordata[2] = AcZ;
  sensordata[3] = (Tmp/340.00+36.53);
  sensordata[4] = GyZ;
  sensordata[5] = GyY;
  sensordata[6] = GyZ;
  
 //---Baro---

  float temperature = bmp280.readTemperature();  // get temperature
  float pressure    = bmp280.readPressure();     // get pressure
  float altitude_   = bmp280.readAltitude(localpressure); //references previous integer
  sensordata[7] = temperature;
  sensordata[8] = pressure;
  sensordata[9] = altitude_;

  // print temperature
  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(" °C");
  // print pressure
  Serial.print("Pressure    = ");
  Serial.print(pressure/100);
  Serial.println(" hPa");
  // print altitude
  Serial.print("Approx Altitude = ");
  Serial.print(altitude_);
  Serial.println(" m"); 
 //---SD Card Logging (LAST!)---
  String dataString = "";
  // This for loop probably needs to be changed, either needs to accomodate
  // the I2C bus, or just compile strings to print to SD.
  for (int i = 0; i < 9; i++) {
    int sensor = sensordata[i]; //reads in the sensor value
    dataString += String(sensor); //adds the sensor value to the string
    if (i < 8) {
      dataString += ",";
    }
  }
  // open the file
  File dataFile = SD.open("flightlog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port as well:
    Serial.println(dataString);
  }
  // error reporting:
  else {
    Serial.println("Error opening log file.");
  }
  delay(1000);
}
