/* http://www.youtube.com/c/electronoobs/eng_arduino_tut76.php * 
 * This is an example where we configure te data of the MPU6050
 * and read the Acceleration data and detect movement 
 * Arduino pin    |   MPU6050
 * 5V             |   Vcc
 * GND            |   GND
 * A4             |   SDA
 * A5             |   SCL
 */
#include <Wire.h>
#include <Servo.h>
#include <SD.h>
#include <SPI.h>
#include <Seeed_BME280.h>

Servo gimbleX;
Servo gimbleY;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float t = 0;
float oldt = 0;
float xt = 0;//target x 
float yt = 0;//target y
float kp = 1;
float kd = 1000;
float dt = 0;
float xp = 0;
float xd = 0;
float yp = 0;
float yd = 0;
float xp_old = 0;
float xd_old = 0;
float yp_old = 0;
float yd_old = 0;
float new_output_x = 40;
float new_output_y = 40;
float old_output_x = 40;
float old_output_y = 40;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//SD Card
const int chipSelect = 4; //Initializes the chip select pin:
                          // (Simpler to define a variable than change all references)

//Gyro Variables
float elapsedTime, tme, timePrev; 
int gyro_error=0;                         //We use this variable to only calculate once the gyro data error
float Gyr_rawX, Gyr_rawY, Gyr_rawZ;     //Here we store the raw data read 
float Gyro_angle_x, Gyro_angle_y;         //Here we store the angle value obtained with Gyro data
float Gyro_raw_error_x, Gyro_raw_error_y; //Here we store the initial gyro data error

//Acc Variables
int acc_error=0;                         //We use this variable to only calculate once the Acc data error
float rad_to_deg = 180/3.141592654;      //This value is for pasing from radians to degrees values
float Acc_rawX, Acc_rawY, Acc_rawZ;    //Here we store the raw data read 
float Acc_angle_x, Acc_angle_y;          //Here we store the angle value obtained with Acc data
float Acc_angle_error_x, Acc_angle_error_y; //Here we store the initial Acc data error
float Total_angle_x, Total_angle_y;
float prev_acc_x = 1000;

//state tracking booleans
bool drop = false;
bool ignition = false;
//Variables relating to previous actions
float history_alt[5]= {0.000,0.00,0.00,0.00,0.00};
float history_acc[5]= {0.000,0.00,0.00,0.00,0.00};
int update_delay = 10;
//Time thing
int last_time;

//Barometer Varialbes
BME280 bme280; //Altitude Sensor
float pressure = 0.0000;
float altitude = 0.0000;
float initAltitude;

//Record a history of 5 instances of past acceleration to the history
void set_history_acc(float acc){//replace first entry with current data after shifting all entries left
  float new_history_acc[5];
  //The first entry of the history is set to the current acceleration value
  new_history_acc[0] = acc;
  //Set the last 4 entries of the new history to the first 4 of the old history
  for(int i=0; i<4;i++){
    new_history_acc[i+1] = history_acc[i];
  }
  //Set the history to the new history
  for(int i=0; i<5;i++){
    history_acc[i] = new_history_acc[i];
  }
}

void set_history_alt(float acc){//replace first entry with current data after shifting all entries left
  float new_history_alt[5];
  //The first entry of the history is set to the current acceleration value
  new_history_alt[0] = acc;
  //Set the last 4 entries of the new history to the first 4 of the old history
  for(int i=0; i<4;i++){
    new_history_alt[i+1] = history_alt[i];
  }
  //Set the history to the new history
  for(int i=0; i<5;i++){
    history_alt[i] = new_history_alt[i];
  }
}

void log_data(){
  //---SD Card Logging (LAST!)---
  String dataString = "";
  // This for loop probably needs to be changed, either needs to accomodate
  // the I2C bus, or just compile strings to print to SD.
  //CREATE THE DATA STRING
  dataString += "Drop: ";
  dataString += String(drop);
  dataString += ",Ignition: ";
  dataString += String(ignition);
  dataString += ",RotX: ";
  dataString += String(Total_angle_x);
  dataString += ",RotY: ";
  dataString += String(Total_angle_y);
  dataString += ",AccX: ";
  dataString += String(Acc_rawX);
  dataString += ",AccY: ";
  dataString += String(Acc_rawY);
  dataString += ",AccZ: ";
  dataString += String(Acc_rawZ);
  dataString += ",Pressure: ";
  dataString += String(pressure);
  dataString += ",Altitude: ";
  dataString += String(altitude);
  
  // open the file
  File dataFile = SD.open("log.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(String(millis()));
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port as well:
    Serial.println(dataString);
  }
  // error reporting:
  else {
    Serial.println("Error opening log file.");
  }
}

void setup() {
  gimbleX.attach(10);//Limit 0 to 80
  gimbleY.attach(9);
  
    if(!bme280.init()){
    Serial.println("BME280 Device error!");
  }
  pressure = bme280.getPressure();
  initAltitude = bme280.calcAltitude(pressure);
  
  Wire.begin();                           //begin the wire comunication
  Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68)              
  Wire.write(0x6B);                       //make the reset (place a 0 into the 6B register)
  Wire.write(0x00);
  Wire.endTransmission(true);             //end the transmission
  //Gyro config
  Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68) 
  Wire.write(0x1B);                       //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                       //Set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(true);             //End the transmission with the gyro
  //Acc config
  Wire.beginTransmission(0x68);           //Start communication with the address found during search.
  Wire.write(0x1C);                       //We want to write to the ACCEL_CONFIG register
  Wire.write(0x10);                       //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true); 

  Serial.begin(9600);                     //Remember to set this same baud rate to the serial monitor  
  tme = millis();                        //Start counting time in milliseconds

  //---SD Setup---
    
    if (!SD.begin(chipSelect)) {
      Serial.println("SD Card not found.");
      // don't do anything more:
      while (1);
      Serial.println("SD card initialized.");
      
    }
/*Here we calculate the acc data error before we start the loop
 * I make the mean of 200 values, that should be enough*/
  if(acc_error==0)
  {
    for(int a=0; a<200; a++)
    {
      Wire.beginTransmission(0x68);
      Wire.write(0x3B);                       //Ask for the 0x3B register- correspond to AcX
      Wire.endTransmission(false);
      Wire.requestFrom(0x68,6,true); 
      
      Acc_rawX=(Wire.read()<<8|Wire.read())/4096.0 ; //each value needs two registres
      Acc_rawZ=-(Wire.read()<<8|Wire.read())/4096.0 ;
      Acc_rawY=(Wire.read()<<8|Wire.read())/4096.0 ;

      
      /*---X---*/
      Acc_angle_error_x = Acc_angle_error_x + ((atan((Acc_rawY)/sqrt(pow((Acc_rawX),2) + pow((Acc_rawZ),2)))*rad_to_deg));
      /*---Y---*/
      Acc_angle_error_y = Acc_angle_error_y + ((atan(-1*(Acc_rawX)/sqrt(pow((Acc_rawY),2) + pow((Acc_rawZ),2)))*rad_to_deg)); 
      
      if(a==199)
      {
        Acc_angle_error_x = Acc_angle_error_x/200;
        Acc_angle_error_y = Acc_angle_error_y/200;
        acc_error=1;
      }
    }
    last_time=millis();
  }//end of acc error calculation   


/*Here we calculate the gyro data error before we start the loop
 * I make the mean of 200 values, that should be enough*/
  if(gyro_error==0)
  {
    for(int i=0; i<200; i++)
    {
      Wire.beginTransmission(0x68);            //begin, Send the slave adress (in this case 68) 
      Wire.write(0x43);                        //First adress of the Gyro data
      Wire.endTransmission(false);
      Wire.requestFrom(0x68,6,true);           //We ask for just 6 registers 
         
      Gyr_rawX=Wire.read()<<8|Wire.read();     //Once again we shif and sum
      Gyr_rawZ=-(Wire.read()<<8|Wire.read());
      Gyr_rawY=Wire.read()<<8|Wire.read();
   
      /*---X---*/
      Gyro_raw_error_x = Gyro_raw_error_x + (Gyr_rawX/32.8); 
      /*---Y---*/
      Gyro_raw_error_y = Gyro_raw_error_y + (Gyr_rawY/32.8);
      if(i==199)
      {
        Gyro_raw_error_x = Gyro_raw_error_x/200;
        Gyro_raw_error_y = Gyro_raw_error_y/200;
        gyro_error=1;
      }
    }
  }//end of gyro error calculation   
  oldt = micros();
}//end of setup void






void loop(){
  timePrev = tme;                        // the previous time is stored before the actual time read
  tme = millis();                        // actual time read
  elapsedTime = (tme - timePrev) / 1000; //divide by 1000 in order to obtain seconds

  //////////////////////////////////////Gyro read/////////////////////////////////////

    Wire.beginTransmission(0x68);            //begin, Send the slave adress (in this case 68) 
    Wire.write(0x43);                        //First adress of the Gyro data
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,6,true);           //We ask for just 6 registers
        
    Gyr_rawX=Wire.read()<<8|Wire.read();     //Once again we shif and sum
    Gyr_rawZ=-(Wire.read()<<8|Wire.read());
    Gyr_rawY=Wire.read()<<8|Wire.read();
    /*Now in order to obtain the gyro data in degrees/seconds we have to divide first
    the raw value by 32.8 because that's the value that the datasheet gives us for a 1000dps range*/
    /*---X---*/
    Gyr_rawX = (Gyr_rawX/32.8) - Gyro_raw_error_x; 
    /*---Y---*/
    Gyr_rawY = (Gyr_rawY/32.8) - Gyro_raw_error_y;
    
    /*Now we integrate the raw value in degrees per seconds in order to obtain the angle
    * If you multiply degrees/seconds by seconds you obtain degrees */
    /*---X---*/
    Gyro_angle_x = Gyr_rawX*elapsedTime;
    /*---X---*/
    Gyro_angle_y = Gyr_rawY*elapsedTime;


    
  
  //////////////////////////////////////Acc read/////////////////////////////////////

  Wire.beginTransmission(0x68);     //begin, Send the slave adress (in this case 68) 
  Wire.write(0x3B);                 //Ask for the 0x3B register- correspond to AcX
  Wire.endTransmission(false);      //keep the transmission and next
  Wire.requestFrom(0x68,6,true);    //We ask for next 6 registers starting with the 3B  
  /*We have asked for the 0x3B register. The IMU will send a brust of register.
  * The amount of register to read is specify in the requestFrom function.
  * In this case we request 6 registers. Each value of acceleration is made out of
  * two 8bits registers, low values and high values. For that we request the 6 of them  
  * and just make then sum of each pair. For that we shift to the left the high values 
  * register (<<) and make an or (|) operation to add the low values.
  If we read the datasheet, for a range of+-8g, we have to divide the raw values by 4096*/    
  Acc_rawX=(Wire.read()<<8|Wire.read())/4096.0 ; //each value needs two registres
  Acc_rawZ=-(Wire.read()<<8|Wire.read())/4096.0 ;
  Acc_rawY=(Wire.read()<<8|Wire.read())/4096.0 ; 
 /*Now in order to obtain the Acc angles we use euler formula with acceleration values
 after that we substract the error value found before*/  
 /*---X---*/
 Acc_angle_x = (atan((Acc_rawY)/sqrt(pow((Acc_rawX),2) + pow((Acc_rawZ),2)))*rad_to_deg) - Acc_angle_error_x;
 /*---Y---*/
 Acc_angle_y = (atan(-1*(Acc_rawX)/sqrt(pow((Acc_rawY),2) + pow((Acc_rawZ),2)))*rad_to_deg) - Acc_angle_error_y;    


 //////////////////////////////////////Total angle and filter/////////////////////////////////////
 /*---X axis angle---*/
 Total_angle_x = 0.98 *(Total_angle_x + Gyro_angle_x) + 0.02*Acc_angle_x;
 /*---Y axis angle---*/
 Total_angle_y = 0.98 *(Total_angle_y + Gyro_angle_y) + 0.02*Acc_angle_y;
   
 /* //////////////////////////////////////Acc read/////////////////////////////////////

  Wire.beginTransmission(0x68);     //begin, Send the slave adress (in this case 68) 
  Wire.write(0x3B);                 //Ask for the 0x3B register- correspond to AcX
  Wire.endTransmission(false);      //keep the transmission and next
  Wire.requestFrom(0x68,6,true);    //We ask for next 6 registers starting withj the 3B  
  /*We have asked for the 0x3B register. The IMU will send a brust of register.
  * The amount of register to read is specify in the requestFrom function.
  * In this case we request 6 registers. Each value of acceleration is made out of
  * two 8bits registers, low values and high values. For that we request the 6 of them  
  * and just make then sum of each pair. For that we shift to the left the high values 
  * register (<<) and make an or (|) operation to add the low values.
  If we read the datasheet, for a range of+-8g, we have to divide the raw values by 4096*/    
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
pressure = bme280.getPressure();
if(millis()-last_time >= update_delay){
    last_time = millis();
  
  ///////////Acceleration
    float magAcc = sqrt(pow(Acc_rawX,2)+pow(Acc_rawY,2)+pow(Acc_rawZ,2));
    set_history_acc(magAcc);
    float sumAcc = 0;
    for(int i=0; i<5;i++){
      sumAcc += history_acc[i];
    }
    float avgAcc = sumAcc / 5.0;
    
    /*Serial.print("Acceleration: ");
    Serial.print(avgAcc);
    Serial.println("m/s^2");*/
    
    
  
  ////////////Altitude
    float magAltitude = bme280.calcAltitude(pressure);
    altitude = magAltitude - initAltitude;
    set_history_alt(magAltitude);
    float sumAltitude = 0;
    for(int i=0; i<5;i++){
      sumAltitude += history_alt[i];
    }
    float avgAltitude = sumAltitude / 5.0;
    
    /*Serial.print("Altitude: ");
    Serial.print(avgAltitude);
    Serial.println("m");*/

  log_data();
  
}
   
 if(drop==false){//Drop detection state
      
 }
if(/*drop==*/true){//Dropping state
    if(ignition==true){//Motor firing state
      t = micros(); // get current time
      dt = t - oldt;
      xp = xt - Total_angle_x;
      xd = (xp - xp_old)/dt;
      yp = yt - Total_angle_y;
      yd = (yp - yp_old)/dt;
      new_output_x = ((kp * xp) + (kd * xd))+40;
      new_output_y = ((kp * yp) + (kd * yd))+40;
      new_output_x = min(max(new_output_x, 0),80);
      new_output_y = min(max(new_output_y, 0),80);
      gimbleX.write(int(new_output_x));
      gimbleY.write(int(new_output_y));


      
      //Serial.print("X: ");Serial.print(new_output_x);
      //Serial.print(" Y: ");Serial.println(new_output_y);

      
      oldt = micros(); 
      xp_old = xp;
      yp_old = yp;
      xd_old = xd;
      yd_old = yd;
      old_output_x = new_output_x;
      old_output_y = new_output_y;
        
    }else{//The engine has not yet been ignited
      //
      if(true/*Check that the rocket is falling before igniting the engine*/){
        //Light the engine
        ignition = true;
        
    }
   }
 }

}
