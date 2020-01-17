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

Servo gimbleX;
Servo gimbleY;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float t = 0;
float oldt = 0;
float xt = 0;//target x 
float yt = 0;//target y
float kp = 1; // THIS IS THE GAIN THAT WE TWEAK FOR THE PID
float ki = 0;
float kd = 1000;
float dt = 0;
float xp = 0;
float xi = 0;
float xd = 0;
float yp = 0;
float yi = 0;
float yd = 0;
float xp_old = 0;
float xi_old = 0;
float xd_old = 0;
float yp_old = 0;
float yi_old = 0;
float yd_old = 0;
float new_output_x = 40;
float new_output_y = 40;
float old_output_x = 40;
float old_output_y = 40;

int hallSensorPin = 2;     
int magnetState = 0; 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//SD Card
const int chipSelect = 7; //Initializes the chip select pin:
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
bool liftoff = false;
bool drop = false;
bool dropAccel = false; //Accelerometer detects drop
bool dropMagnet = false; // Hall sensor detects drop
bool ignition = false;
//Variables relating to previous actions
float history[5]= {0.000,0.00,0.00,0.00,0.00};
int light_time = 3*1000;
int update_delay = 10;
int drop_time;
//Detection thresholds
float liftoff_threshold = 1.25;
float drop_threshold = 0.2;
//Time thing
int last_time;
//float pos[3] = {0.000,0.000,0.000};

float posx = 0.000;
float posy = 0.000;
float posz = 0.000;
float accx_old = 0.000;
float accy_old = 0.000;
float accz_old = 0.000;

//Record a history of 5 instances of past acceleration to the history
void set_history(float z){//replace first entry with current data after shifting all entries left
  float new_history[5];
  //The first entry of the history is set to the current acceleration value
  new_history[0] = z;
  //Set the last 4 entries of the new history to the first 4 of the old history
  for(int i=0; i<4;i++){
    new_history[i+1] = history[i];
  }
  //Set the history to the new history
  for(int i=0; i<5;i++){
    history[i] = new_history[i];
  }
}

void log_data(){
  //---SD Card Logging (LAST!)---
  String dataString = "";
  // This for loop probably needs to be changed, either needs to accomodate
  // the I2C bus, or just compile strings to print to SD.
  //CREATE THE DATA STRING
  dataString += "Liftoff: ";
  dataString += String(liftoff);
  dataString += " ,Drop: ";
  dataString += String(drop);
  dataString += " ,Ignition: ";
  dataString += String(ignition);
  
  // open the file
  File dataFile = SD.open("flightlog.txt", FILE_WRITE);

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
  
  pinMode(hallSensorPin, INPUT);
  
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
    
    /*if (!SD.begin(chipSelect)) {
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
      Wire.requestFrom(0x68,6,true);           //We ask for just 4 registers 
         
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
  
   magnetState = digitalRead(hallSensorPin); // State of Hall Magnet Sensor
  
  timePrev = tme;                        // the previous time is stored before the actual time read
  tme = millis();                        // actual time read
  elapsedTime = (tme - timePrev) / 1000; //divide by 1000 in order to obtain seconds

  //////////////////////////////////////Gyro read/////////////////////////////////////

    Wire.beginTransmission(0x68);            //begin, Send the slave adress (in this case 68) 
    Wire.write(0x43);                        //First adress of the Gyro data
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,6,true);           //We ask for just 4 registers
        
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
  Wire.requestFrom(0x68,6,true);    //We ask for next 6 registers starting withj the 3B  
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
  /*Acc_rawX=(Wire.read()<<8|Wire.read())/4096.0 ; //each value needs two registres
  Acc_rawY=(Wire.read()<<8|Wire.read())/4096.0 ;
  Acc_rawZ=(Wire.read()<<8|Wire.read())/4096.0 ; */


  /*Serial.print("X: "); Serial.print(Acc_rawX);
  Serial.print(" Y: "); Serial.print(Acc_rawY);
  Serial.print(" Z: "); Serial.println(Acc_rawZ);*/
if(millis()-last_time >= update_delay){
    last_time = millis();
    float magAcc = sqrt(pow(Acc_rawX,2)+pow(Acc_rawY,2)+pow(Acc_rawZ,2));
    set_history(magAcc);
    float avg = 0.00;
    //Position System Code
    //posx += (Acc_rawX*9.81)*pow(0.05,2);
    //posy += (Acc_rawY*9.81)*pow(0.05,2);
    //posz += (Acc_rawZ*9.81)*pow(0.05,2);
 
    if(liftoff==false&&drop==false){//Liftoff detection state
      for(int i=0; i<5; i++){
        avg += history[i];
      }
      avg /= 5;
      //Serial.print("Avg: ");
      //Serial.println(avg);
    if(avg >= liftoff_threshold){
      liftoff=true;
      Serial.println("LIFTOFF!");
    }
    
    //Serial.println("Liftoff Detection State");
    }  
    
  else if(liftoff==true&&drop==false){//Drop detection state
      for(int i=0; i<5; i++){
        avg += history[i];
      }
      avg /= 5;
      //Serial.print("Avg: ");
      //Serial.println(avg);
      
      //Serial.println("Drop Detection State");
      if(avg <= drop_threshold){
        dropAccel=true;
      } 
    if (magnetState == LOW) {        
        dropMagnet = true;
    } 
    if (dropMagnet && dropAccel){
        drop = true;
        Serial.print("DROP!!");
    }
    
  }
 }
if(/*liftoff==true&&drop==true*/true){//Dropping state
    if(ignition==true){//Motor firing state
      /*gimbleY.write((int(Total_angle_y)+90)/2);
      Serial.print("Gyro ValueY: ");
      Serial.print((int(Total_angle_y)+90)/2);

      gimbleX.write((int(Total_angle_x)+90)/2);
      Serial.print(" Gyro ValueX: ");
      Serial.println((int(Total_angle_x)+90)/2);*/
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      t = micros(); // get current time
      dt = t - oldt;
      xp = xt - Total_angle_x;
      xd = (xp - xp_old)/dt;
      xi = xi + xp * dt;
      yp = yt - Total_angle_y;
      yd = (yp - yp_old)/dt;
      yi = yi + yp * dt;
      new_output_x = ((kp * xp) + (ki * xi) + (kd * xd))+40;
      new_output_y = ((kp * yp) + (ki * yi) + (kd * yd))+40;
      new_output_x = min(max(new_output_x, 0),80);
      new_output_y = min(max(new_output_y, 0),80);
      gimbleX.write(int(new_output_x));
      gimbleY.write(int(new_output_y));

      /*Serial.print("X: ");Serial.print(Total_angle_x);
      Serial.print(" Y: ");Serial.println(Total_angle_y);*/
      if(abs(Acc_rawX) > 0.2){
        posx += ((Acc_rawX+accx_old)/2)*pow((dt/100000),2);
      }
      if(abs(Acc_rawY) > 0.2){
        posy += ((Acc_rawY+accy_old)/2)*pow((dt/100000),2);
      }
      if(abs(Acc_rawZ) > 0.2){
        posz += ((Acc_rawZ+accz_old)/2)*pow((dt/100000),2);
      }

      Serial.print("X: ");Serial.print(posx);
      Serial.print(" Y: ");Serial.print(posy);
      Serial.print(" Z: ");Serial.println(posz);

      accx_old = Acc_rawX*9.81;
      accy_old = Acc_rawY*9.81;
      accz_old = Acc_rawZ*9.81;
      
      oldt = micros(); 
      xp_old = xp;
      yp_old = yp;
      xi_old = xi;
      yi_old = yi;
      xd_old = xd;
      yd_old = yd;
      old_output_x = new_output_x;
      old_output_y = new_output_y;
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
    }else{//The engine has not yet been ignited
      if((millis()-drop_time)>=light_time){
        //Light the engine
        ignition = true;
        
    }
   }
 }
}
