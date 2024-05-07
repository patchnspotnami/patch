/*
   Original from Arduino and MPU6050 Accelerometer and Gyroscope Sensor Tutorial
   by Dejan, https://howtomechatronics.com
   only interested in z axis
*/
#include <Wire.h>

#include <math.h>
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;

int c = 0;





// set up motors
int enable3 = 10;
int motor3 = 9;
int enable4 = 11;
int motor4 =12;
int enable1 = 5;
int motor1 = 4;
int enable2 = 6;
int motor2 = 7;

// set up turning
int proportion_control = 1;  // multiplier for control

float jerror = 0; // error term for controller
long jdist = -1;  // temp distance
float ldutyCycle = 0;
float rdutyCycle = 0;
float jexp=0;
int jtargetAngle = 0;  // the target angle recieved on serial
int jtargetTime = 0;  // the target time/distance recieved from serial
String indata = "";  // string to hold data coming in from serial
long startTime;

void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }  //end of while c < 200
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }  // end of while c < 200
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  
}  // end of calculate_mpu_error

void reset_MPU(){
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU>
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B regi>
  Wire.endTransmission(true);        //end the transmission
  yaw = 0 ;                          // reset the yaw value
  startTime = millis()/1000;              // reset start time in seconds
}  // end of reset_mpu


void setup() {

  // sets the pins as outputs:
  pinMode(motor3, OUTPUT);
  pinMode(motor4, OUTPUT);
  pinMode(enable3, OUTPUT);
  pinMode(enable4, OUTPUT);
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(enable1, OUTPUT);
  pinMode(enable2, OUTPUT);
  
  Serial.begin(9600);

  reset_MPU();
  calculate_IMU_error(); 
}  // end of setup

void Read_MPU(){
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x47); // Gyro Z data first register address 0x47
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 2, true); // Read 2 registers total, the axis value is stored in 2 registers>

  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0; // the datasheet says divide by 131.0 for a 250deg/s range (default)
  // Correct the outputs with the calculated error values

  GyroZ = GyroZ - GyroErrorZ ;  
  // Currently the raw values are in degrees per seconds, deg/s, multiply by time elapsed in crude integration

  yaw =  yaw + GyroZ * elapsedTime;

}  // end of read_mpu

void write_pwm(){

  analogWrite(enable1, ldutyCycle);
  analogWrite(enable2, ldutyCycle);
  analogWrite(enable3, rdutyCycle);
  analogWrite(enable4, rdutyCycle);

}



void jmoving(){

  jerror = -proportion_control*(jtargetAngle - yaw); // how far we are away from target
  
 if (jdist < 5) {  // close to the end, so spin
        if (abs(jerror) < 5) {  // error small so stop
           ldutyCycle = 0;  //stop
           rdutyCycle = 0;
           write_pwm();
           }  // end of spin

         else { // not small error so  turn
          if (jerror > 0) {  //turning to the right - reverse right motors
             digitalWrite(motor1, LOW);  // left motors forward
             digitalWrite(motor2, LOW);
             digitalWrite(motor3, LOW);   //right motors back
             digitalWrite(motor4, LOW);

             }  // end of turn to right
           else {  // turning to left - reverse left motors
             digitalWrite(motor1, HIGH);  // left motors back
             digitalWrite(motor2, HIGH);
             digitalWrite(motor3, HIGH);   //right motors forward
             digitalWrite(motor4, HIGH);

             }  // end of turn to th left
           ldutyCycle = 200;  //spin on the spot
           rdutyCycle = 200;
           write_pwm();
          }  // end of not small error
        } // end of close to the end

 
  else  {  // not close to the end
     if (abs(jerror) < 90){  // no reversing
        digitalWrite(motor1, LOW);  // set motors for going forward
        digitalWrite(motor2, LOW);
        digitalWrite(motor3, HIGH);
        digitalWrite(motor4, HIGH);
        ldutyCycle = (jerror+90)*240/180;  // max value of jerror is 180 and max duty cycle = 240
        rdutyCycle = 240 - ((90 + jerror)*240/180);
        write_pwm();
        }  // end of no reversing
     else {  // jerror between 90 and 180, so spin 
          if (jerror > 0) {  //turning to the right - reverse right motors
             digitalWrite(motor1, LOW);  // left motors forward
             digitalWrite(motor2, LOW);
             digitalWrite(motor3, LOW);   //right motors back
             digitalWrite(motor4, LOW);
    
             } // end of turn right
           else {  // turning to left - reverse left motors
             digitalWrite(motor1, HIGH);  // left motors back 
             digitalWrite(motor2, HIGH);
             digitalWrite(motor3, HIGH);   //right motors forward
             digitalWrite(motor4, HIGH);
      
             }  // end of turn left
           ldutyCycle = 200;  //spin on the spot
           rdutyCycle = 200;
           write_pwm();
          } // end of spin
 
    }  // end of not close to the end

}  //end of  jmoving



void loop() {

Read_MPU();  // update yaw
jdist = jtargetTime + startTime - (millis()/1000); //distance to go


if (Serial.available() > 0) {  // incoming data on serial
    indata = Serial.readStringUntil('\n'); // read first 
    jtargetAngle = indata.toInt();  // the target angle recieved on serial
    indata = Serial.readStringUntil('\n');  // read second
    jtargetTime = indata.toInt();  // target time/distance recieved 
    reset_MPU();  // reset MPU to 0 and reset startTime
    Serial.print("You sent me: ");
    Serial.print(jtargetAngle);
    Serial.print("    and : ");
    Serial.println(jtargetTime);
    }  // end of incoming data

if (jdist < 0 ) { // out of time, so stop
    analogWrite(enable1, 0);
    analogWrite(enable2, 0);
    analogWrite(enable3, 0);
    analogWrite(enable4, 0);
    }  // end of stop
else  {  // in time
    jmoving();

    } //end of in time 
} // end of loop


