
#include <Servo.h>
#include<Wire.h>
//#include <KalmanFilter.h>
const int MPU=0x68; 
int16_t AcX,AcY,AcZ,Tmp,GyX,GyroY,GyroZ;
int pitch;
int accAngleX;
int accAngleY;

int yaw;
int GyroX;
int gyroAngleX;
int gyroAngleY;


int valueX = 90;
int valueY = 90;

Servo servoX;
Servo servoY;



float elapsedTime, currentTime, previousTime;
float kp = 1;
float ki = 1;
float kd = 1;
int pos;
//KalmanFilter kalman(0.001, 0.003, 0.03);

void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);
  Serial.begin(9600);
  servoX.attach(6);
  servoY.attach(7);
}
void loop() {
  
  filter();
  map();
  previousTime = currentTime;        
  currentTime = millis();            
  elapsedTime = (currentTime - previousTime) / 1000; 
  accAngleX = (atan(AcY / sqrt(pow(AcX, 2) + pow(AcZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AcX / sqrt(pow(AcY, 2) + pow(AcZ, 2))) * 180 / PI) + 1.58; // 
  //accAngleZ = (atan(-1 * AcY / sqrt(pow(AcZ, 2) + pow(AcY, 2))) * 180 / PI) + 1.48; // 
 
 
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,12,true);
  AcX=Wire.read()<<8|Wire.read();    
  AcY=Wire.read()<<8|Wire.read();  
  AcZ=Wire.read()<<8|Wire.read();  
  GyroX=Wire.read()<<8|Wire.read();  
  GyroY=Wire.read()<<8|Wire.read();  
  GyroZ=Wire.read()<<8|Wire.read();  
  
  gyroAngleX = gyroAngleX + GyroZ * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  //Serial.print(valueX);
  
  
  //Serial.print("Accelerometer: ");
  //Serial.print("X = "); Serial.print(AcX);
  //Serial.print(" | Y = "); Serial.print(AcY);
 // Serial.print(" | Z = "); Serial.println(AcZ);
  
 //Serial.print("Gyroscope: ");
 //Serial.print("X = "); Serial.print(GyX);
 // Serial.print(" | Y = "); Serial.print(GyroY);
  //Serial.print(" | Z = "); Serial.println(GyroZ);
  //ignition();
   //delay(5);

}
void filter () {
  //Serial.print(GyroX);
  pitch = 0.9 * gyroAngleX + 0.1 * accAngleX;
 yaw = 0.9 * gyroAngleY + 0.1 * accAngleY;
 
}

void map () {
 //TVC Startup//
 valueY = map(yaw, -17000, 17000, 0, 180); // map(value,from high,from low,to low,to high)
 valueX = map(pitch, -17000, 17000, 0, 180);// map(value,from high,from low,to low,to high)
 servoX.write(valueX);
 servoY.write(valueY);
 Serial.print (valueX);
 Serial.print (".......");
 Serial.println (valueY);
}
