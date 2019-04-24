#include <RPLidar.h>

RPLidar lidar;
//SoftwareSerial mySerial(10, 11); 

#define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor.
                        // This pin should connected with the RPLIDAR's MOTOCTRL signal 
const int MOTOR_SPEED = 255;
//unsigned long t, T_MAX;    
//int count = 0; 
//int distances[300];    
String dist_string = "", ang_string=":";               
void setup() {
//   bind the RPLIDAR driver to the arduino hardware serial
  lidar.begin(Serial1);
  Serial.begin(500000);

  // set pin modes
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  analogWrite(RPLIDAR_MOTOR,MOTOR_SPEED);
  delay(1000);
//  t=millis()/1000;
//  T_MAX = 1;
  
}

void loop() {
//    float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
//    float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
//    bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
//    byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement
//    Serial.println(distance);
  if (IS_OK(lidar.waitPoint())) {
    int distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    int angle    = 10*lidar.getCurrentPoint().angle; //anglue value in degree
    bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
    byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement
//    Serial.print(distance);
//    Serial.print(" ");
//    if(startBit)
//      Serial.println("new scan");
    if (distance>0){
//      ang_string= ang_string + String(angle,2) +" ";
//      dist_string= dist_string + String(angle,1) + "_" + String(distance) +" ";
//      count++;
//      Serial.println(String(distance,2));
//      distances[count] = distance;
      Serial.print(angle);
      Serial.print("_");
      Serial.print(distance);
      Serial.print(" ");
    }
//    Serial.println(angle);

//    if(millis()/1000-t>T_MAX){
    if(startBit){
//      Serial.println("1 1 1 1 1 1  11 1 1 1 1 1 1 1 1 11 1 11 1 111 1 1 1 1 1 1 1 1 1 111 1 1 1 1 111 1 1 11 1 1 1 1 1 1 1 1 1  1 1 11 1 1 1 1 1 1 1 1 1 1 1 1  1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 ");
      Serial.println();
//      if(count>100){
////        String prstr = dist_string+ang_string;
//        Serial.println("G");
////        Serial.println();
////        Serial.print("count");
////        Serial.println(count);
////        Serial.println(dist_string);
//      }else {
//        Serial.println("B");
//      }
//      Serial.print("count: ");
//      Serial.println(count);
//      for(int i=0; i<count; i++){
//        Serial.print(distances[i]);
//        Serial.print(' ');
//      }
//      
//      count=0;
//      dist_string = "";
//      ang_string = ":";
//      delay(100);
//      t=millis()/1000;
    }
//    Serial.println(distance);
//    SerialUSB.print(distance);
    
    //perform data processing here... 
    
    
  } else {
//    Serial.println("potato");
    analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
    
    // try to detect RPLIDAR... 
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
       // detected...
       lidar.startScan();
       
       // start motor rotating at max allowed speed
       analogWrite(RPLIDAR_MOTOR, MOTOR_SPEED);
       delay(1000);
    }
  }
}
