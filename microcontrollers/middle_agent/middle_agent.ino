/*
  Pi Pico 2 W WiFi Station Demo
  pico2w_wifi_test.ino
  Use WiFi library to connect Pico 2 W to WiFi in station mode
*/

// include the WiFi Library
#include <WiFi.h>
#include <Wire.h>
#include "AS5600.h"
#include <BasicLinearAlgebra.h>
#include "math.h"
using namespace BLA;

AS5600 as5600_0(&Wire);
AS5600 as5600_1(&Wire1);


// replace with your network credentials
const char* ssid = "REPLACE_WITH_SSID";
const char* password = "REPLACE_WITH_PASSWORD";

# define RIGHT_MOTOR_FORWARD GP0
# define RIGHT_MOTOR_BACKWARD GP1
# define LEFT_MOTOR_FORWARD GP2
# define LEFT_MOTOR_BACKWARD GP3

# define TOP_FLEX_SENSOR GP4
# define BOTTOM_FLEX_SENSOR GP5

# define TOP_ENCODER GP6
# define BOTTOM_ENCODER GP6


// FLEX SENSOR PARAMTERS
float trueFlexDataBottom; // Unchanged output for bottom flex sensor
float trueFlexDataTop; // Unchanged output for top flex sensor

float calibratedFlexDataBottom; // Manipulated flex sensor data for bottom
float calibratedFlexDataTop; // Manipulated flex sensor data for top

float strainRepresentationBottom; // The representation of strain for the bottom tether
float strainRepresentationTop; // The representation of strain for the top tether

float STRAIGHT = 0; // The angle, in degrees, that is used to calibrate the flex sensor
                   // for when it is not bent
float BEND = 90; // The angle, in degrees, that is used to calibrate the flex sensor
                 // for when it is bent

const float STRAIGHT_BOTTOM = 420; // used for calibration and manipulation of raw data
const float STRAIGHT_TOP = 410; // used for calibration and manipulation of raw data
const float BEND_BOTTOM = 300; // used for calibration and manipulation of raw data
const float BEND_TOP = 300; // used for calibration and manipulation of raw data



// ENCODER ANGLE PARAMETERS
float trueAngleDataBottom; // Unchanged output for bottom encoder
float trueAngleDataTop; // Unchanged output for top encoder

float calibratedAngleDataBottom; // Manipulated encoder data for bottom
float calibratedAngleDataTop; // Manipulated encoder data for top

float deltaMeasured; // The difference between calibratedAngleDataBottom and calibratedAngleDataTop
float deltaError; // The error value between the desired and measured deltas

float encBottom_0 = 360; // value used for calibration for the bottom encored and desired angle of 0 degrees
float encBottom_90 = 262; // value used for calibration for the bottom encored and desired angle of 90 degrees
float encBottom_180 = 176; // value used for calibration for the bottom encored and desired angle of 180 degrees
float encBottom_270 = 86; // value used for calibration for the bottom encored and desired angle of 270 degrees
float encBottom_360 = 0; // value used for calibration for the bottom encored and desired angle of 360 degrees

float encTop_0 = 210; // value used for calibration for the bottom encored and desired angle of 0 degrees
float encTop_90 = 122; // value used for calibration for the bottom encored and desired angle of 90 degrees
float encTop_180_first = 30; // value used for calibration for the bottom encored and desired angle of 180 degrees
float encTop_180_second = 390; // value used for calibration for the bottom encored and desired angle of 180 degrees
float encTop_270 = 300; // value used for calibration for the bottom encored and desired angle of 270 degrees
float encTop_360 = 210; // value used for calibration for the bottom encored and desired angle of 360 degrees


// DIRECTION VECTORS
float directionMagnitude; // magnitude of the direction vector
float directionDirection; // direction of the direction vector


// RESULTANT VECTORS
float resultantMagnitude; // magnitude of the resultant vector
float resultantDirection; // direction of the resultant vector


// PID Initialization 
float dt; 
float errorE; 
float derivative; 
float PID_control_O; 
float previousError; 


// Goal Angle and Upper Stop Angle
float deltaDesired = 90; // The desired difference between calibratedAngleDataBottom and calibratedAngleDataTop
float lowerA = 45;
float upperA = 80;


// Flex Sensor Range Angles 
float bottomSensorMin = 60; 
float bottomSensorMax = 70;
float topSensorMin = 60; 
float topSensorMax = 70;

// Bluetooth Variables
float run_START_CASE;
float deltaD = 130 * M_PI/180; 
float angleInput = 0;
float Right = 150; 
float Left = 150;

// Array Initialization, for storing the history of data
const int maxnew = 5000; 
float timeR[maxnew];
float calibratedFlexDataBottom_Array[maxnew]; 
float calibratedFlexDataTop_Array[maxnew];     
float calibratedAngleDataBottom_Array[maxnew]; 
float calibratedAngleDataTop_Array[maxnew];     
float deltaMeasured_Array[maxnew]; 
float deltaD_Array[maxnew]; // I DON'T KNOW WHAT DELTAD IS, MUST FIND OUT
float resultantMagnitude_Array[maxnew]; 
float resultantDirection_Array[maxnew]; 
float directionDirection_Array[maxnew]; 




void setup() {

  Serial.begin(115200); // start the Serial Monitor
  WiFi.mode(WIFI_STA); // operate in WiFi Station mode
  WiFi.begin(ssid, password); // start WiFi with supplied parameters

  Wire.begin();
  Wire1.begin();

  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);

  pinMode(TOP_FLEX_SENSOR, INPUT);
  pinMode(BOTTOM_FLEX_SENSOR, INPUT);

  as5600_0.begin();  //  set direction pin.
  as5600_0.setDirection(AS5600_CLOCK_WISE);
  Serial.println(as5600_0.getAddress(),HEX);
  Serial.print("Connect device 0: ");
  Serial.println(as5600_0.isConnected() ? "true" : "false");
  int b = as5600_0.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);
  delay(1000);

  as5600_1.begin();  //  set direction pin.
  as5600_1.setDirection(AS5600_CLOCK_WISE);
  Serial.println(as5600_1.getAddress(),HEX);
  Serial.print("Connect device 1: ");
  Serial.println(as5600_1.isConnected() ? "true" : "false");
  int c = as5600_1.isConnected();
  Serial.print("Connect: ");
  Serial.println(c);
  delay(1000);

//   // print periods on monitor while establishing connection
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//     delay(500);
//   }

//   // connection established
//   Serial.println("");
//   Serial.print("Pico 2 W is connected to WiFi network ");
//   Serial.println(WiFi.SSID());

//   // print IP Address
//   Serial.print("Assigned IP Address: ");
//   Serial.println(WiFi.localIP());

}


void SPIN(float theta0, float phiDes, float deltaD) 
{
  Serial.print("theta0: "); Serial.println(theta0 * 180/M_PI);
  Serial.print("phiDes: "); Serial.println(phiDes * 180/M_PI);
  float Kp = 1;//16; float Kd = 1; 
  float endTime = millis(); float previousTime = millis(); 
  
  while (theta0*180/M_PI > (phiDes*180/M_PI)+5 || theta0*180/M_PI < (phiDes*180/M_PI)-5)
  {
    Serial.println("START:");
    // Angle Formation //////////////////////////////////////////////////////////////////
    ang_T0 = as5600_0.readAngle() * AS5600_RAW_TO_RADIANS * 180/M_PI;
    ang_T1 = as5600_1.readAngle() * AS5600_RAW_TO_RADIANS * 180/M_PI;

  //// ROBOT 4 
    // Calibrating the magnetic encoder for tether angles
    if(ang_T0 > T0_90 && ang_T0 <= T0_0) {theta0 = map(ang_T0, T0_0, T0_90, 0, 90);}
    else if (ang_T0 > T0_180 && ang_T0 <= T0_90) {theta0 = map(ang_T0, T0_90, T0_180, 90, 180);}
    else if (ang_T0 > T0_270 && ang_T0 <= T0_180) {theta0 = map(ang_T0, T0_180, T0_270, 180, 270);}
    else if (ang_T0 > T0_360 && ang_T0 <= T0_270) {theta0 = map(ang_T0, T0_270, T0_360, 270, 360);}

    //Serial.println(ang_T1);
    if (ang_T1 < T1_180first && ang_T1 >= 0) {ang_T1 = ang_T1 + 360;}
    if (ang_T1 > T1_90 && ang_T1 <= T1_0) {theta1 = map(ang_T1, T1_0, T1_90, 0, 90) + theta0;}
    else if (ang_T1 > T1_180first && ang_T1 <= T1_90) {theta1 = map(ang_T1, T1_90, T1_180first, 90, 180) + theta0;}
    else if (ang_T1 > T1_270 && ang_T1 <= T1_180second) {theta1 = map(ang_T1, T1_180second, T1_270, 180, 270) + theta0;}
    else if (ang_T1 > T1_360 && ang_T1 <= T1_270) {theta1 = map(ang_T1, T1_270, T1_360, 270, 360) + theta0;}
    if (theta1 > 360) {theta1 = theta1 - 360;}


    theta0 = theta0 * M_PI/180;
    theta1 = theta1 * M_PI/180;

    if (theta0 < theta1) {deltaM = theta1 - theta0;}
    else if (theta0 > theta1 && theta0 >= (270 * M_PI/180) && theta0 <= (360 * M_PI/180) && theta1 >= 0 && theta1 <= (270 * M_PI/180))
    {deltaM = (360*M_PI/180) - theta0 + theta1;}
    else {deltaM = (360*M_PI/180) - abs(theta1 - theta0);}

    // PID Control
    dt = (millis() - previousTime) / 1000;

    errorE = -1*(phiDes - theta0);
    if (errorE > M_PI) {errorE = -1*(2*M_PI - errorE);}
    if (errorE < -M_PI) {errorE = 1*(2*M_PI + errorE);}

    derivative = (errorE - previousError) / dt;
    if (derivative > 10) {derivative = 10;}
    else if (derivative < -10) {derivative = -10;}
    
    PID_control_O = Kp*errorE;// + Kd*derivative;  
    if (PID_control_O > 50) {PID_control_O = 50;}
    else if (PID_control_O < -50) {PID_control_O = -50;}
    previousError = errorE;
    previousTime = millis();
    
    // Motor Control
    int lowerR = 145;
    int lowerL = 145;
    RIGHT_MOTOR(lowerR);                   
    LEFT_MOTOR(lowerL); 

    int flexADC_0 = analogRead(A2);
    float ang_cant0 = map(flexADC_0, STRAIGHT_0, BEND_0, 40, 90);
    int flexADC_1 = analogRead(A3);
    float ang_cant1 = map(flexADC_1, STRAIGHT_1, BEND_1, 40, 90);

    Serial.print("PID_control_O: "); Serial.print(PID_control_O); Serial.println(); 
    Serial.print("ang_cant0: "); Serial.print(ang_cant0); Serial.print("  "); 
    Serial.print("ang_cant1: "); Serial.print(ang_cant1); Serial.println(); Serial.println();
  /*
    // Stop Robot if flex sensor angle is less than 60 or greater than 85 degrees
    if (ang_cant0 < 50 || ang_cant1 < 60 || ang_cant0 > 85 || ang_cant1 > 85)  
    {
      Serial.println("STOPPED ROBOT: ");
      analogWrite(A16, 0);
      analogWrite(A15, 0);   
      analogWrite(13, 0);
      analogWrite(A14, 0);   
      delay(1000);
    };
*/
    delay(250); 
  }  

  Serial.println("DONE:");     
  analogWrite(A16, 0);
  analogWrite(A15, 0);   
  analogWrite(13, 0);
  analogWrite(A14, 0);   
  delay(1000);
}

void DRIVE()
{ 
  analogWrite(A16, Right);
  analogWrite(A15, 0);   
  analogWrite(13, Left);
  analogWrite(A14, 0);  
  delay(100); 

  analogWrite(A16, 0);
  analogWrite(A15, 0);   
  analogWrite(13, 0);
  analogWrite(A14, 0);  
  delay(1000); 
}


//Function to momag_VE Right Motor 
void RIGHT_MOTOR(int lower) {
  // Pin A16 - Right, Forward
  if (PID_control_O > 0) {
    if (PID_control_O >= lower && PID_control_O <= 255) {
      analogWrite(A16, PID_control_O);
      analogWrite(A15, 0);
    }
    else if (PID_control_O >= 0 && PID_control_O < lower) {
      analogWrite(A16, lower);
      analogWrite(A15, 0);
    }
    else if (PID_control_O > 255) {
      analogWrite(A16, 255);
      analogWrite(A15, 0);
    }
  }
  // Pin A15 - Right, Backward
  if (PID_control_O < 0) {
    if (abs(PID_control_O) >= lower && abs(PID_control_O) <= 255) {
      analogWrite(A16, 0);
      analogWrite(A15, PID_control_O);
    }
    else if (abs(PID_control_O) >= 0 && abs(PID_control_O) < lower) {
      analogWrite(A16, 0);
      analogWrite(A15, lower);
    }
    else if (abs(PID_control_O) > 255) {
      analogWrite(A16, 0);
      analogWrite(A15, 255);
    }
  }
}


//Function to momag_VE Left Motor
void LEFT_MOTOR(int lower) {
  // Pin 13 - Left, Forward
  if (PID_control_O < 0) {
    if (abs(PID_control_O) >= lower && abs(PID_control_O) <= 255) {
      analogWrite(13, abs(PID_control_O));
      analogWrite(A14, 0);
    }
    else if (abs(PID_control_O) >= 0 && abs(PID_control_O) < lower) {
      analogWrite(13, lower);
      analogWrite(A14, 0);
    }
    else if (abs(PID_control_O) > 255) {
      analogWrite(13, 255);
      analogWrite(A14, 0);
    }
  }
  // Pin A14 - Left, Backward
  if (PID_control_O > 0) {
    if (abs(PID_control_O) >= lower && abs(PID_control_O) <= 255) {
      analogWrite(13, 0);
      analogWrite(A14, PID_control_O);
    }
    else if (abs(PID_control_O) >= 0 && abs(PID_control_O) < lower) {
      analogWrite(13, 0);
      analogWrite(A14, lower);
    }
    else if (abs(PID_control_O) > 255) {
      analogWrite(13, 0);
      analogWrite(A14, 255);
    }
  }
}


void loop() {

  delay(2000);

  // print IP Address
  Serial.print("Assigned IP Address: ");
  Serial.println(WiFi.localIP());


  trueAngleDataBottom = as5600_0.readAngle() * AS5600_RAW_TO_RADIANS * 180/M_PI;
  trueAngleDataTop = as5600_1.readAngle() * AS5600_RAW_TO_RADIANS * 180/M_PI;
  Serial.print("The bottom's measured angle is: "); Serial.print(trueAngleDataBottom); Serial.print("  "); Serial.print("The tops's measured angle is: "); Serial.print(trueAngleDataTop); Serial.println();


  // Calibrates the angle of the bottom encoder so that it makes sense on 
  // a scale of 0-360 counter-clockwise off of the +x-axis
  // The calibration values differs from agent to agent on
  if((trueAngleDataBottom > encBottom_90) && (trueAngleDataBottom <= encBottom_0)) {
    calibratedAngleDataBottom = map(trueAngleDataBottom, encBottom_0, encBottom_90, 0, 90);
  }
  else if ((trueAngleDataBottom > encBottom_180) && (trueAngleDataBottom <= encBottom_90)) {
    calibratedAngleDataBottom = map(trueAngleDataBottom, encBottom_90, encBottom_180, 90, 180);
  }
  else if ((trueAngleDataBottom > encBottom_270) && (trueAngleDataBottom <= encBottom_180)) {
    calibratedAngleDataBottom = map(trueAngleDataBottom, encBottom_180, encBottom_270, 180, 270);
  }
  else if ((trueAngleDataBottom > encBottom_360) && (trueAngleDataBottom <= encBottom_270)) {
    calibratedAngleDataBottom = map(trueAngleDataBottom, encBottom_270, encBottom_360, 270, 360);
  }

  // Calibrates the angle of the top encoder so that it makes sense on 
  // a scale of 0-360 counter-clockwise off of the +x-axis
  // The calibration values differs from agent to agent on
  if((trueAngleDataTop < encTop_180_first) && (trueAngleDataTop <= 0)) {
    trueAngleDataTop = trueAngleDataTop + 360;
  }
  if ((trueAngleDataTop > encTop_90) && (trueAngleDataTop <= trueAngleDataBottom)) {
    calibratedAngleDataTop = map(trueAngleDataTop, trueAngleDataBottom, encTop_90, 0, 90) + calibratedAngleDataBottom;
  }
  else if ((trueAngleDataTop > encTop_180_first) && (trueAngleDataTop <= encTop_90)) {
    calibratedAngleDataTop = map(trueAngleDataTop, encTop_90, encTop_180_first, 90, 180) + calibratedAngleDataBottom;
  }
  else if ((trueAngleDataTop > encBottom_360) && (trueAngleDataTop <= encBottom_270)) {
    calibratedAngleDataBottom = map(trueAngleDataTop, encBottom_270, encBottom_360, 270, 360);
  }



}
