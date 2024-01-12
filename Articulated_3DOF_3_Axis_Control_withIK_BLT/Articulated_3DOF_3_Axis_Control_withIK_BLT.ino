#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <DabbleESP32.h>

#define INCLUDE_GAMEPAD_MODULE

#define SERVOMIN  145 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  645 // This is the 'maximum' pulse length count (out of 4096)

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();  // create servo driver object

//Servo Driver
const uint8_t Servo1 = 0;
const uint8_t Servo2 = 1;
const uint8_t Servo3 = 2;
const uint8_t Servo4 = 7;

//Link Lengths
const float a1 = 9.5;
const float a2 = 10.5;
const float a3 = 15.5;

//Position
float r1 = 15.5;
float theta1 = 90.;
float z = 20.;
int grip = 135.;

//Increment
float axis_step = 0.5;
float angle_step = 2.;

void setup() {
  Serial.begin(115200);  // initialize serial communication
  Dabble.begin("NodeMCU ESP32S");       //set bluetooth name of your device
  pwm.begin();
  pwm.setPWMFreq(60);  // Set PWM frequency to 60 Hz
}

void loop() {
  //process input from device
  Dabble.processInput();

  //Control scheme variables
  int a = GamePad.getAngle();
  int b = GamePad.getRadius();

  // Controller Scheme

  //FORWARD + r1
  if (a > 60 && a < 120 || GamePad.isUpPressed()) {
    r1 = r1 + axis_step;
    if(r1 >= (a2 + a3)) {
      r1 = (a2 + a3);
    }
    delay(20);
  }
    //BACKWARD - r1
  if (a > 240 && a < 300 || GamePad.isDownPressed()) {
    r1 = r1 - axis_step;
    if(r1 <= 0.){
      r1 = 0.;
    }
    delay(20);
  }

    //LEFT + Base
  if (a > 150 && a < 210 || GamePad.isLeftPressed()) {
    theta1 = theta1 + angle_step;
    if(theta1 >= 180.) {
      theta1 = 180.;
    }
    delay(20);
  }
    //RIGHT - Base
  if (a > 330 || a < 30 && b != 0 || GamePad.isRightPressed()) {
    theta1 = theta1 - angle_step;
    if(theta1 <= 0.) {
      theta1 = 0.;
    }
    delay(20);
  }

    //UP DOWN + z
  if (GamePad.isTrianglePressed()) {
    z = z + axis_step;
    if(z >= (a1 + a2 + a3)) {
      z = (a1 + a2 + a3);
    }
    delay(20);
  } // - z
  if (GamePad.isCrossPressed()) {
    z = z - axis_step;
    if(z <= 0.) {
      z = 0.;
    }
    delay(20);
  }

  //Gripper open
  if (GamePad.isCirclePressed()) {
    grip = grip - angle_step;
    if (grip <= 110.) {
      grip = 110.;
    }
    delay(20);
  } //close
  if (GamePad.isSquarePressed()) {
    grip = grip + angle_step;
    if (grip >= 180.) {
      grip = 180.;
    }
    delay(20);
  }

  ///RESET to default
  if (GamePad.isStartPressed()) {
    reset();
  }
  
  //Compute Joint Angles
  IK(r1, theta1, z, grip);
}

  //Convert angle to pulse for PCA9685 Servo Driver
double angletoPulse(double angle) {
  float pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  return pulse;
}

void IK (double r1, double theta1, double z, double grip) {
  //convert theta1 to radians
  double theta1_r = (theta1 * 71.) / 4068.;

  //x and y axis based on r1 length and base angle
  double x = (r1 * cos(theta1_r));
  double y = (r1 * sin(theta1_r));

  //#3
  double r2 = (z - a1);

  //#4
  double phi1 = atan(r2/r1); 

  //#5
  double r3 = sqrt((r2*r2) + (r1*r1)); 

  //#6
  double acos_phi2 = (((a3*a3)-(a2*a2)-(r3*r3))/((-2.) * a2 * r3));

    //avoid nan error// arccos returns error if -1 > x > 1
  if (acos_phi2 > 1.) {
    acos_phi2 = 1.;
  } 
  else if (acos_phi2 < -1.) {
    acos_phi2 = -1.;
  }

  double phi2 = acos(acos_phi2);

  //#7
  double theta2 = (phi1 + phi2); 
  theta2 = degrees(theta2);

  //#8
  double acos_phi3 = (((r3*r3)-(a2*a2)-(a3*a3))/((-2.)*(a2)*(a3)));

    //avoid nan error// arccos returns error if -1 > x > 1
  if (acos_phi3 > 1.) {
    acos_phi3 = 1.;
  } 
  else if (acos_phi3 < -1.) {
    acos_phi3 = -1.;
  }
    //calculate after checks
  double phi3 = acos(acos_phi3);

  //#9
  //float theta3 = (phi3 - 180);
  double theta3 = degrees(phi3); //due to servo angle direction

  Serial.print("θ1: ");
  Serial.print(theta1);
  Serial.print(" ");
  Serial.print("θ2: ");
  Serial.print(theta2);
  Serial.print(" ");
  Serial.print("θ3: ");
  Serial.print(theta3);
  Serial.print(" ");
  Serial.print("R1: ");
  Serial.print(r1);
  Serial.print(" ");
  Serial.print("X-axis: ");
  Serial.print(x);
  Serial.print(" ");
  Serial.print("Y-axis: ");
  Serial.print(y);
  Serial.print(" ");
  Serial.print("Z-axis: ");
  Serial.print(z);
  Serial.println(" ");

  //Computed Joint Angles to PWM
  moveAngle(theta1, theta2, theta3, grip);
}

  //Write computed Joint Angles to Servo Driver for servo movement
void moveAngle(double theta1, double theta2, double theta3, double grip) {
  pwm.setPWM(Servo1, 0, angletoPulse(theta1) );
  pwm.setPWM(Servo2, 0, angletoPulse(theta2) );
  pwm.setPWM(Servo3, 0, angletoPulse(theta3) );
  pwm.setPWM(Servo4, 0, angletoPulse(grip) );
}

  //Reset to 90 degree servo position
void reset() {
  r1 = 15.5;
  theta1 = 90.;
  z = 20.;
  grip = 135.;
  IK(r1, theta1, z, grip);
}

