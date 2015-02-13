#include <Servo.h>

#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

#include <Wire.h>
#include "CommunicationUtils.h"

float q[4];

FreeSixIMU my3IMU = FreeSixIMU();

Servo frontRight;
Servo frontLeft;
Servo backRight;
Servo backLeft;

int kp = 1;
int ki = 0;
int kd = 0;

double lastInput = 0;
double errSum = 0;

void setup() {
  frontRight.attach(6);
  frontLeft.attach(9);
  backRight.attach(5);
  backLeft.attach(3);

  Serial.begin(19200);
  Wire.begin();

  delay(5);
  my3IMU.init();
  delay(5);

  frontRight.writeMicroseconds(1000);
  frontLeft.writeMicroseconds(1000);
  backRight.writeMicroseconds(1000);
  backLeft.writeMicroseconds(1000);
  delay(3000);

  Serial.println("Initialized");
}

double lastInputRoll = 0;
double errSumRoll = 0;

double lastInputPitch = 0;
double errSumPitch = 0;

void loop() { 
  my3IMU.getQ(q);

  double rollInput = q[1];
  double pitchInput = q[2];

  print(computePid(rollInput, 0, &errSumRoll, &lastInputRoll), computePid(pitchInput, 0, &errSumPitch, &lastInputPitch));

  //frontRight.writeMicroseconds(1200 + rollOutput + pitchOutput);
  //frontLeft.writeMicroseconds(1200 - rollOutput + pitchOutput);
  //backRight.writeMicroseconds(1200 + rollOutput - pitchOutput);
  //backLeft.writeMicroseconds(1200 - rollOutput - pitchOutput);

  delay(60);
}

double computePid(double input, double setpoint, double *errSum, double *lastInput) {
    double error = setpoint - input;
    double dIn = *lastInput - input;
    *errSum += error;

    *lastInput = input;

    return kp*error + ki*(*errSum) + kd*dIn;
}

void print(double d1, double d2) {
  Serial.print(d1);
  Serial.print(" ");
  Serial.println(d2);
}

void print(double d1, double d2, double d3, double d4) {
  Serial.print(d1); Serial.print(" ");
  Serial.print(d2); Serial.print(" ");
  Serial.print(d3); Serial.print(" ");
  Serial.println(d4);
}