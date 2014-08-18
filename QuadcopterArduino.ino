#include <Servo.h>
#include <PID_v1.h>

#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

#include "CommunicationUtils.h"
#include "FreeSixIMU.h"
#include <Wire.h>

float q[4];

FreeSixIMU my3IMU = FreeSixIMU();

double rsp, rip, rop;
double rsn, rin, ron;
double psp, pip, pop;
double psn, pin, pon;

PID rp(&rip, &rop, &rsp, 100, 0, 0, DIRECT);
PID rn(&rin, &ron, &rsn, 100, 0, 0, DIRECT);
PID pp(&pip, &pop, &psp, 100, 0, 0, DIRECT);
PID pn(&pin, &pon, &psn, 100, 0, 0, DIRECT);

Servo sfr;
Servo sfl;
Servo sbr;
Servo sbl;

double fr = 1200;
double fl = 1200;
double br = 1200;
double bl = 1200;

int pfr = 1000;
int pfl = 1000;
int pbr = 1000;
int pbl = 1000;

void setup() {
  sfr.attach(6);
  sfl.attach(9);
  sbr.attach(5);
  sbl.attach(3);

  Serial.begin(115200);
  Wire.begin();

  delay(5);
  my3IMU.init();
  delay(5);

  rp.SetMode(AUTOMATIC);
  rn.SetMode(AUTOMATIC);
  pp.SetMode(AUTOMATIC);
  pn.SetMode(AUTOMATIC);

  rp.SetOutputLimits(-50, 50);
  rn.SetOutputLimits(-50, 50);
  pp.SetOutputLimits(-50, 50);
  pn.SetOutputLimits(-50, 50);

  setup_motor();
}

void loop() { 
  pid();

  //print(1200+rop+pop, 1200+ron+pop, 1200+rop+pon, 1200+ron+pon);

  sfr.writeMicroseconds(1200 + rop + pop);
  sfl.writeMicroseconds(1200 + ron + pop);
  sbr.writeMicroseconds(1200 + rop + pon);
  sbl.writeMicroseconds(1200 + ron + pon);

  delay(60);
}

void setup_motor() {
  sfr.writeMicroseconds(1000);
  sfl.writeMicroseconds(1000);
  sbr.writeMicroseconds(1000);
  sbl.writeMicroseconds(1000);
  delay(15000);
}

void pid() {
  my3IMU.getQ(q);

  rip = q[1];
  rin = -q[1];
  pip = (q[2]+0.03);
  pin = -(q[2]+0.03);

  rp.Compute();
  rn.Compute();
  pp.Compute();
  pn.Compute();
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