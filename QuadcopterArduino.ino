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

PID rp(&rip, &rop, &rsp, 10, 25, 5, DIRECT);
PID rn(&rin, &ron, &rsn, 10, 25, 5, DIRECT);
PID pp(&pip, &pop, &psp, 10, 25, 5, DIRECT);
PID pn(&pin, &pon, &psn, 10, 25, 5, DIRECT);

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
  sfr.attach(2);
  sfl.attach(3);
  sbr.attach(4);
  sbl.attach(5);

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
  
  //setup_motor();
}

void loop() { 
  pid();

  sfr.writeMicroseconds(1200 + q[1]>0?pop:pon + q[2]>0?rop:ron);
  sfl.writeMicroseconds(1200 + q[1]>0?pop:pon + q[2]>0?ron:rop);
  sbr.writeMicroseconds(1200 + q[1]>0?pon:pop + q[2]>0?rop:ron);
  sbl.writeMicroseconds(1200 + q[1]>0?pon:pop + q[2]>0?ron:rop);

  delay(60);
}

void setup_motor() {
  sfr.writeMicroseconds(1000);
  sfl.writeMicroseconds(1000);
  sbr.writeMicroseconds(1000);
  sbl.writeMicroseconds(1000);
  delay(10000);
}

void pid() {
  my3IMU.getQ(q);

  rip = q[1];
  rin = -q[1];
  pip = q[2];
  pin = -q[2];

  rp.Compute();
  rn.Compute();
  pp.Compute();
  pn.Compute();

  print(rop, ron, pop, pon);
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