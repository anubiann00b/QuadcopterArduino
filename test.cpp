#include <stdio.h>
#include <iostream>

using namespace std;

double lastInput = 0;
double errSum = 0;
double kp = 1;
double ki = 0;
double kd = 0;

double computePid(double);

int main(int c, char** v) {
    while (true) {
        double input;
        cin>>input;
        printf("%f\n\n", computePid(input));
    }
}

double computePid(double input, double setpoint, double errSum, double lastInput) {
    double error = setpoint - input;
    printf("%f %f\n", setpoint, input);
    double dIn = lastInput - input;
    errSum += error;

    lastInput = input;

    return kp*error + ki*errSum + kd*dIn;
}