#include "PinNames.h"
#include "ThisThread.h"
#include "mbed.h"
#include <cmath>

const double wheel_period = 10;
const double wheel_r = 200;

PwmOut motor1_1(PA_0);
PwmOut motor1_2(PA_1);
PwmOut motor2_1(PA_2);
PwmOut motor2_2(PA_3);
PwmOut motor3_1(PA_4);
PwmOut motor3_2(PA_5);

Ticker ticker_robot;

void drive_wheel(double *speed); // speed of x,y,r on the base coordinate system
                                 // of drive base
void drive_robot();

double speed_base[3];

// main() runs in its own thread in the OS
int main() {
  motor1_1.period(wheel_period);
  motor1_2.period(wheel_period);
  motor2_1.period(wheel_period);
  motor2_2.period(wheel_period);
  motor3_1.period(wheel_period);
  motor3_2.period(wheel_period);

  ticker_robot.attach(&drive_robot, 20ms);

  for (;;) {
    speed_base[0] = 0.5;
    speed_base[1] = 0.5;
    speed_base[2] = 0.25;
  }
}

void drive_wheel(double *speed) {

  printf("%f %f %f \n", speed[0], speed[1], speed[2]);

  double speed_wheel[3] = {
      // the speed of each motor
      1.0 * speed[0] + 0.0 * speed[1] + wheel_r * speed[2],
      -0.5 * speed[0] + sqrt(3) / 2.0 * speed[1] + wheel_r * speed[2],
      -0.5 * speed[0] - sqrt(3) / 2.0 * speed[1] + wheel_r * speed[2]};

  for (int i = 0; i < 3; i++) {
    if (speed_wheel[i] <= -0.99) {
      speed_wheel[i] = -0.99;
    } else if (speed_wheel[i] >= 0.99) {
      speed_wheel[i] = 0.99;
    }
  }

  if (speed_wheel[0] < 0) {
    motor1_1.write(0);
    motor1_2.write(-speed_wheel[0]);
  } else {
    motor1_1.write(speed_wheel[0]);
    motor1_2.write(0);
  }

  if (speed_wheel[1] < 0) {
    motor2_1.write(0);
    motor2_2.write(-speed_wheel[1]);
  } else {
    motor2_1.write(speed_wheel[1]);
    motor2_2.write(0);
  }

  if (speed_wheel[2] < 0) {
    motor3_1.write(0);
    motor3_2.write(-speed_wheel[2]);
  } else {
    motor3_1.write(speed_wheel[2]);
    motor3_2.write(0);
  }
}

void drive_robot() {
  drive_wheel(speed_base);
  // todo: more drive functions
}