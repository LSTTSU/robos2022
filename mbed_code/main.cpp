#include "PinNames.h"
#include "ThisThread.h"
#include "mbed.h"
#include <cmath>

const double wheel_period = 0.001;
const double wheel_r = 50;
const int arm_off = 1700;
const int arm_on = 1190;

PwmOut motor1_1(PB_13);  //
PwmOut motor1_2(PB_14);  // 
PwmOut motor2_1(PB_10); //
PwmOut motor2_2(PC_7);  //
PwmOut motor3_1(PA_6);  // 
PwmOut motor3_2(PA_5);  // 

PwmOut arm1(PB_7);

I2CSlave slave(D14, D15);
// Ticker ticker_i2c_slave
int i2c_buf[4] = {0};
int i2c_buf_old[4] = {0};

void drive_wheel(double *speed); // speed of x,y,r on the base coordinate system
                                 // of drive base

double speed_base[3] = {0}; // x, y, r
int i2c_counter = 0;

// main() runs in its own thread in the OS
int main() {
  motor1_1.period(wheel_period);
  motor1_2.period(wheel_period);
  motor2_1.period(wheel_period);
  motor2_2.period(wheel_period);
  motor3_1.period(wheel_period);
  motor3_2.period(wheel_period);

  arm1.period_ms(20);
  arm1.pulsewidth_us(arm_off);

  slave.address(0x53);

  //   ticker_i2c_slave.attach(&drive_robot, 20ms);

  while (1) {

    i2c_buf[i2c_counter] = slave.read();

    if (i2c_buf[i2c_counter] != -1) {
      i2c_counter += 1;
      if (i2c_counter > 3) {
        for (int i = 0; i < 3; i++) {
          speed_base[i] = (((double)i2c_buf[i] - 127.0) / 127.0);
          speed_base[1] *= 1;
          speed_base[2] *= 0.05;
          printf("%f ", speed_base[i]);
        }
        printf("\n");
        i2c_counter = 0;
        if (i2c_buf[3] != i2c_buf_old[3]) {
          if (i2c_buf[3] != 0x23) {
            arm1.pulsewidth_us(arm_off);
          } else {
            arm1.pulsewidth_us(arm_on);
          }
        }
        for (int i = 0; i < 4; i++) {
          i2c_buf_old[i] = i2c_buf[i];
        }
      }
    }

    drive_wheel(speed_base);
  }
}

void drive_wheel(double *speed) {

  double speed_wheel[3] = {
      // the speed of each motor
      1.0 * speed[0] + 0.0 * speed[1] + wheel_r * speed[2],
      -0.5 * speed[0] + sqrt(3) / 2.0 * speed[1] + wheel_r * speed[2],
      -0.5 * speed[0] - sqrt(3) / 2.0 * speed[1] + wheel_r * speed[2]};

//    printf("%f %f %f \n", speed_wheel[0], speed_wheel[1], speed_wheel[2]);

  for (int i = 0; i < 3; i++) {
    if (speed_wheel[i] <= -0.99) {
      speed_wheel[i] = -0.99;
    } else if (speed_wheel[i] >= 0.99) {
      speed_wheel[i] = 0.99;
    }
  }

//   speed_wheel[0] *= 1.1;

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
