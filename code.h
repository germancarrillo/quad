#ifndef _code_h_
#define _code_h_

#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <vector>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>

#include "giro/I2Cdev.h"
#include "giro/MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//
bool setup_pwm();
bool setup_orientaion();
void setup_mpu();
bool loop_mpu(double *yaw,double *pitch,double *roll);
int  read_mpu();
//

class Pupdate{          // Class to be used later for tracking
 public:
  uint date;                  
  uint time;

 public:
  Pupdate();
  virtual ~Pupdate(){ }
};

Pupdate::Pupdate(){
  date=0;
  time=0;
}

class Quad{             // Quad class 
 private:
  double x;             // Positions 
  double y;             
  double z;

  double yaw;           // angles 
  double pitch;             
  double roll;

 public:
  Quad();
  virtual ~Quad(){ }
  void InitMotors();
  void StopMotors();
  void Throttle(uint motorID,uint throttle);
  void ThrottleAll(uint throttle);
  void ThrottleAllplusplus();
  void Stabilise(float timeS,double Kp,double Ki,double Kd);

  uint GetMeanThrottle(){ 
    return int((N_t+E_t+S_t+W_t)*0.25);
  };

  uint N_t;           // effective throttle for each motor          
  uint E_t;
  uint S_t;
  uint W_t;
};

Quad::Quad(){
  x=0; y=0; z=0; 
  yaw=0; pitch=0; roll=0;
  N_t=0; E_t=0; S_t=0; W_t=0; 
}

float difference(float Ax,float Ay,float Bx,float By);

int main();
#endif
