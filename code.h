#ifndef _code_h_
#define _code_h_

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <vector>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/time.h>

#include "giro/I2Cdev.h"
#include "giro/MPU6050_6Axis_MotionApps20.h"

#define S 0 //0
#define W 1 //4
#define E 2 //5
#define N 3 //7

#define printlog true
#define PI 3.1415926

#define OUTPUT_READABLE_YAWPITCHROLL

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
//for time measurments                                                                                                                                                                   
struct timeval start, end, buff,t0,start_mpu,end_mpu,time_log,t_now,t_old;
long mtime, seconds, useconds;
using namespace std;

uint badFIFOcount=0;

bool firstime=true;

double N_t;
double W_t;
double E_t;
double S_t;

double Kp,Ki,Kd;
bool ramping=false;

double yaw0=0,pitch0=0,roll0=0;
double yaw=0,pitch=0,roll=0;

double dt;
double dt_old;
double dt_oldd;
double dt_olddd;

ofstream logfile;
ofstream myfile;
 
stringstream command;

double pitch_error=0, pitch_error_old=0, pitch_error_oldd=0, pitch_error_olddd=0, pitch_derivative=0, pitch_output=0, pitch_integral=0;
double roll_error=0,  roll_error_old=0,  roll_error_oldd=0,  roll_error_olddd=0,  roll_derivative=0,  roll_output=0,  roll_integral=0;
double yaw_error=0,   yaw_error_old=0,   yaw_error_oldd=0,   yaw_error_olddd=0,   yaw_derivative=0,   yaw_output=0,   yaw_integral=0;

FILE *pFile;

// 
// functions to setup/read devices  
//
bool setup_pwm();
bool setup_orientaion();
void setup_mpu();
bool loop_mpu(double *yaw,double *pitch,double *roll);
//
// functions to setup/ramp and modify motors
//
void  InitMotors();
float GetMeanThrottle();
float Valid_Throttle(float throttle);
void  Throttle(int motorID,float throttle);
void  ThrottleAll(float throttle);
void  StopMotors();
//
void  StabiliseQuad(uint stabilizing_attemps,double Kp,double Ki,double  Kd);

#endif
