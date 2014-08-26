#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <cmath>
#include <sys/time.h>
#include <fstream>
#include <sstream>


#define take_off_T 1610  // take-off Throttle
#define min_spin_T 1581  // start-rotating-propellers 
#define max_allowed_T 1620 // max-allowed Throttle
#define min_allowed_T 1580 // min-allowed Throttle
#define tolerance 3.0   // stability tolerance in degrees
#define OUTPUT_READABLE_YAWPITCHROLL
#define S 0
#define W 4
#define E 5
#define N 7 
#define printlog true
#define PI 3.1415926

//for time measurments
struct timeval start, end, buff,t0,start_mpu,end_mpu,time_log,t_now,t_old;
long mtime, seconds, useconds; 
using namespace std;

uint badFIFOcount=0;

bool firstime=true;

bool isMC=false;
double N_t;
double W_t;
double E_t;
double S_t;

double Kp,Ki,Kd;
bool ramping=false;

double yaw0=0,pitch0=0,roll0=0;  
double yaw=0,pitch=0,roll=0;  

double dt; 
    
ofstream logfile;
ofstream myfile;

stringstream command;

double pitch_error=0, pitch_error_old=0, pitch_derivative=0, pitch_output=0, pitch_integral=0;
double roll_error=0,  roll_error_old=0,  roll_derivative=0,  roll_output=0,  roll_integral=0;
double yaw_error=0,   yaw_error_old=0,   yaw_derivative=0,   yaw_output=0,   yaw_integral=0;

FILE *pFile;

int GetMeanThrottle(){
  return int((N_t+E_t+S_t+W_t)*0.25);
};

bool Valid_Throttle(int throttle){
  if(ramping==false && throttle<min_allowed_T){ std::cout<<"ERROR: min throttle excceed min_spin_T = "<<min_spin_T<<" asked_T="<<throttle<<std::endl; return false;}
  if(ramping==false && throttle>max_allowed_T){ std::cout<<"ERROR: max throttle excceed max_allowed_T = "<<max_allowed_T<<" asked_T="<<throttle<<std::endl; return false;}
  return true;
}

void  Throttle(int motorID,int throttle){
  if(Valid_Throttle(throttle)==false) return;
  if     (motorID==N)N_t=throttle;
  else if(motorID==S)S_t=throttle;
  else if(motorID==W)W_t=throttle;
  else if(motorID==E)E_t=throttle;

  if(firstime==true){
    gettimeofday(&start, NULL);
    t0=start;
    buff=start;
    firstime=false;
  }

  if(isMC==false){
    //echo 0=1500 > /dev/servoblaster
    command.str(""); command<<"echo "<<motorID<<"="<<throttle<<" > /dev/servoblaster"<<endl;
    const char *char_command = command.str().c_str();
    //cout<<char_command<<endl;
    system(char_command);
    //pFile = fopen ("/dev/servoblaster","w");
    //fprintf(pFile,"%d=%d\n",motorID,throttle);
    //fclose(pFile);
  }else{
    pFile = fopen ("servoblasterMC.txt","a");
    gettimeofday(&start, NULL);
    seconds  = start.tv_sec  - buff.tv_sec; useconds = start.tv_usec - buff.tv_usec;buff=start;
    mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5; 
    fprintf(pFile,"%d=%d %ld\n",motorID,throttle,mtime);
    fclose(pFile);
  }
}

void ThrottleAllplusplus(){
  Throttle(N,N_t+1); 
  Throttle(E,E_t+1); 
  Throttle(S,S_t+1); 
  Throttle(W,W_t+1); 
}

void  ThrottleAll(int throttle){
  Throttle(N,throttle); 
  Throttle(E,throttle); 
  Throttle(S,throttle); 
  Throttle(W,throttle); 
}

void  InitMotors(){
  cout<<"INFO: Init Motors"<<endl;
  ramping=true;
  ThrottleAll(0);
  usleep(1000000);
  ThrottleAll(1500);
  ramping=false;
  usleep(1000000);
}

void  StopMotors(){
  cout<<"INFO: Stop Motors"<<endl;
  ramping=true;
  ThrottleAll(0);
  ramping=false;
}

void Ramping(int motorID,int final_Throttle){
  if(Valid_Throttle(final_Throttle)==false) return;
  ramping=true;
  int initial_Throttle=0;
  if     (motorID==N){initial_Throttle=N_t; cout<<"INFO: Ramping N from "<<initial_Throttle<<" to "<<final_Throttle<<endl;}
  else if(motorID==S){initial_Throttle=S_t; cout<<"INFO: Ramping S from "<<initial_Throttle<<" to "<<final_Throttle<<endl;}
  else if(motorID==W){initial_Throttle=W_t; cout<<"INFO: Ramping W from "<<initial_Throttle<<" to "<<final_Throttle<<endl;}
  else if(motorID==E){initial_Throttle=E_t; cout<<"INFO: Ramping E from "<<initial_Throttle<<" to "<<final_Throttle<<endl;}
  else cout<<"WARNING  WRONG motorID : "<<motorID<<endl;
  
  if(initial_Throttle==final_Throttle) return;
  
  if(initial_Throttle<final_Throttle){
    for(int throttle=initial_Throttle;throttle<=final_Throttle;throttle++){
      Throttle(motorID,throttle);
      if(throttle<=final_Throttle-4) usleep(50000); 
    }
  }else{
    for(int throttle=initial_Throttle;throttle>=final_Throttle;throttle--){
      Throttle(motorID,throttle);
      if(throttle>=final_Throttle+4) usleep(50000); 
    }
  }
  ramping=false;
}

void RampingAll(int final_Throttle){
  if(Valid_Throttle(final_Throttle)==false) return;
  int initial_Throttle_N=N_t;
  int initial_Throttle_S=S_t;
  int initial_Throttle_W=W_t;
  int initial_Throttle_E=E_t;
  
  double steps=10;
  
  for(double t=1;t<=steps;t++){
    Ramping(N,initial_Throttle_N+int(double(final_Throttle-initial_Throttle_N)*t/steps));
    Ramping(E,initial_Throttle_E+int(double(final_Throttle-initial_Throttle_E)*t/steps));
    Ramping(S,initial_Throttle_S+int(double(final_Throttle-initial_Throttle_S)*t/steps));
    Ramping(W,initial_Throttle_W+int(double(final_Throttle-initial_Throttle_W)*t/steps));
  }
}

void  Stabilise(double timeS,double Kp,double Ki,double  Kd,int value_T){   // PID

  gettimeofday(&t_now, NULL);
  seconds  = t_now.tv_sec  - t_old.tv_sec; useconds = t_now.tv_usec - t_old.tv_usec,t_old=t_now;
  dt= ((seconds) * 1000 + useconds/1000.0) + 0.5;

  pitch_error      = fabs(pitch0 - pitch) > tolerance? pitch0 - pitch :0;
  pitch_integral   = pitch_integral + pitch_error*dt;
  pitch_derivative = (pitch_error - pitch_error_old)/dt;
  pitch_output     = int(Kp*pitch_error + Ki*pitch_integral + Kd*pitch_derivative);	
  pitch_error_old  = pitch_error;
  
  roll_error       = fabs(roll0 - roll) > tolerance? roll0 - roll :0;
  roll_integral    = roll_integral + roll_error*dt;
  roll_derivative  = (roll_error - roll_error_old)/dt;
  roll_output      = int(Kp*roll_error + Ki*roll_integral + Kd*roll_derivative);	
  roll_error_old   = roll_error;
  
  yaw_error        = fabs(yaw0 - yaw) > tolerance? yaw0 - yaw :0;
  yaw_integral     = yaw_integral + yaw_error*dt;
  yaw_derivative   = (yaw_error - yaw_error_old)/dt;
  yaw_output       = int(Kp*yaw_error + Ki*yaw_integral + Kd*yaw_derivative);	
  yaw_error_old    = yaw_error;
  
  Throttle(N,value_T+pitch_output); 
  Throttle(S,value_T-pitch_output);
  Throttle(E,value_T+roll_output);  
  Throttle(W,value_T-roll_output);
}
