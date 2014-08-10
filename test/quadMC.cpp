#include "../interface/quad.h"

double Kp=0.25;
double Ki=0.02;
double Kd=0.25;

int main () {
  logfile.open("log.txt");
  logfile<<"Conf: "<<Kp<<" "<<Ki<<" "<<Kd<<endl;

  
  InitMotors();                       // start motors pulse 1500us  
  ThrottleAll(min_spin_T);
  RampingAll(take_off_T);
  
  cout<<"INFO:Simulating"<<endl;{ 
    for(double t=0;t<=300;t++){//This is MC for the input angles
      double w=2*PI/50;
      pitch=20*sin(w*t);
      roll=30*sin(2*w*t);
      Stabilise(1,Kp,Ki,Kd,take_off_T);//the output will be written in the angle variables
    }
  }

  RampingAll(take_off_T);
  StopMotors();                       // stop motors  

  return 0;
}
