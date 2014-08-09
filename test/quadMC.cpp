#include "../interface/quad.h"

int main () {
  logfile.open("log.txt");
  logfile<<"Conf: "<<Kp<<" "<<Ki<<" "<<Kd<<endl;
  
  cout<<"INFO:InitMotors"<<endl; 
  InitMotors();
  cout<<"INFO:All to min_spin_T"<<endl; 
  ThrottleAll(min_spin_T);
  cout<<"INFO:All to take_off_T"<<endl; 
  RampingAll(take_off_T); 
  cout<<"INFO:Simulating"<<endl;{ 
    for(double t=0;t<=300;t++){//This is MC for the input angles
      double w=2*PI/50;
      pitch=20*sin(w*t);
      roll=30*sin(2*w*t);
      Stabilise(1,Kp,Ki,Kd,take_off_T);//the output will be written in the angle variables
    }
  }
  cout<<"INFO:All to take_off_T"<<endl; 
  RampingAll(take_off_T); 
  cout<<"INFO:All to min_spin_T"<<endl; 
  RampingAll(min_spin_T);
  cout<<"INFO:StopMotors"<<endl; StopMotors();                 
  
  return 0;
}
