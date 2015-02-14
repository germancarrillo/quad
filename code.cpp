#define take_off_T     797 // take-off Throttle              

#define max_allowed_T  810 // max-allowed Throttle           
#define min_allowed_T  790 // min-allowed Throttle           

#define tolerance      2.0 // stability tolerance in degrees 

#include "code.h"
#include "functions.cpp"      // all functions except StabiliseQuad  

using namespace std;

int main(int argc, char* argv[]){
  //PID variables
  Kp=0.050;
  Ki=0.000;
  Kd=0.000;
  //-------------

  // Check the number of parameters
  if(argc==4){
    Kp=double(atof(argv[1]));
    Ki=double(atof(argv[2]));
    Kd=double(atof(argv[3]));
    cout<<"INFO: Running with constants: "<<Kp<<" "<<Ki<<" "<<Kd<<endl;
  }    
  else cout<<"WARNING: Running with default constants: "<<Kp<<" "<<Ki<<" "<<Kd<<endl;
  
  if(!setup_orientaion())return 0; 
  if(!setup_pwm())return 0; 

  logfile.open("log.txt");
  logfile<<"Conf: "<<Kp<<" "<<Ki<<" "<<Kd<<endl;
  
  InitMotors();                       // start motors pulse 1500us    
  ThrottleAll(take_off_T);
  
  gettimeofday(&t_old, NULL);
  std::cout<<"INFO: Ready to stabilise"<<std::endl<<std::endl;
  StabiliseQuad(20,Kp,Ki,Kd);
  
  ThrottleAll(min_allowed_T);
  StopMotors();                       // stop motors       

  logfile.close();
  std::cout<<"-- DONE --"<<std::endl;
  return 0;
}


//-------------------------------- Functions ---------------------------------- //

void StabiliseQuad(uint stabilizing_attemps,double Kp,double Ki,double  Kd){   // PID
  for(uint count=0; count<stabilizing_attemps; count++){
    bool firstread=true;
    while(loop_mpu(&yaw,&pitch,&roll)){
      if(fabs(pitch0 - pitch) > tolerance || fabs(roll0 - roll) > tolerance){
	
	gettimeofday(&t_now, NULL);
	seconds  = t_now.tv_sec  - t_old.tv_sec; 
	useconds = t_now.tv_usec - t_old.tv_usec;
	t_old=t_now;
	dt= ((seconds) * 1000 + useconds/1000.0) + 0.5;
	
	std::cout<<"INFO: Time step dt:                   "<<dt<<" um"<<endl;

	dt=dt/1000.0;

	pitch_error       = fabs(pitch0 - pitch) > tolerance? pitch0 - pitch :0;
	pitch_integral    = pitch_integral + pitch_error*dt;
	pitch_derivative  = (pitch_error - pitch_error_old)/dt;
	pitch_output      = Kp*pitch_error + Ki*pitch_integral + Kd*pitch_derivative;	
	pitch_error_old   = firstread?0:pitch_error;
	pitch_error_oldd  = pitch_error_old;
	pitch_error_olddd = pitch_error_oldd;
	
	roll_error        = fabs(roll0 - roll) > tolerance? roll0 - roll :0;
	roll_integral     = roll_integral + roll_error*dt;
	roll_derivative   = (roll_error - roll_error_old)/dt;
	roll_output       = Kp*roll_error + Ki*roll_integral + Kd*roll_derivative;	
	roll_error_old    = firstread?0:roll_error;
	roll_error_oldd   = roll_error_old;
	roll_error_olddd  = roll_error_oldd;
	
	yaw_error        = fabs(yaw0 - yaw) > tolerance? yaw0 - yaw :0;
	yaw_integral     = yaw_integral + yaw_error*dt;
	yaw_derivative   = (yaw_error - yaw_error_old)/dt;
	yaw_output       = Kp*yaw_error + Ki*yaw_integral + Kd*yaw_derivative;	
	yaw_error_old    = firstread?0:yaw_error;
	yaw_error_oldd   = yaw_error_old;
	yaw_error_olddd  = yaw_error_oldd;

	dt_old   = firstread?0:dt;
	dt_oldd  = dt_old;
	dt_olddd = dt_oldd;
	
	std::cout.precision(4);
	std::cout<<"INFO: MPU read in:                    "<<badFIFOcount<<" attempts "<<std::endl<<std::endl; 
	std::cout<<"INFO: Actual throtles:                N:"<<N_t<<" S:"<<S_t<<" W:"<<W_t<<" E:"<<E_t<<std::endl; 
	std::cout<<"INFO: Pitch Roll and Yaw angles:      "<<pitch_error <<" \t "<<roll_error <<" \t "<<yaw_error <<std::endl; 
	//
	std::cout<<"INFO: Pitch"<<std::endl;
	std::cout<<"INFO: PID:                            "<<Kp<<"*"<<pitch_error<<"+"<<Ki<<"*"<<pitch_integral<<"+"<<Kd<<"*"<<pitch_derivative<<"="<<pitch_output <<std::endl;
	std::cout<<"INFO: |sin(alpha)|*output:            "<<fabs(sin(pitch_error/180.*M_PI))*pitch_output<<std::endl; 
	//
	std::cout<<"INFO: Roll"<<std::endl;
	std::cout<<"INFO: PID:                            "<<Kp<<"*"<<roll_error<<"+"<<Ki<<"*"<<roll_integral<<"+"<<Kd<<"*"<<roll_derivative<<"="<<roll_output <<std::endl;
	std::cout<<"INFO: |sin(alpha)|*output:            "<<fabs(sin(roll_error/180.*M_PI))*roll_output<<std::endl; 

	Throttle(N,N_t+pitch_output*fabs(sin(pitch_error/180.*M_PI))); Throttle(S,S_t-pitch_output*fabs(sin(pitch_error/180.*M_PI)));
	Throttle(E,E_t+roll_output *fabs(sin(roll_error /180.*M_PI))); Throttle(W,W_t-roll_output *fabs(sin(roll_error /180.*M_PI)));

	std::cout<<"INFO: New throtles:                   N:"<<N_t<<" S:"<<S_t<<" W:"<<W_t<<" E:"<<E_t<<std::endl; 
	std::cout<<endl<<"INFO: ----- "<<std::endl<<std::endl; 
	firstread=false;
      }else{
	cout<<"INFO: ---- finish stabilise ---- count:"<<count<<endl;  
	break;
      }
    }
  }
}




