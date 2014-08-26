#define take_off_T     795 // take-off Throttle              
#define setmin_spin_T  750 // setup                          
#define min_spin_T     790 // start-rotating-propellers      
#define max_allowed_T  820 // max-allowed Throttle           
#define min_allowed_T  793 // min-allowed Throttle           
#define tolerance      3.0 // stability tolerance in degrees 

#include "code.h"
#include "functions.cpp"      // all functions except StabiliseQuad  

using namespace std;

int main(int argc, char* argv[]){
  //PID variables
  Kp=0.10;
  Ki=0.00;
  Kd=0.00;
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
  ThrottleAll(min_spin_T);
  ThrottleAll(take_off_T);
  
  gettimeofday(&t_old, NULL);
  StabiliseQuad(20,Kp,Ki,Kd);
  
  ThrottleAll(min_spin_T);
  StopMotors();                       // stop motors       

  logfile.close();
  std::cout<<"-- DONE --"<<std::endl;
  return 0;
}


//-------------------------------- Functions ---------------------------------- //

void StabiliseQuad(uint stabilizing_attemps,double Kp,double Ki,double  Kd){   // PID
  for(uint count=0; count<stabilizing_attemps; count++){

    while(loop_mpu(&yaw,&pitch,&roll)){
      if(fabs(pitch0 - pitch) > tolerance || fabs(roll0 - roll) > tolerance){

	gettimeofday(&t_now, NULL);
	seconds  = t_now.tv_sec  - t_old.tv_sec; 
	useconds = t_now.tv_usec - t_old.tv_usec;
	t_old=t_now;
	dt= ((seconds) * 1000 + useconds/1000.0) + 0.5;

	cout<< "dt = t_now=" << t_now.tv_sec << " - t_old= " << t_old.tv_sec <<" = dt ="<<dt<<endl;

	pitch_error      = fabs(pitch0 - pitch) > tolerance? pitch0 - pitch :0;
	pitch_integral   = pitch_integral + pitch_error*dt;
	pitch_derivative = (pitch_error - pitch_error_old)/dt;
	pitch_output     = int(Kp*pitch_error + Ki*pitch_integral + Kd*pitch_derivative);	
	pitch_error_old  = pitch_error;
	
	roll_error      = fabs(roll0 - roll) > tolerance? roll0 - roll :0;
	roll_integral   = roll_integral + roll_error*dt;
	roll_derivative = (roll_error - roll_error_old)/dt;
	roll_output     = int(Kp*roll_error + Ki*roll_integral + Kd*roll_derivative);	
	roll_error_old  = roll_error;

	yaw_error      = fabs(yaw0 - yaw) > tolerance? yaw0 - yaw :0;
	yaw_integral   = yaw_integral + yaw_error*dt;
	yaw_derivative = (yaw_error - yaw_error_old)/dt;
	yaw_output     = int(Kp*yaw_error + Ki*yaw_integral + Kd*yaw_derivative);	
	yaw_error_old  = yaw_error;

	std::cout.precision(3);
	std::cout<<"INFO: MPU read in:"<<badFIFOcount<<" attempts "<<std::endl; 
	std::cout<<"INFO: Pitch Roll and Yaw        :"<<pitch_error <<" \t "<<roll_error <<" \t "<<yaw_error <<std::endl; 
	std::cout<<"INFO: Pitch Roll and Yaw Outputs:"<<pitch_output<<" \t "<<roll_output<<" \t "<<yaw_output<<std::endl; 

	Throttle(N,take_off_T+pitch_output); Throttle(S,take_off_T-pitch_output);
	Throttle(E,take_off_T+roll_output);  Throttle(W,take_off_T-roll_output);

	//if(fabs(pitch0 - pitch) > tolerance){
	//  Throttle(N,min_spin_T+20+pitch_output); Throttle(S,min_spin_T+20-pitch_output);
	//}
	//if(fabs(roll0 - roll) > tolerance){
	// Throttle(E,min_spin_T+20+roll_output);  Throttle(W,min_spin_T+20-roll_output);
	//}
      }else{
	cout<<"INFO: ---- finish stabilise ---- count:"<<count<<endl;  
	break;
      }
    }
  }
}




