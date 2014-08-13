#include"code.h"
#include"interface/quad.h"

#define OUTPUT_READABLE_YAWPITCHROLL

using namespace std;

//-------------------------------- Functions ---------------------------------- //

void StabiliseMPU(uint stabilizing_attemps,double Kp,double Ki,double  Kd){   // PID
  for(uint count=0; count<stabilizing_attemps; count++){
    
   double dt=1; 
    
    double pitch_error=0, pitch_error_old=0, pitch_derivative=0, pitch_output=0, pitch_integral=0;
    double roll_error=0,  roll_error_old=0,  roll_derivative=0,  roll_output=0,  roll_integral=0;
    double yaw_error=0,   yaw_error_old=0,   yaw_derivative=0,   yaw_output=0,   yaw_integral=0;

    while(loop_mpu(&yaw,&pitch,&roll)){
      if(fabs(pitch0 - pitch) > tolerance || fabs(roll0 - roll) > tolerance){
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

	//Throttle(N,take_off_T+pitch_output); Throttle(S,take_off_T-pitch_output);
	//Throttle(E,take_off_T+roll_output);  Throttle(W,take_off_T-roll_output);

	if(fabs(pitch0 - pitch) > tolerance){
	  Throttle(N,min_spin_T+20+pitch_output); Throttle(S,min_spin_T+20-pitch_output);
	}
	if(fabs(roll0 - roll) > tolerance){
	  Throttle(E,min_spin_T+20+roll_output);  Throttle(W,min_spin_T+20-roll_output);
	}
      }else{
	cout<<"INFO: ---- finish stabilise ---- count:"<<count<<endl;  
	break;
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////

bool setup_pwm(){
  if(true){ 
    std::cout<<std::endl<<std::endl<<"INFO: Invoking ServoBlaster"<<std::endl;   
    system("/root/code/servoblaster/servod"); // sourcing ServoBlaster for pulse width control
  }
  else{ std::cout<<"FATAL: mpu: failed to retrieve angles, can't continue!"<<std::endl; return 0; }
  return 1;
};

bool setup_orientaion(){
  system("echo i2cdetect -y 1; i2cdetect -y 1");         // scanning address for i2c protocol
  setup_mpu();                                           // giro & accelerometer setup
  usleep(250000);                                        // sleep 0.25s  
  loop_mpu(&yaw0,&pitch0,&roll0);  usleep(500000);       // starting values (twice) must be in a horizontal position  
  if(loop_mpu(&yaw0,&pitch0,&roll0)){
    std::cout<<std::endl<<std::endl<<"INFO: Reading initial angles::: yaw0:"<<yaw0<<" pitch0:"<<pitch0<<" roll0:"<<roll0<<std::endl; 
    cout<<"Hacking initial angles to 0,0,0"<<endl;
    pitch0=0;
    roll0=0;
  }
  else{ 
    std::cout<<"FATAL: mpu: failed to retrieve angles, can't continue!"<<std::endl; return 0; 
  }
  return 1;
};


void setup_mpu() {
  // initialize device
  printf("Initializing I2C devices...\n");
  mpu.initialize();
  // verify connection
  printf("Testing device connections...\n");
  printf(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");
  // load and configure the DMP
  printf("Initializing DMP...\n");
  devStatus = mpu.dmpInitialize();    
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    printf("Enabling DMP...\n");
    mpu.setDMPEnabled(true);
    
    // enable Arduino interrupt detection
    //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    //attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    printf("DMP ready!\n");
    dmpReady = true;
    
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    printf("DMP Initialization failed (code %d)\n", devStatus);
  }
}

bool loop_mpu(double *yaw,double *pitch,double *roll){
  // if programming failed, don't try to do anything
  if (!dmpReady) return 0;
    
  bool badFIFO=true;
  badFIFOcount=0;
  while(badFIFO){
    badFIFOcount++;
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
      
    if (fifoCount == 1024 || fifoCount <42 ) {
      mpu.resetFIFO();      
      //printf("FIFO overflow!\n");
      badFIFO=true;
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }else if (fifoCount >= 42) {
      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      #ifdef OUTPUT_READABLE_QUATERNION
      // display quaternion values in easy matrix form: w x y z
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      printf("quat %7.2f %7.2f %7.2f %7.2f    ", q.w,q.x,q.y,q.z);
      #endif
      
      #ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      *yaw   = ypr[0] * 180/M_PI;
      *pitch = ypr[1] * 180/M_PI;
      *roll  = ypr[2] * 180/M_PI;
      //printf("ypr  %7.2f %7.2f %7.2f    ",ypr[0] * 180/M_PI,ypr[1] * 180/M_PI,ypr[2] * 180/M_PI);
      if(printlog){
	gettimeofday(&time_log, NULL);
	seconds  = time_log.tv_sec - t0.tv_sec; useconds = time_log.tv_usec - t0.tv_usec;
	cout<<"timelog: "<<time_log.tv_sec<<" "<<time_log.tv_usec<<endl;
	mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5; 
	logfile<<"to_plot: "<<*roll<<" "<<*pitch<<" "<<*yaw<<" "<<N_t<<" "<<S_t<<" "<<E_t<<" "<<W_t<<" "<<mtime<<endl; 
      }
      return 1;
      #endif     
      printf("\n");
      badFIFO=false;
    }
  }
  return 0;
}

int main(int argc, char* argv[]){
  //PID variables
  double Kp=0.10;
  double Ki=0.00;
  double Kd=0.00;
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
  
  StabiliseMPU(50,Kp,Ki,Kd);
  
  ThrottleAll(min_spin_T);
  StopMotors();                       // stop motors       

  logfile.close();
  std::cout<<"-- DONE --"<<std::endl;
  return 0;
}

