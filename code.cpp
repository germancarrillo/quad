#include"code.h"
#include"interface/quad.h"

#define OUTPUT_READABLE_YAWPITCHROLL

using namespace std;

//-------------------------------- Functions ---------------------------------- //


void StabiliseMPU(double timeS,double Kp,double Ki,double  Kd){   // PID
  for(double st=0; st<timeS; st+=0.1){
    uint readfail=0;
    uint dt=1; 
    
    double pitch_error=0, pitch_error_old=0, pitch_derivative=0, pitch_output=0, pitch_integral=0;
    double roll_error=0,  roll_error_old=0,  roll_derivative=0,  roll_output=0,  roll_integral=0;
    double yaw_error=0,   yaw_error_old=0,   yaw_derivative=0,   yaw_output=0,   yaw_integral=0;

    loop_mpu(&yaw,&pitch,&roll);

    while(fabs(pitch0 - pitch) > tolerance || fabs(roll0 - roll) > tolerance)
      if(loop_mpu(&yaw,&pitch,&roll)){
	
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
	std::cout<<"INFO: Pitch Roll and Yaw        :"<<pitch_error <<" \t "<<roll_error <<" \t "<<yaw_error <<std::endl; 
	std::cout<<"INFO: Pitch Roll and Yaw Outputs:"<<pitch_output<<" \t "<<roll_output<<" \t "<<yaw_output<<std::endl; 

	Throttle(N,take_off_T+pitch_output); Throttle(S,take_off_T-pitch_output);
	Throttle(E,take_off_T+roll_output);  Throttle(W,take_off_T-roll_output);
		
      }else{
	readfail++;
	if(readfail>10){ std::cout<<"ERROR: Can't read mpu, failed to stabilise!"<<std::endl;  break; } 
      }
  }
  std::cout<<"INFO: -------finish stabilise----------"<<std::endl;  
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
  usleep(500000);                                        // sleep 0.5s  
  loop_mpu(&yaw0,&pitch0,&roll0);  usleep(500000);       // starting values (twice) must be in a horizontal position  
  if(loop_mpu(&yaw0,&pitch0,&roll0)) std::cout<<std::endl<<std::endl<<"INFO: Reading initial angles::: yaw0:"<<yaw0<<" pitch0:"<<pitch0<<" roll0:"<<roll0<<std::endl; 
  else{ std::cout<<"FATAL: mpu: failed to retrieve angles, can't continue!"<<std::endl; return 0; }
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
  usleep(100000);
    if (!dmpReady) return 0;
    
    bool badFIFO=true;

    while(badFIFO){
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
	return 1;
      #endif     
	printf("\n");
	badFIFO=false;
      }
    }
    return 0;
}

int read_mpu(){
  double yaw=-999,pitch=-999,roll=-999;
  int readmpu_f=loop_mpu(&yaw,&pitch,&roll);
  if(readmpu_f) std::cout<<" yaw:"<<yaw<<" pitch:"<<pitch<<" roll:"<<roll<<std::endl; else std::cout<<" mpu: failed to retrieve angles"<<std::endl;
  return 0; 
}

int main(int argc, char* argv[]){
  //PID variables
  double Kp=0.25;
  double Ki=0.02;
  double Kd=0.25;
  //-------------

  // Check the number of parameters
  Kp=double(atof(argv[1]));
  Ki=double(atof(argv[2]));
  Kd=double(atof(argv[3]));
    
  cout<<"Warning running with this conf: "<<Kp<<" "<<Ki<<" "<<Kd<<endl;

  usleep(2000000);

  if(!setup_orientaion())return 0; 
  if(!setup_pwm())return 0; 

  logfile.open("log.txt");

  logfile<<"Conf: "<<Kp<<" "<<Ki<<" "<<Kd<<endl;
  
  InitMotors();                       // start motors pulse 1500us  
  ThrottleAll(min_spin_T);
  ThrottleAll(take_off_T);
  
  StabiliseMPU(1,Kp,Ki,Kd);

  ThrottleAll(take_off_T);
  StopMotors();                       // stop motors       

  logfile.close();
  std::cout<<"-- DONE --"<<std::endl;
  return 0;
}

