#include"code.h"

#define take_off_T 1625  // take-off Throttle
#define min_spin_T 1580  // start-rotating-propellers 
#define max_allowed_T 1660 // max-allowed Throttle
#define tolerance 2.0   // stability tolerance in degrees
#define OUTPUT_READABLE_YAWPITCHROLL
#define S 0
#define W 4
#define E 5
#define N 7 

double yaw0,pitch0,roll0;                                // initial angles offset - horizontal  
using namespace std;

ofstream myfile;
ofstream logfile;
bool ramped_up = false;

int main(){
  if(!setup_orientaion())return 0; 
  if(!setup_pwm())return 0; 
  Quad *quad = new Quad();                  // class QUAD to manage position, orientation, velocity, angular speeds etc...
  
  myfile.open("/dev/servoblaster");
  logfile.open("log.txt");

  quad->InitMotors();                       // start motors pulse 1500us  
  for(uint p=min_spin_T;p<=take_off_T;p++){              // ramp up throttle 
    std::cout<<"INFO: Starting -> Throttle at "<<int((p-1580))<<"%  pulse width:"<<p<<std::endl;
    quad->ThrottleAll(p);                   // throttle from 1580 to 1680us 
    if(p<=take_off_T-4) usleep(50000); 
  }

  quad->Stabilise(2,0.5,0.0,0.0); // (t,Kp,Ki,Kd)   
  // quad->Stabilise(2,0.1,0.1,0.1); // (t,Kp,Ki,Kd)   
  // quad->Stabilise(2,0.1,0.1,0.1); // (t,Kp,Ki,Kd)   

  for(uint p=quad->GetThrottle();p>=min_spin_T;p-=10){  // ramp down throttle 
    std::cout<<"INFO: Stopping -> Throttle at "<<p<<std::endl;
    quad->ThrottleAll(p);
    usleep(500000);                         // in microseconds
  }

  quad->StopMotors();                       // stop motors       

  delete quad;
  myfile.close();
  logfile.close();
  std::cout<<"-- DONE --"<<std::endl;
  return 0;
}
//--- Functions
float differecnce(float Ax,float Ay,float Bx,float By){return 1;}  

void  Quad::Throttle(uint motorID,int throttle){
  if(throttle>max_allowed_T){ std::cout<<"ERROR: max throttle excceed"<<std::endl; return;}
  if(throttle<(min_spin_T+10)){ std::cout<<"ERROR: max throttle excceed"<<std::endl; return;}
  myfile <<motorID<<"="<<throttle; myfile.seekp(0);
  
  if(printlog){
    double percentage_N=(N_t-min_spin_T)/(max_allowed_T-min_spin_T);
    double percentage_S=(S_t-min_spin_T)/(max_allowed_T-min_spin_T);
    double percentage_E=(E_t-min_spin_T)/(max_allowed_T-min_spin_T);
    double percentage_W=(W_t-min_spin_T)/(max_allowed_T-min_spin_T);
    //logfile<<"to_plot: "<<roll<<" "<<pitch<<" "<<yaw<<" "<<percentage_N<<" "<<percentage_S<<" "<<percentage_E<<" "<<percentage_W<<std::endl;
    logfile<<"to_plot: "<<roll<<" "<<pitch<<" "<<yaw<<" "<<N_t<<" "<<S_t<<" "<<E_t<<" "<<W_t<<std::endl; 
  }
}

void  Quad::ThrottleAllplusplus(){
  Quad::Throttle(N,N_t+1); 
  Quad::Throttle(E,E_t+1); 
  Quad::Throttle(S,S_t+1); 
  Quad::Throttle(W,W_t+1); 
}

void  Quad::ThrottleAll(int throttle){
  Quad::Throttle(N,throttle); 
  Quad::Throttle(E,throttle); 
  Quad::Throttle(S,throttle); 
  Quad::Throttle(W,throttle); 
}

void  Quad::InitMotors(){
  ramping=true;
  Quad::ThrottleAll(0);
  Quad::ThrottleAll(1500);
  ramping=false;
  usleep(1000000);
}

void  Quad::StopMotors(){
  ramping=true;
  Quad::ThrottleAll(0);
  ramping=false;
  usleep(200000);
}

void  Quad::Stabilise(float timeS,double Kp,double Ki,double  Kd){   // PID
  for(float st=0; st<timeS; st+=0.1){
    uint readfail=0;
    uint dt=1; 

    double pitch_error=0, pitch_error_old=0, pitch_derivative=0, pitch_output=0, pitch_integral=0;
    double roll_error=0,  roll_error_old=0,  roll_derivative=0,  roll_output=0,  roll_integral=0;
    double yaw_error=0,   yaw_error_old=0,   yaw_derivative=0,   yaw_output=0,   yaw_integral=0;

    loop_mpu(&yaw,&pitch,&roll);

    while(fabs(pitch0 - pitch) > tolerance || fabs(roll0 - roll) > tolerance
)
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

	Quad::Throttle(N,N_t-pitch_output); Quad::Throttle(S,S_t+pitch_output);
	Quad::Throttle(E,E_t-roll_output);  Quad::Throttle(W,W_t+roll_output);
		
      }else{
	readfail++;
	if(readfail>10){ std::cout<<"ERROR: Can't read mpu, failed to stabilise!"<<std::endl;  break; } 
      }
  }



	usleep(dt*1000);
	
      }else{
	readfail++;
	if(readfail>10){ std::cout<<"ERROR: Can't read mpu, failed to stabilise!"<<std::endl;  break; } 
      }
  }
  std::cout<<"-----------------"<<std::endl;  
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
