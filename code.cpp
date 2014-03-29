#define maxT 1625        // throttle limtis: esc+motors max 168   
#define minT 1580        // throttle limits: min 158 to start propellers  

#define limT 1660

#define tolerance 2.0   // stability tolerance in degrees

//#define OUTPUT_READABLE_QUATERNION
#define OUTPUT_READABLE_YAWPITCHROLL

#include"code.h"

#define size   10  

double yaw0,pitch0,roll0;

FILE *pFile;

using namespace std;

int main(){
  
  system("i2cdetect -y 1");        // scanning address for i2c protocol
  setup_mpu();                     // giro & accelerometer setup
  usleep(500000);                  // sleep 0.5s  
  loop_mpu(&yaw0,&pitch0,&roll0);  usleep(500000); // starting values (twice) must be in a horizontal position  
  if(loop_mpu(&yaw0,&pitch0,&roll0)) std::cout<<std::endl<<std::endl<<"INFO: Reading initial angles::: yaw0:"<<yaw0<<" pitch0:"<<pitch0<<" roll0:"<<roll0<<std::endl; 
  else{ std::cout<<"FATAL: mpu: failed to retrieve angles, can't continue!"<<std::endl; return 0; }
  
  Quad *quad = new Quad();                  // class QUAD position, orientation, effective and angular speeds etc...
  std::vector<Pupdate> P;  P.clear();       // position track -> to be used later 

  system("/root/code/servoblaster/servod"); // sourcing ServoBlaster for pulse width control
  
  quad->InitMotors();                       // start motors pulse 1500us  
  
  for(uint p=minT;p<=maxT;p++){              // ramp up throttle 
    Pupdate position;  
    std::cout<<"INFO: Starting -> Throttle at "<<int((p-1580)*100.)<<"%  pulse width:"<<p<<std::endl;
    quad->ThrottleAll(p);                   // throttle from 1580 to 1680us 
    usleep(200000);                         // in microseconds
  }

  usleep(1000000); 
  quad->ThrottleAllplusplus();
  quad->Stabilise(1);   

  //usleep(2000000); 
 
  /*
  quad->Stabilise(1); 
  quad->ThrottleAllplusplus();              // throttle ++              
  quad->Stabilise(1); 
  quad->ThrottleAllplusplus();              // throttle ++              
  quad->Stabilise(8); 
  */

  for(uint p=quad->GetThrottle();p>=minT;p-=10){  // ramp down throttle 
    std::cout<<"INFO: Stopping -> Throttle at "<<int((p-1580)*100.)<<"%  pulse width:"<<p<<std::endl;
    quad->ThrottleAll(p);
    usleep(500000);                         // in microseconds
  }

  quad->StopMotors();                       // stop motors       

  delete quad;
  std::cout<<"-- DONE --"<<std::endl;
  return 0;
}
//--- Functions
float differecnce(float Ax,float Ay,float Bx,float By){return 1;}  

void  Quad::InitMotors(){
  std::cout<<"INFO: Initialising Motors"<<std::endl;
  system("echo 0=0 > /dev/servoblaster"); system("echo 0=1500 > /dev/servoblaster"); // Motor N
  system("echo 4=0 > /dev/servoblaster"); system("echo 4=1500 > /dev/servoblaster"); // Motor E
  system("echo 5=0 > /dev/servoblaster"); system("echo 5=1500 > /dev/servoblaster"); // Motot S
  system("echo 7=0 > /dev/servoblaster"); system("echo 7=1500 > /dev/servoblaster"); // Motor W 
  usleep(1000000); // wait at leat one second
  std::cout<<"-----------------"<<std::endl;
}

void  Quad::StopMotors(){
  std::cout<<"INFO: Stopping Motors"<<std::endl;
  system("echo 0=0 > /dev/servoblaster"); 
  system("echo 4=0 > /dev/servoblaster"); 
  system("echo 5=0 > /dev/servoblaster"); 
  system("echo 7=0 > /dev/servoblaster"); 
  usleep(200000);
  std::cout<<"-----------------"<<std::endl;
}

void  Quad::Throttle(uint motorID,int throttle){
  if(throttle>limT){ std::cout<<"ERROR: max throttle excceed"<<std::endl;  return;}
  std::cout<<"INFO: Motor"<<motorID<<"->"<<throttle<<std::endl;
  if(     motorID==1){ motorID=0; N_t=throttle; } // N 1
  else if(motorID==2){ motorID=4; E_t=throttle; } // E 2
  else if(motorID==3){ motorID=7; S_t=throttle; } // S 3
  else if(motorID==4){ motorID=5; W_t=throttle; } // W 4  
  pFile = fopen ("/dev/servoblaster","w");
  fprintf(pFile,"%d=%d\n",motorID,throttle);
  fclose(pFile);
}

void  Quad::ThrottleAllplusplus(){
  Quad::Throttle(1,N_t+1); 
  Quad::Throttle(2,E_t+1); 
  Quad::Throttle(3,S_t+1); 
  Quad::Throttle(4,W_t+1); 
  std::cout<<"-----------------"<<std::endl;
}

void  Quad::ThrottleAll(int throttle){
  mean_t=throttle;
  for(uint i=1;i<5;i++)  Quad::Throttle(i,throttle); 
  std::cout<<"-----------------"<<std::endl;
}

void  Quad::Stabilise(float timeS){

  double Delta_pitch=0, Delta_roll=0; 
  
  for(float st=0; st<timeS; st+=0.1){
    
    std::cout<<"INFO: Stabilising Quad"<<std::endl;
    uint readfail=0;
    
    while(1)
      if(loop_mpu(&yaw,&pitch,&roll)){
	
	Delta_pitch = pitch-pitch0;  Delta_roll  = roll-roll0;
	
	if(fabs(Delta_pitch) < tolerance && fabs(Delta_roll ) < tolerance) break;	
	else{	  
	  std::cout<<"INFO: Roll and Pitch:"<<Delta_roll<<" \t "<<Delta_pitch<<std::endl; 
	  
	  if(Delta_pitch >=  tolerance){Quad::Throttle(1,N_t<limT-1?N_t+1:N_t); Quad::Throttle(3,S_t>minT?S_t-1:S_t); }
	  if(Delta_pitch <= -tolerance){Quad::Throttle(3,S_t<limT-1?S_t+1:S_t); Quad::Throttle(1,N_t>minT?N_t-1:N_t); }
	  if(Delta_roll  <= -tolerance){Quad::Throttle(2,E_t<limT-1?E_t+1:E_t); Quad::Throttle(4,W_t>minT?W_t-1:W_t); }
	  if(Delta_roll  >=  tolerance){Quad::Throttle(4,W_t<limT-1?W_t+1:W_t); Quad::Throttle(2,E_t>minT?E_t-1:E_t); }
	  
	  //usleep(20000);
	  
	  /*
	  if(Delta_pitch >=  tolerance){Quad::Throttle(1,N_t-3); Quad::Throttle(3,S_t+3); }
	  if(Delta_pitch <= -tolerance){Quad::Throttle(3,S_t-3); Quad::Throttle(1,N_t+3); }
	  if(Delta_roll  <= -tolerance){Quad::Throttle(2,E_t-3); Quad::Throttle(4,W_t+3); }
	  if(Delta_roll  >=  tolerance){Quad::Throttle(4,W_t-3); Quad::Throttle(2,E_t+3); }	  
	  */
	}
      }else{
	readfail++;
	if(readfail>10){ std::cout<<"ERROR: Can't read mpu, failed to stabilise!"<<std::endl;  break; } 
      }
  }
  std::cout<<"-----------------"<<std::endl;  
}
////////////////////////////////////////////////////////////////////////////

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
