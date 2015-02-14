#include "code.h"

float GetMeanThrottle(){
  return (N_t+E_t+S_t+W_t)*0.25;
};

float Valid_Throttle(float throttle){
  if(throttle<min_allowed_T){ std::cout<<"WARNING: min throttle excceed min_allowed_T = "<<min_allowed_T<<" asked_T="<<throttle<<std::endl; return min_allowed_T;}
  if(throttle>max_allowed_T){ std::cout<<"WARINIG: max throttle excceed max_allowed_T = "<<max_allowed_T<<" asked_T="<<throttle<<std::endl; return max_allowed_T;}
  return throttle;
}

void  Throttle(int motorID,float throttle){
  if(!ramping) throttle=Valid_Throttle(throttle);
  
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
  
  command.str(""); command<<"echo "<<motorID<<"="<<int(throttle)<<" > /dev/servoblaster"<<endl; //here throttle needs to be integer!
  const char *char_command = command.str().c_str();
  system(char_command);
  return;
}


void  ThrottleAll(float throttle){
  Throttle(N,throttle); 
  Throttle(E,throttle); 
  Throttle(S,throttle); 
  Throttle(W,throttle); 
  usleep(1000000);
  return; 
}

void  InitMotors(){
  cout<<"INFO: Calling InitMotors"<<endl;
  ramping=true;
  ThrottleAll(0);
  usleep(1000000);
  ThrottleAll(750); //needed to start rotaring motors
  usleep(1000000);
  ThrottleAll(min_allowed_T);
  ramping=false;
  usleep(1000000);
  return;
}

void  StopMotors(){
  cout<<"INFO: Stop Motors"<<endl;
  ramping=true;
  ThrottleAll(0);
  ramping=false;
}

/////////////////////////

bool setup_pwm(){
  std::cout<<std::endl<<std::endl<<"INFO: Invoking ServoBlaster"<<std::endl;
  std::cout<<"INFO: --------- "<<std::endl;
  system("/root/code/servoblaster/servod --cycle-time=1700us --step-size=2us --p1pins=\"7,15,16,22\" --max=820");
  std::cout<<"INFO: --------- "<<std::endl;
  return true;
};

bool setup_orientaion(){
  system("echo i2cdetect -y 1; i2cdetect -y 1");         // scanning address for i2c protocol
  setup_mpu();                                           // giro & accelerometer setup
  usleep(250000);                                        // sleep 0.25s
  loop_mpu(&yaw0,&pitch0,&roll0);  usleep(500000);       // starting values (twice) must be in a horizontal position 
  if(loop_mpu(&yaw0,&pitch0,&roll0)){
    std::cout<<std::endl<<std::endl<<"INFO: Reading initial angles::: yaw0:"<<yaw0<<" pitch0:"<<pitch0<<" roll0:"<<roll0<<std::endl;
  }
  else{
    std::cout<<"FATAL: mpu: failed to retrieve angles, can't continue!"<<std::endl; return 0;
  }
  return true;
};


void setup_mpu() {  
  // initialize device  
  printf("INFO: Initializing I2C devices...\n");
  mpu.initialize();
  // verify connection
  printf("INFO: Testing device connections...\n");
  printf(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");
  // load and configure the DMP
  printf("INFO: Initializing DMP...\n");
  devStatus = mpu.dmpInitialize();
  // make sure it worked (returns 0 if so) 
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    printf("INFO: Enabling DMP...\n");
    mpu.setDMPEnabled(true);
    // enable Arduino interrupt detection
    //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    //attachInterrupt(0, dmpDataReady, RISING); 
    mpuIntStatus = mpu.getIntStatus();
    // set our DMP Ready flag so the main loop() function knows it's okay to use it  
    printf("INFO: DMP ready!\n");
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR! 
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    printf("INFO: DMP Initialization failed (code %d)\n", devStatus);
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
      printf("INFO: quat %7.2f %7.2f %7.2f %7.2f    ", q.w,q.x,q.y,q.z);
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
        mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
        logfile<<"to_plot: "<<*roll<<" "<<*pitch<<" "<<*yaw<<" "<<N_t<<" "<<S_t<<" "<<E_t<<" "<<W_t<<" "<<mtime<<" "
               <<" "<<int(Kp*pitch_error)<<" "<<int(Ki*pitch_integral)<<" "<<int(Kd*pitch_derivative)<<endl;
      }
      return 1;
      #endif
      printf("\n");
      badFIFO=false;
    }
  }
  return 0;
}



