#include "code.h"

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
  
  command.str(""); command<<"echo "<<motorID<<"="<<throttle<<" > /dev/servoblaster"<<endl;
  const char *char_command = command.str().c_str();
  system(char_command);
  return;
}


void  ThrottleAll(int throttle){
  Throttle(N,throttle); 
  Throttle(E,throttle); 
  Throttle(S,throttle); 
  Throttle(W,throttle); 
  return; 
}

void  InitMotors(){
  cout<<"INFO: Init Motors"<<endl;
  ramping=true;
  ThrottleAll(0);
  usleep(1000000);
  ThrottleAll(setmin_spin_T);
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
  return;
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
  return;
}



/////////////////////////

bool setup_pwm(){
  if(true){
    std::cout<<std::endl<<std::endl<<"INFO: Invoking ServoBlaster"<<std::endl;
    system("/root/code/servoblaster/servod --cycle-time=2000us --step-size=2us --p1pins=\"7,15,16,22\" ");
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
        mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
        cout<<" PID= "<<Kp<<"*"<<pitch_error<<"+"<<Ki<<"*"<<pitch_integral<<"+"<<Kd<<"*"<<pitch_derivative<<endl;
        cout<<" intPID="<<int(Kp*pitch_error)<<"+"<<int(Ki*pitch_integral)<<"+"<<int(Kd*pitch_derivative)<<endl;
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



