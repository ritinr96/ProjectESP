/*
 * 
 */

//----------DEBUG----------//
#define DEBUG_MAIN
//#define DEBUG_IMU_GYRO
//#define DEBUG_IMU_YPR
//#define DEBUG_IMU_OFFSETS
//#define DEBUG_WIFI_INPUT
//#define DEBUG_MOTOR_PWM
#define LOG_YPR_AND_PWM
//#define DEBUG_APP_PID

int pidUpdateRateLoopCounter = 0;

void setup()
{

  ESP.eraseConfig();
#ifdef DEBUG_MAIN
  //Open Serial Comms
  Serial.begin(115200);
  while (!Serial)
  {
    yield();
  }
#endif
  yield();
  //Setup WiFi, IMU, PID and Motor outputs
  udp_setup();
  imu_setup();
  pid_setup();
  motor_setup();
}

void loop()
{
  yield();
  //Take App inputs from UDP AP server
  udp_input();

  //Take IMU readings
  imu_ypr();

  //Input into PID controller
  pid_stab();

  //Take Outputs from PID controller
  pid_rate();
    
  //Return LOG_MOTOR_YPR
  //udp_output();
  
  //Write into motors!
  motor_compute_outputs();

  
}



