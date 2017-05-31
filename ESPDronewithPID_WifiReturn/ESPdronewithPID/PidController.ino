
//All PID variabless declared in Define file

void pid_setup()
{
  roll_rate_controller.SetOutputLimits(ROLL_PID_MIN, ROLL_PID_MAX);
  pitch_rate_controller.SetOutputLimits(PITCH_PID_MIN, PITCH_PID_MAX);
  yaw_rate_controller.SetOutputLimits(YAW_PID_MIN, YAW_PID_MAX);

  roll_rate_controller.SetMode(AUTOMATIC);
  pitch_rate_controller.SetMode(AUTOMATIC);
  yaw_rate_controller.SetMode(AUTOMATIC);

  roll_rate_controller.SetSampleTime(20);
  pitch_rate_controller.SetSampleTime(20);
  yaw_rate_controller.SetSampleTime(20);

  roll_stab_controller.SetOutputLimits(ROLL_PID_MIN, ROLL_PID_MAX);
  pitch_stab_controller.SetOutputLimits(PITCH_PID_MIN, PITCH_PID_MAX);
  yaw_stab_controller.SetOutputLimits(YAW_PID_MIN, YAW_PID_MAX);

  roll_stab_controller.SetMode(AUTOMATIC);
  pitch_stab_controller.SetMode(AUTOMATIC);
  yaw_stab_controller.SetMode(AUTOMATIC);

  roll_stab_controller.SetSampleTime(20);
  pitch_stab_controller.SetSampleTime(20);
  yaw_stab_controller.SetSampleTime(20);
}

void pid_stab()
{
  //Updating actual ypr
  pidStabIn[2] = ypr[2];
  pidStabIn[1] = ypr[1];
  pidStabIn[0] = ypr[0];

  //Updating desired ypr
  pidStabSetpoint[2] = rcInput[2];
  pidStabSetpoint[1] = rcInput[1];
  pidStabSetpoint[0] = yaw_set;

  pid_stab_compute_outputs();  
}

void pid_stab_compute_outputs()
{
  //Outputs are written to pidOut[]  (declared in Define file)
  roll_stab_controller.SetTunings(prStabKp, prStabKi, prStabKd);
  roll_stab_controller.Compute();
  pitch_stab_controller.SetTunings(prStabKp, prStabKi, prStabKd);
  pitch_stab_controller.Compute();
  yaw_stab_controller.SetTunings(yawStabKp, yawStabKi, yawStabKd);
  yaw_stab_controller.Compute();
}

void pid_rate()
{
  //Updating actual ypr
  pidRateIn[2] = g.x;
  pidRateIn[1] = g.y;
  pidRateIn[0] = g.z;

  //Updating desired ypr
  if(rcInput[3]<=1080)
  {
    pidRateSetpoint[2] = 0.0;
    pidRateSetpoint[1] = 0.0;

  }
  else
  {
    pidRateSetpoint[2] = pidStabOut[2];
    pidRateSetpoint[1] = pidStabOut[1];

  if (rcInput[0]>5 || rcInput[0]<-5)
  {
    pidRateSetpoint[0] = rcInput[0];//Rate Mode
    yaw_set = ypr[0];
    yaw_rate_controller.SetTunings(0.0, yawRateKi, yawRateKd);

    pidRateSetpoint[0] = 0.0;
    pidStabSetpoint[2]=0.0;
    pidStabSetpoint[1]=0.0;
    pidStabSetpoint[0]=0.0;
     //Reseting I drift
    
    roll_rate_controller.SetOutputLimits(0.0, 1.0);  // Forces minimum up to 0.0
    roll_rate_controller.SetOutputLimits(-1.0, 0.0);  // Forces maximum down to 0.0
    roll_rate_controller.SetOutputLimits(ROLL_PID_MIN, ROLL_PID_MAX);
    
    pitch_rate_controller.SetOutputLimits(0.0, 1.0);  // Forces minimum up to 0.0
    pitch_rate_controller.SetOutputLimits(-1.0, 0.0);  // Forces maximum down to 0.0
    pitch_rate_controller.SetOutputLimits(PITCH_PID_MIN, PITCH_PID_MAX);
    
    yaw_rate_controller.SetOutputLimits(0.0, 1.0);  // Forces minimum up to 0.0
    yaw_rate_controller.SetOutputLimits(-1.0, 0.0);  // Forces maximum down to 0.0
    yaw_rate_controller.SetOutputLimits(YAW_PID_MIN, YAW_PID_MAX);
    
  
  }
  else
  {
    pidRateSetpoint[0] = pidStabOut[0];//Stabilize Mode
    yaw_rate_controller.SetTunings(yawRateKp, yawRateKi, yawRateKd);
  
  }
  }
  
  
  pid_rate_compute_outputs();  
}

void pid_rate_compute_outputs()
{
  //Outputs are written to pidOut[]  (declared in Define file)
  
  roll_rate_controller.SetTunings(prRateKp, prRateKi, prRateKd);
  roll_rate_controller.Compute();
  pitch_rate_controller.SetTunings(prRateKp, prRateKi, prRateKd);
  pitch_rate_controller.Compute();
  yaw_rate_controller.Compute();
}
