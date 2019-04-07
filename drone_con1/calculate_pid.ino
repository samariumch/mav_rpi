///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//The PID controllers are explained in part 5 of the YMFC-3D video session:
//https://youtu.be/JBvnB0279-Q
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_pid(void) {


  // Think of this as a PDD control on the angle, rather than a PID on the rate. 

  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of dividing by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  Delta_Roll = 0;
  //We need a little dead band of 16us for better results.
  if (channel_4 > 1508)Delta_Roll = channel_4 - 1508;
  else if (channel_4 < 1492)Delta_Roll = channel_4 - 1492;
  command_filter_2nd_Roll();

  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of dividing by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  Delta_Pitch = 0;
  //We need a little dead band of 16us for better results.
  if (channel_2 > 1508)Delta_Pitch = channel_2 - 1508;
  else if (channel_2 < 1492)Delta_Pitch = channel_2 - 1492;
  Delta_Pitch *= -1;
  command_filter_2nd_Pitch();

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of dividing by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  
  //We need a little dead band of 16us for better results.
  if (channel_3 > 1050) { //Do not yaw when turning off the motors.
    if (channel_1 > 1508)pid_yaw_setpoint = (channel_1 - 1508) / 3.0;
    else if (channel_1 < 1492)pid_yaw_setpoint = (channel_1 - 1492) / 3.0;
  }

  //Roll calculations
  pid_roll_rate_error    = Rate_Roll_input - Rate_Roll_c; 
  pid_roll_angle_error   = Angle_Roll - Angle_Roll_c;
  pid_roll_angle_i_error += pid_roll_angle_error*0.005;
  if(channel_5<1500)   pid_roll_angle_i_error=0;  
  
  if (pid_i_gain_roll*pid_roll_angle_i_error > pid_max_roll)pid_roll_angle_i_error = pid_max_roll/pid_i_gain_roll;
  else if (pid_i_gain_roll*pid_roll_angle_i_error < pid_max_roll * -1)pid_roll_angle_i_error = pid_max_roll * -1/pid_i_gain_roll;

  pid_output_roll = pid_p_gain_roll * pid_roll_angle_error + pid_i_gain_roll*pid_roll_angle_i_error  + pid_d_gain_roll * pid_roll_rate_error;
  if (pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if (pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;


  //Pitch calculations
  pid_pitch_rate_error    = Rate_Pitch_input - Rate_Pitch_c; 
  pid_pitch_angle_error   = Angle_Pitch - Angle_Pitch_c;
  pid_pitch_angle_i_error += pid_pitch_angle_error*0.005;
  if(channel_5<1500)   pid_pitch_angle_i_error=0;   
  
  if (pid_i_gain_pitch*pid_pitch_angle_i_error  > pid_max_pitch)pid_pitch_angle_i_error = pid_max_pitch/pid_i_gain_pitch;
  else if (pid_i_gain_pitch*pid_pitch_angle_i_error < pid_max_pitch * -1)pid_pitch_angle_i_error = pid_max_pitch * -1/pid_i_gain_pitch;
  

  pid_output_pitch = pid_p_gain_pitch * pid_pitch_angle_error + pid_i_gain_pitch*pid_pitch_angle_i_error + pid_d_gain_pitch * pid_pitch_rate_error;
  if (pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if (pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  //Yaw calculations
  pid_error_temp = Rate_Yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if (pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;
  if(channel_5<1500) pid_i_mem_yaw=0;  

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if (pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
  
}
void command_filter_1st_Pitch(void)
{
  Angle_Pitch_c = Prev_Angle_Pitch_c*(kv)/(kv+Ts*ka)+Delta_Pitch/(kv/Ts+ka);
  Rate_Pitch_c = (Delta_Pitch-Prev_Delta_Pitch+kv*Prev_Rate_Pitch_c)/(kv+ka*Ts);
  
  Prev_Angle_Pitch_c = Angle_Pitch_c;
  Prev_Rate_Pitch_c = Rate_Pitch_c;
  Prev_Delta_Pitch = Delta_Pitch;
}
void command_filter_1st_Roll(void)
{
  Angle_Roll_c = Prev_Angle_Roll_c*(kv)/(kv+Ts*ka)+Delta_Roll/(kv/Ts+ka);
  Rate_Roll_c = (Delta_Roll-Prev_Delta_Roll+kv*Prev_Rate_Roll_c)/(kv+ka*Ts);
  
  Prev_Angle_Roll_c = Angle_Roll_c;
  Prev_Rate_Roll_c = Rate_Roll_c;
  Prev_Delta_Roll = Delta_Roll;
}

/*
second order command filter parameters

theta =
 
          a1*z^2 + a2*z + a3
  ----------------------------------- 
          b1*z^2 + b2*z + b3
          
theta_dot =
 
          a4*z^2 + a5*z + a6
  ---------------------------------------
          b1*z^2 + b2*z + b3
        
Sample time: 0.005 seconds
Discrete-time transfer function.

*/
void command_filter_2nd_Roll(void)
{
  Angle_Roll_c = (float)((a1*Delta_Roll+a2*Prev_Delta_Roll+a3*PPrev_Delta_Roll-b2*Prev_Angle_Roll_c-b3*PPrev_Angle_Roll_c)/b1);
  Rate_Roll_c = (float)((a4*Delta_Roll+a5*Prev_Delta_Roll+a6*PPrev_Delta_Roll-b2*Prev_Rate_Roll_c-b3*PPrev_Rate_Roll_c)/b1);

  PPrev_Angle_Roll_c = Prev_Angle_Roll_c;
  PPrev_Rate_Roll_c = Prev_Rate_Roll_c;
  PPrev_Delta_Roll = Prev_Delta_Roll;
  
  Prev_Angle_Roll_c = Angle_Roll_c;
  Prev_Rate_Roll_c = Rate_Roll_c;
  Prev_Delta_Roll = Delta_Roll;
}

void command_filter_2nd_Pitch(void)
{
  Angle_Pitch_c = (a1*Delta_Pitch+a2*Prev_Delta_Pitch+a3*PPrev_Delta_Pitch-b2*Prev_Angle_Pitch_c-b3*PPrev_Angle_Pitch_c)/b1;
  Rate_Pitch_c = (a4*Delta_Pitch+a5*Prev_Delta_Pitch+a6*PPrev_Delta_Pitch-b2*Prev_Rate_Pitch_c-b3*PPrev_Rate_Pitch_c)/b1;

  PPrev_Angle_Pitch_c = Prev_Angle_Pitch_c;
  PPrev_Rate_Pitch_c = Prev_Rate_Pitch_c;
  PPrev_Delta_Pitch = Prev_Delta_Pitch;
  
  Prev_Angle_Pitch_c = Angle_Pitch_c;
  Prev_Rate_Pitch_c = Rate_Pitch_c;
  Prev_Delta_Pitch = Delta_Pitch;
}
