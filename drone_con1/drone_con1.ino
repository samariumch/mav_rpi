#include <Wire.h>
//#include "PX4Flow.h"
//#include "VL53L1X.h"

#define DEBUG_MODE_IDE 1
#define DEBUG_MODE_MATLAB 0

//VL53L1X Distance_Sensor;

int incomingByte = 0;

float pid_p_gain_roll = 1;               //Gain setting for the pitch and roll P-controller (default = 0.7).
float pid_i_gain_roll = 0.04;              //Gain setting for the pitch and roll I-controller (default = 0.03).
float pid_d_gain_roll = 0.4;              //Gain setting for the pitch and roll D-controller (default = 0.4).
int pid_max_roll = 30;                    //Maximum output of the PID-controller (+/-).

float pid_p_gain_pitch = 1.2;               //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = 0.04;            //Gain setting for the pitch I-controller.(0.055)
float pid_d_gain_pitch = 0.7;              //Gain setting for the pitch D-controller.
int pid_max_pitch = 40;          //Maximum output of the PID-controller (+/-).

float pid_p_gain_yaw = 2;                //Gain setting for the pitch P-controller (default = 4.0).
float pid_i_gain_yaw = 0.01;               //Gain setting for the pitch I-controller (default = 0.02).
float pid_d_gain_yaw = 0;                //Gain setting for the pitch D-controller (default = 0.0).
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-).



//During flight the battery voltage drops and the motors are spinning at a lower RPM. This has a negative effecct on the
//altitude hold function. With the battery_compensation variable it's possible to compensate for the battery voltage drop.
//Increase this value when the quadcopter drops due to a lower battery voltage during a non altitude hold flight.
float battery_compensation = 40.0;         

float pid_p_gain_altitude = 1.4;           //Gain setting for the altitude P-controller (default = 1.4).
float pid_i_gain_altitude = 0.2;           //Gain setting for the altitude I-controller (default = 0.2).
float pid_d_gain_altitude = 0.75;          //Gain setting for the altitude D-controller (default = 0.75).
int pid_max_altitude = 400;                //Maximum output of the PID-controller (+/-).

//Second order command filter parameters

float Prev_Delta_Pitch = 0;
float PPrev_Delta_Pitch = 0;
float Delta_Pitch = 0;
float Prev_Angle_Pitch_c = 0;
float PPrev_Angle_Pitch_c = 0;
float Prev_Rate_Pitch_c = 0;
float PPrev_Rate_Pitch_c = 0;
float Rate_Pitch_c = 0;
float Angle_Pitch_c = 0;

float Prev_Delta_Roll = 0;
float PPrev_Delta_Roll = 0;
float Delta_Roll = 0;
float Prev_Angle_Roll_c = 0;
float PPrev_Angle_Roll_c = 0;
float Prev_Rate_Roll_c = 0;
float PPrev_Rate_Roll_c = 0;
float Rate_Roll_c = 0;
float Angle_Roll_c = 0;


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
const float a1 = 0.000039;
const float a2 = 0.000153;
const float a3 = 0.000037;

const float b1 = 1;
const float b2 = -1.905;
const float b3 = 0.9085;

const float a4 = 0.02325;
const float a5 = -0.0007324;
const float a6 = -0.02252;
const float ka=0,kv=0,Ts=0;

int32_t channel_1_start, channel_1, pid_roll_setpoint_base;
int32_t channel_2_start, channel_2, pid_pitch_setpoint_base;
int32_t channel_3_start, channel_3;
int32_t channel_4_start, channel_4;
int32_t channel_5_start, channel_5;
int32_t channel_6_start, channel_6;
int32_t measured_time, measured_time_start;
int32_t acc_total_vector, acc_total_vector_at_start;
int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
int16_t acc_pitch_cal_value;
int16_t acc_roll_cal_value;

int32_t acc_z_average_short_total, acc_z_average_long_total, acc_z_average_total ;
int16_t acc_z_average_short[26], acc_z_average_long[51];

uint8_t acc_z_average_short_rotating_mem_location, acc_z_average_long_rotating_mem_location;

int32_t acc_alt_integrated;

uint32_t error_timer, flight_mode_timer;

float roll_level_adjust, pitch_level_adjust;
float pid_error_temp;
float pid_roll_angle_error,pid_roll_rate_error,pid_roll_angle_i_error,pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error,pid_roll_cmd;
float pid_pitch_angle_error,pid_pitch_rate_error,pid_pitch_angle_i_error,pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error, pid_pitch_cmd;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll, angle_yaw;
float battery_voltage, dummy_float; 
float throttle;       //new


int16_t Gyro_X, Gyro_Y, Gyro_Z;
float Gyro_Y_temp;
float Gyro_Y_delay;
float Gyro_X_temp;
float Gyro_X_delay;

float D_Pitch,D_Roll;
int16_t Acc_X, Acc_Y, Acc_Z, Acc_Total;
float odo_x = 0, odo_y = 0, odo_z = 0;
float offset_x, offset_y, offset_z;
int16_t temperature;
int32_t Gyro_X_cal, Gyro_Y_cal, Gyro_Z_cal;
int16_t Acc_X_cal, Acc_Y_cal, Acc_Z_cal;
int16_t cal_int_g, cal_int_a, cal_int_op;
uint16_t frame_count;
int16_t pixel_flow_x_sum, pixel_flow_y_sum, pixel_flow_x_sum_cal, pixel_flow_y_sum_cal;
int16_t flow_comp_m_x, flow_comp_m_y, qual, gyro_x_rate, gyro_y_rate, gyro_z_rate, ground_distance;
int8_t gyro_range, sonar_timestamp;
uint8_t x, y;
uint32_t loop_timer;
uint32_t timer_gyro;
uint8_t counter;
uint8_t counter_start;
int16_t sum_x;
float mean_x = 0;
int16_t sum_y;
float mean_y = 0;


float Angle_Roll_acc, Angle_Pitch_acc, Angle_Roll_gyro, Angle_Pitch_gyro, Angle_Yaw_gyro;
float Angle_Pitch, Angle_Roll, Angle_Yaw;
float Rate_Pitch, Rate_Roll, Rate_Yaw;                            //new
float Rate_Pitch_input, Rate_Roll_input, Rate_Yaw_input;          //new
float Angle_Roll_acc_cal, Angle_Pitch_acc_cal;
float flow_rate_x, flow_rate_y;
float vel_x, vel_y, vel_x_o, vel_y_o, pos_x, pos_y;
float pre_storage_flow_x[5];
float pre_storage_flow_y[5];
float temp1;
float temp2;

int16_t compass_x, compass_y, compass_z;
int16_t compass_cal_values[6];
float compass_x_horizontal, compass_x_horizontal_virtual, compass_y_horizontal, compass_y_horizontal_virtual, actual_compass_heading , actual_compass_heading_pre;
float compass_scale_y, compass_scale_z;
int16_t compass_offset_x, compass_offset_y, compass_offset_z;
int compass_flag = 0;
float actual_compass_heading_offset = 0;
float diff;
int virtual_north;


uint16_t pre_height;
uint32_t timer_tof;
float pre_storage[10];
float FIR[7] = {0.0172, 0.0971, 0.2328, 0.3058, 0.2328, 0.0971, 0.0172};
float vel_z;

float pos_z, pos_z_o;

bool print_flag = 0;
float timer_counter;
float timer_pre;
float z_cal;
int16_t esc_1, esc_2, esc_3, esc_4;


float base_vel_x, base_vel_y, base_vel_z;  //delcare inertial frame translation variables
float base_pos_x = 0, base_pos_y = 0, base_pos_z = 0; //declare body frame translation variables
float phi_r, theta_r, psi_r;

uint8_t channel_select_counter;

int16_t motor_idle_speed = 1;           //Enter the minimum throttle pulse of the motors when they idle (between 1000 and 1200) //new
//



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(57600);
  Wire.begin();
  Wire.setClock(400000);
  pinMode(PB5, OUTPUT);
  delay(250);
  
//  pinMode(4, INPUT_ANALOG);
  setup_compass();
  setup_mpu_6050_registers();
  sensor_calibration();

  read_compass();
  Angle_Yaw = 0;
  base_pos_x = 0;
  base_pos_y = 0;
  virtual_north = actual_compass_heading;
  read_compass();
  Angle_Yaw = actual_compass_heading;

  actual_compass_heading_pre = actual_compass_heading;
//
//  Distance_Sensor.setTimeout(500);
//  Distance_Sensor.init();
//  Distance_Sensor.setDistanceMode(VL53L1X::Long);
//  Distance_Sensor.setMeasurementTimingBudget(33000);
//  Distance_Sensor.startContinuous(33);
//  while (!Distance_Sensor.dataReady()) {}
//  Distance_Sensor.read();
//  pre_height = Distance_Sensor.ranging_data.range_mm;
//  pos_z = Distance_Sensor.ranging_data.range_mm;
//  z_cal = Distance_Sensor.ranging_data.range_mm;
  loop_timer = micros();
  timer_setup();
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

//  GPIOB_BASE->BSRR = 0b1 << 5;
  read_mpu_6050_data();

  //Gyro angle calculations

  calculate_euler_acc();                                               //Calculating Euler Angles from accelerometer readings
  calculate_euler_gyro();                                              //Calculating Euler Angles (Z(Yaw)-Y(Pitch)-X(Roll)) from gyro readings.

  read_compass();

  com_filter();

  //read_px4_flow_data();

//  read_tof();

  //calculate_pos_vel();
  //transformation();

// Prepare for PID //new
  pitch_level_adjust = Angle_Pitch * 15;                                           //Calculate the pitch angle correction. Limits the commanded angle to +/- 500/15 = 33.3
  roll_level_adjust = Angle_Roll * 15;                                             //Calculate the roll angle correction. Limits the commanded angle to +/- 500/15 = 33.3
  
  calculate_pid();

  if (channel_5>1500){
    throttle = channel_3;  
    if (throttle < 1050) throttle =1050; 
  }
// Set ESC Pulse //new
  if (throttle > 1850) throttle = 1850;                                          //We need some room to keep full control at full throttle.
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 1 (front-right - CCW).
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 2 (rear-right - CW).
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 3 (rear-left - CCW).
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 4 (front-left - CW).

 if (esc_1 < motor_idle_speed) esc_1 = motor_idle_speed;                        //Keep the motors running.
 if (esc_2 < motor_idle_speed) esc_2 = motor_idle_speed;                        //Keep the motors running.
 if (esc_3 < motor_idle_speed) esc_3 = motor_idle_speed;                        //Keep the motors running.
 if (esc_4 < motor_idle_speed) esc_4 = motor_idle_speed;                        //Keep the motors running.

  if (esc_1 > 2000)esc_1 = 2000;                                                 //Limit the esc-1 pulse to 2000us.
  if (esc_2 > 2000)esc_2 = 2000;                                                 //Limit the esc-2 pulse to 2000us.
  if (esc_3 > 2000)esc_3 = 2000;                                                 //Limit the esc-3 pulse to 2000us.
  if (esc_4 > 2000)esc_4 = 2000;                                                 //Limit the esc-4 pulse to 2000us.

 if (channel_5 < 1500)esc_1 = 1000;
 if (channel_5 < 1500)esc_2 = 1000;
 if (channel_5 < 1500)esc_3 = 1000;
 if (channel_5 < 1500)esc_4 = 1000;
  
//  GPIOB_BASE->BSRR = 0b1 << 21;
  debug_mode();
  TIMER2_BASE->CCR1 = esc_2;                                            //Set the throttle receiver input pulse to the ESC 2 output pulse.
  TIMER2_BASE->CCR2 = esc_3;                                            //Set the throttle receiver input pulse to the ESC 1 output pulse.
  TIMER2_BASE->CCR3 = esc_1;                                            //Set the throttle receiver input pulse to the ESC 3 output pulse.
  TIMER2_BASE->CCR4 = esc_4;                                            //Set the throttle receiver input pulse to the ESC 4 output pulse.
  while (micros() - loop_timer < 5000);
  loop_timer = micros();  //Set the timer for the next loop.
}
