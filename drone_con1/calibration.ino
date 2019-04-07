void sensor_calibration(void){
  for ( cal_int_g = 0; cal_int_g < 2000 ; cal_int_g ++) {              //Run this code 2000 times
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    Gyro_X_cal += Gyro_X;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    Gyro_Y_cal += Gyro_Y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    Gyro_Z_cal += Gyro_Z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }

  Gyro_X_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  Gyro_Y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  Gyro_Z_cal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset
//
//  for ( cal_int_a = 0; cal_int_a < 64; cal_int_a ++) {
//    read_mpu_6050_data();
//    Acc_X_cal   += Acc_X;
//    Acc_Y_cal   += Acc_Y;
//    delayMicroseconds(3700);
//  }
//
//  Acc_X_cal /= 64;
//  Acc_Y_cal /= 64;
//  D_Roll=3.4;
//  D_Pitch=-1.10;

  for ( cal_int_op = 0; cal_int_op < 1000; cal_int_op ++) {
//    read_px4_flow_data();
    pixel_flow_x_sum_cal   += pixel_flow_x_sum;
    pixel_flow_y_sum_cal   += pixel_flow_y_sum;
    delayMicroseconds(5000);
  }

  pixel_flow_x_sum_cal /= 1000;
  pixel_flow_y_sum_cal /= 1000;
//
  pos_x = 0;
  pos_y = 0;
  counter_start = 0;
}
