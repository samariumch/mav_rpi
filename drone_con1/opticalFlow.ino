void read_px4_flow_data() {                                            //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x42);
  Wire.write(0x0);
  Wire.endTransmission();
  Wire.requestFrom(0x42, 5);
  
  while (Wire.available() < 5);
  pixel_flow_x_sum  = Wire.read() + (uint16_t) ((Wire.read() << 8));
  pixel_flow_y_sum  = Wire.read() + (uint16_t) ((Wire.read() << 8));
  qual              = Wire.read();
 
}

void calculate_pos_vel() {
    Gyro_Y_delay = Gyro_Y_temp;
    Gyro_Y_temp = Gyro_Y;

    Gyro_X_delay = Gyro_X_temp;
    Gyro_X_temp = Gyro_X;
    
    flow_rate_x = 0.06*(pixel_flow_x_sum-pixel_flow_x_sum_cal);
    flow_rate_y = 0.06*(pixel_flow_y_sum-pixel_flow_y_sum_cal);
    
    vel_x = pos_z*(flow_rate_x+Gyro_Y_delay*0.000266);
    vel_y = pos_z*(flow_rate_y-Gyro_X_delay*0.000266);
    
    vel_x = int(1000 * vel_x) / 1000.0;
    vel_y = int(1000 * vel_y) / 1000.0;



}
