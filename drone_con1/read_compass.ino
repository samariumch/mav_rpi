void setup_compass() {
  Wire.beginTransmission(0x0D);                     //Start communication with the compass.
  Wire.write(0x0B);                                            //We want to write to the Configuration Register A (00 hex).
  Wire.write(0x01);
  Wire.endTransmission();
  Wire.beginTransmission(0x0D);
  Wire.write(0x09);                                            //We want to write to the Configuration Register A (00 hex).
  Wire.write(0x0D);                                            //Set the Configuration Regiser A bits as 01111000 to set sample rate (average of 8 at 75Hz).
  Wire.endTransmission();                                      //End the transmission with the compass.

  compass_cal_values[0] = -3150;
  compass_cal_values[1] = 7315;
  compass_cal_values[2] = -6995;
  compass_cal_values[3] = 3345;
  compass_cal_values[4] = -4742;
  compass_cal_values[5] = 5832;


  compass_scale_y = ((float)compass_cal_values[1] - compass_cal_values[0]) / (compass_cal_values[3] - compass_cal_values[2]);
  compass_scale_z = ((float)compass_cal_values[1] - compass_cal_values[0]) / (compass_cal_values[5] - compass_cal_values[4]);

  compass_offset_x = (compass_cal_values[1] - compass_cal_values[0]) / 2 - compass_cal_values[1];
  compass_offset_y = (((float)compass_cal_values[3] - compass_cal_values[2]) / 2 - compass_cal_values[3]);
  compass_offset_z = (((float)compass_cal_values[5] - compass_cal_values[4]) / 2 - compass_cal_values[5]);
}

void read_compass() {
  Wire.beginTransmission(0x0D);                     //Start communication with the compass.
  Wire.write(0x00);                                            //We want to start reading at the hexadecimal location 0x03.
  Wire.endTransmission();                                      //End the transmission with the gyro.

  Wire.requestFrom(0x0D, 6);                        //Request 6 bytes from the compass.

  compass_y = (int16_t)(Wire.read() | Wire.read() << 8);                //Add the low and high byte to the compass_y variable.                                              //Invert the direction of the axis.
  compass_x = (int16_t)(Wire.read() | Wire.read() << 8);                  //Add the low and high byte to the compass_z variable.;
  compass_z = (int16_t)(Wire.read() | Wire.read() << 8);                //Add the low and high byte to the compass_x variable.;
  compass_z *= -1;                                              //Invert the direction of the axis.
  
  compass_y += compass_offset_y;                              //Add the y-offset to the raw value.
  compass_y *= compass_scale_y;                               //Scale the y-value so it matches the other axis.
  compass_z += compass_offset_z;                              //Add the z-offset to the raw value.
  compass_z *= compass_scale_z;                               //Scale the z-value so it matches the other axis.
  compass_x += compass_offset_x;                              //Add the x-offset to the raw value.

  compass_x_horizontal = (float)compass_x * cos(Angle_Pitch * 0.0174533) + (float)compass_y * sin(Angle_Roll * 0.0174533) * sin(Angle_Pitch * 0.0174533) + (float)compass_z * cos(Angle_Roll * 0.0174533) * sin(Angle_Pitch * 0.0174533);
  compass_y_horizontal = (float)compass_y * cos(Angle_Roll * 0.0174533) - (float)compass_z * sin(Angle_Roll * 0.0174533);

  compass_x_horizontal_virtual = (compass_x_horizontal*cos(virtual_north*0.0174533)-compass_y_horizontal*sin(virtual_north*0.0174533));
  compass_y_horizontal_virtual = (compass_x_horizontal*sin(virtual_north*0.0174533)+compass_y_horizontal*cos(virtual_north*0.0174533));
  
  actual_compass_heading = -(atan2(compass_y_horizontal_virtual, compass_x_horizontal_virtual)) * (180 / 3.14);
}


//The following subrouting calculates the smallest difference between two heading values.
float course_deviation(float course_b, float course_c) {
  float course_a;
  float base_course_mirrored;
  float actual_course_mirrored;
  course_a = course_b - course_c;
  if (course_a < -180 || course_a > 180) 
  {
    if (course_c < 0)base_course_mirrored = course_c + 180;
    else base_course_mirrored = course_c - 180;
    if (course_b < 0)actual_course_mirrored = course_b + 180;
    else actual_course_mirrored = course_b - 180;
    course_a = actual_course_mirrored - base_course_mirrored;
  }
  return course_a;
}

