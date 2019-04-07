void setup_mpu_6050_registers() {
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the acc low pass filter
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1A);                                                    //Send the requested starting register
  Wire.write(0x04);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}

void read_mpu_6050_data() {  //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68, 14);                                          //Request 14 bytes from the MPU-6050
  while (Wire.available() < 14);                                       //Wait until all the bytes are received
  Acc_X = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_x variable
  Acc_Y = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_y variable
  Acc_Z = Wire.read() << 8 | Wire.read();                              //Add the low and high byte to the acc_z variable
  Acc_X *=  -1;
  temperature = Wire.read() << 8 | Wire.read();                        //Add the low and high byte to the temperature variable
  Gyro_X = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_x variable
  Gyro_Y = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_y variable
  Gyro_Z = Wire.read() << 8 | Wire.read();                             //Add the low and high byte to the gyro_z variable
  Gyro_Y *= -1;                                                        //Invert the direction of the axis.
  Gyro_Z *= -1;

  if (cal_int_g >= 2000) {
    Gyro_X -= Gyro_X_cal;                                              //Subtact the manual gyro roll calibration value.
    Gyro_Y -= Gyro_Y_cal;                                              //Subtact the manual gyro pitch calibration value.
    Gyro_Z -= Gyro_Z_cal;                                              //Subtact the manual gyro yaw calibration value.
  }

//    Acc_X -= 93;                                                //Subtact the manual acc X calibration value.
//    Acc_Y -= -63;                                                //Subtact the manual acc Y calibration value.
//    Acc_Z -= -144; 
    
    Acc_X_cal=int16_t(Acc_X+D_Pitch*Acc_Z);
    Acc_Y_cal=int16_t(Acc_Y-D_Roll*Acc_Z);
    Acc_Z_cal=int16_t(-D_Pitch*Acc_X+D_Roll*Acc_Y+Acc_Z);

}

void com_filter(void)
{
  Angle_Pitch      = Angle_Pitch * 0.99 + Angle_Pitch_acc * 0.01;         //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
  Angle_Roll       = Angle_Roll  * 0.99 + Angle_Roll_acc * 0.01;          //Correct the drift of the gyro roll angle with the accelerometer roll angle
  
  while (Angle_Yaw>180){Angle_Yaw-= 360;}                                 //If gyro yaw angle is above 180 deg, 360 is subtracted until yaw is betwen [-180,180]
  while (Angle_Yaw<-180){Angle_Yaw+=360;}                                 //If gyro yaw angle is below 180 deg, 360 is added until yaw is betwen [-180,180]
  
  Angle_Yaw       -= course_deviation(Angle_Yaw,actual_compass_heading)/100.0;  //Make a small correction based on difference in heading between compass and gyro
  //Angle_Yaw        = Angle_Yaw   * 0.9 + actual_compass_heading * 0.1;

  while (Angle_Yaw>180){Angle_Yaw-= 360;}                                 //If gyro yaw angle is above 180 deg, 360 is subtracted until yaw is betwen [-180,180]
  while (Angle_Yaw<-180){Angle_Yaw+=360;}                                 //If gyro yaw angle is below 180 deg, 360 is added until yaw is betwen [-180,180]

}

void calculate_euler_acc() {

  // The following part calculates the Roll and Pitch angles from the gravity components in the body axes.
  Acc_Total       = sqrt((Acc_X_cal * Acc_X_cal) + (Acc_Y_cal * Acc_Y_cal) + (Acc_Z_cal * Acc_Z_cal));
  Angle_Pitch_acc = -asin((float)Acc_X_cal / Acc_Total) * 57.296;
  Angle_Roll_acc  = atan2((float)Acc_Y_cal, (float)Acc_Z_cal) * 57.296;

}

void calculate_euler_gyro() {

  // The following part calculates the Euler Angles by propagating the rotational kinematics according to Z(Yaw)-Y(Pitch)-X(Roll) sequence.
  Angle_Roll  += ((float)Gyro_X + (float)Gyro_Y * sin(Angle_Roll * 0.0174533) * tan(Angle_Pitch * 0.0174533) + (float)Gyro_Z * cos(Angle_Roll * 0.0174533) * tan(Angle_Pitch * 0.0174533)) * 0.0000763;
  Angle_Pitch += ((float)Gyro_Y * cos(Angle_Roll * 0.0174533) - (float)Gyro_Z * sin(Angle_Roll * 0.0174533)) * 0.0000763;
  Angle_Yaw   += ((float)Gyro_Y * sin(Angle_Roll * 0.0174533) / cos(Angle_Pitch * 0.0174533) + (float)Gyro_Z * cos(Angle_Roll * 0.0174533) / cos(Angle_Pitch * 00.0174533)) * 0.0000763;
  
  Rate_Roll  = ((float)Gyro_X + (float)Gyro_Y * sin(Angle_Roll * 0.0174533) * tan(Angle_Pitch * 0.0174533) + (float)Gyro_Z * cos(Angle_Roll * 0.0174533) * tan(Angle_Pitch * 0.0174533));
  Rate_Pitch = ((float)Gyro_Y * cos(Angle_Roll * 0.0174533) - (float)Gyro_Z * sin(Angle_Roll * 0.0174533));
  Rate_Yaw   = ((float)Gyro_Y * sin(Angle_Roll * 0.0174533) / cos(Angle_Pitch * 0.0174533) + (float)Gyro_Z * cos(Angle_Roll * 0.0174533) / cos(Angle_Pitch * 00.0174533));
  
  //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
  Rate_Roll/=65.5;
  Rate_Pitch/=65.5;
  Rate_Yaw/=65.5;
  
  //Low Pass Filter on Angular Rates.
  Rate_Roll_input = (Rate_Roll_input * 0.7) + ((float)Rate_Roll * 0.3);   //Gyro pid input is deg/sec.
  Rate_Pitch_input = (Rate_Pitch_input * 0.7) + ((float)Rate_Pitch * 0.3);//Gyro pid input is deg/sec.
  Rate_Yaw_input = (Rate_Yaw_input * 0.7) + ((float)Rate_Yaw * 0.3);      //Gyro pid input is deg/sec.
  
}
