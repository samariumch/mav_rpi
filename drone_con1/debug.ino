void debug_mode(void){
  if(DEBUG_MODE_IDE)
  {
    
    //  GPIOB_BASE->BSRR=0b1<<21;
    //  Serial.print(temp1);
    //  Serial.print(" ");
//      Serial.print(Gyro_X);
//      Serial.print(" ");
//      Serial.print(Gyro_Y);
//      Serial.print(" ");
//      Serial.print(Gyro_Z);

//      Serial.print(course_deviation(Angle_Yaw,actual_compass_heading));
//      Serial.print(" ");
//      Serial.print(actual_compass_heading);
//      Serial.print(" ");
//      Serial.print(Angle_Yaw);
      
    //  Serial.print(" ");
//      Serial.print(actual_compass_heading);
    //  Serial.print(" ");
//      Serial.print(" ");

//      Serial.print(compass_x);
//      Serial.print(" ");
//      Serial.print(compass_y);
//      Serial.print(" ");
//      Serial.print(compass_z);
    //  Serial.print(" ");
//      Serial.print(pos_z);
//      Serial.print(" ");
//      Serial.print(vel_z);
    //  Serial.print(" ");
//      Serial.print(base_pos_x);
//      Serial.print(" ");
//      Serial.print(base_pos_y);
//      Serial.print(" ");
//      Serial.print(-base_pos_z);

//      Serial.print(channel_1);
//      Serial.print(" ");
//      Serial.print(channel_2);
//      Serial.print(" ");
//      Serial.print(channel_3);
//      Serial.print(" ");
//      Serial.print(channel_4);
//      Serial.print(" ");
//      Serial.print(channel_5);
//      Serial.print(" ");
//      Serial.print(channel_6);


//      Serial.print(sqrt(Acc_X^2+Acc_Y^2));
//      Serial.print(pid_output_pitch);
//      Serial.print(" ");
//      Serial.print(pid_output_roll);
//      Serial.print(" ");
//      Serial.print(pid_output_yaw);
//      Serial.print(" ");
//      Serial.print(esc_1);
//      Serial.print(" ");
//      Serial.print(esc_2);
//      Serial.print(" ");
//      Serial.print(esc_3);
//      Serial.print(" ");
//      Serial.print(esc_4);
//      Serial.print(" ");
    //  Serial.print((cos(Angle_Roll*0.0174533)*cos(Angle_Pitch*0.0174533))*Distance_Sensor.ranging_data.range_mm/1000.0);
//      Serial.print(pid_output_pitch);
//      Serial.print(" ");
//      Serial.print(pid_output_roll);
//      Serial.print(" ");
//      Serial.print(pid_output_yaw);
//      Serial.print(" ");
//      Serial.print(vel_z);
    //  Serial.print(" ");
    //  Serial.print(Distance_Sensor.ranging_data.range_mm);
    //  Serial.print(" ");
    //  Serial.print(z_cal);
    //  Serial.print(" ");
    //  Serial.print(pos_z);
    //  Serial.print(" ");
    //  Serial.print(vel_z);
    //  Serial.print("  ");
//      Serial.print(pixel_flow_x_sum);
//      Serial.print("  ");
//      Serial.print(vel_x);
//      Serial.print("  ");
//      Serial.println(vel_y);
    //  Serial.print("  ");
    //  Serial.print(timer_counter/1000.0);
    //  Serial.print((micros()-loop_timer)/1000.0);
//      Serial.print(pid_pitch_angle_i_error);
//      Serial.print(" ");
//
//        float temp;
//        int16_t temp_int;
//        bool zero=0;
//        
//        temp = 11.123*100;
//        temp_int = (int) (temp)&0xff;
//        
//       while(1)
//       {
//        Serial.print(zero,BIN);
//        if()
//        {
//          break;
//        }
//       }
//        Serial.print(temp_int,BIN);
//        Serial.write(temp_int);
//        serialFloatPrint(45.01);
      Serial.print(Angle_Roll);
      Serial.print(" ");
      Serial.print(Angle_Pitch);
      Serial.print(" ");
//      Serial.print(pid_pitch_angle_i_error);
//      Serial.print(" ");
//      Serial.print(pid_roll_angle_i_error);
//      Serial.print(" ");
//      Serial.print(Angle_Yaw);
//      Serial.print(" ");
//      Serial.print(-base_pos_z);
//      Serial.print(D_Roll*57.296);
//      Serial.print(" ");
//      Serial.print(D_Pitch*57.296);
//      if (Serial.available() > 0) 
//      {
//            incomingByte = Serial.read();
//            if(incomingByte=='w')
//            {
//              D_Pitch-=0.1/57.296;
//            }
//            if(incomingByte=='s')
//            {
//              D_Pitch+=0.1/57.296;
//            }
//            if(incomingByte=='a')
//            {
//              D_Roll-=0.1/57.296;
//            }
//            if(incomingByte=='d')
//            {
//              D_Roll+=0.1/57.296;
//            }
//            
//           
//        }


//      Serial.print(" ");
//      Serial.print(Rate_Roll_c);
//      Serial.print(" ");
//      Serial.print(Rate_Pitch_input);
//      Serial.print(" ");
//      Serial.print(Rate_Yaw_input);
//      Serial.print(" ");
//      Serial.print(pid_roll_setpoint);
//      Serial.print(" ");
//      Serial.print(pid_pitch_setpoint);
//      Serial.print(" ");
//      Serial.print(pid_last_roll_d_error);
//      Serial.print(" ");
//      Serial.print(pid_last_pitch_d_error);
//      Serial.print(" ");
//      Serial.print(pid_last_yaw_d_error);
//      Serial.print(" ");
//      Serial.print(actual_compass_heading);
    //  Serial.print(" ");
    //  Serial.print(vel_x_o*1000);
    //  Serial.print(" ");
    //  Serial.print(vel_y_o*1000);
    //  Serial.print(" ");
//      Serial.print(pixel_flow_x_sum);
//      Serial.print(" ");
    //  Serial.print(mean_x);
    //  Serial.print(qual);
  Serial.println();
  }
  if(DEBUG_MODE_MATLAB)
  {
    int number1 =Angle_Roll;
    int number2 =Angle_Pitch;
    int number3 =Angle_Yaw;
    
    byte low1 = number1;
    byte high1 = number1 >> 8;
    byte low2 = number2;
    byte high2 = number2 >> 8;
    byte low3 = number3;
    byte high3 = number3 >> 8;
    
    Serial.write(low1);
    Serial.write(high1);
    Serial.write(low2);
    Serial.write(high2);
    Serial.write(low3);
    Serial.write(high3);
    
    Serial.println("");
  }
}

void serialFloatPrint(float f) {
  byte * b = (byte *) &f;
  Serial.print("f:");
//  Serial.write(b[0]);
//  Serial.write(b[1]);
//  Serial.write(b[2]);
//  Serial.write(b[3]);
//  /* DEBUG */
  Serial.println();
  Serial.print(b[0],BIN);
  Serial.print(b[1], BIN);
  Serial.print(b[2], BIN);
  Serial.println(b[3], BIN);
  //*/
}
