void transformation(void)
{
  phi_r = (3.1416/180)*Angle_Roll;                           //convert measvel_xred Evel_xler angles in body frame from degrees to radians
  theta_r = (3.1416/180)*Angle_Pitch;
  psi_r = (3.1416/180)*Angle_Yaw;

  
  float base_vel_x,base_vel_y,p,q;


  p=vel_x+sin(theta_r)*vel_z;
  q=vel_y-sin(phi_r)*cos(theta_r)*vel_z;

  base_vel_x=(tan(theta_r)*tan(phi_r)*sin(phi_r)+cos(psi_r)/cos(theta_r))*p-(sin(psi_r)/cos(phi_r))*q;
  base_vel_y=(-tan(theta_r)*tan(phi_r)*cos(phi_r)+sin(psi_r)/cos(theta_r))*p+(cos(psi_r)/cos(phi_r))*q;



//  offset_x=9.8*(sin(phi_r)*sin(psi_r)+cos(phi_r)*sin(theta_r)*cos(psi_r));
//  offset_y=9.8*(-cos(phi_r)*sin(psi_r)+sin(phi_r)*sin(theta_r)*cos(psi_r));
//  offset_z=9.8*(cos(theta_r)*cos(psi_r));
  
//  odo_x+=Acc_X*0.000025;
//  odo_y+=Acc_Y*0.000025;
//  odo_z+=Acc_Z*0.000025;

  base_pos_x+=base_vel_x*0.005;
  base_pos_y+=base_vel_y*0.005;
  
  //base_pos_x += (cos(theta_r)*cos(psi_r))*vel_x*0.005 + (-cos(phi_r)*sin(psi_r)+sin(phi_r)*sin(theta_r)*cos(psi_r))*vel_y*0.005 + (sin(phi_r)*sin(psi_r)+cos(phi_r)*sin(theta_r)*cos(psi_r))*vel_z*0.005; //conversion formvel_xlae for body frame to inertial frame
  //base_pos_y += (cos(theta_r)*sin(psi_r))*vel_x*0.005 + (cos(phi_r)*cos(psi_r)+sin(phi_r)*sin(theta_r)*sin(psi_r))*vel_y*0.005 + (-sin(phi_r)*cos(psi_r)+cos(phi_r)*sin(theta_r)*sin(psi_r))*vel_z*0.005;
  
  base_pos_z = cos(phi_r)*cos(theta_r)*pos_z;

}
