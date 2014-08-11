/* This file is part of the Razor AHRS Firmware */

void Compass_Heading()
{
  float mag_x;
  float mag_y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
  /*
    	float temp1[3];
	float temp2[3];
	float xAxis[] = {1.0f, 0.0f, 0.0f};
  	pitch = -atan2(accel_[0], sqrt(accel_[1] * accel_[1] + accel_[2] * accel_[2]));
	
	// GET ROLL
	// Compensate pitch of gravity vector
	Vector_Cross_Product(temp1, accel_, xAxis);
	Vector_Cross_Product(temp2, xAxis, temp1);
	// Normally using x-z-plane-component/y-component of compensated gravity vector
	// roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
	// Since we compensated for pitch, x-z-plane-component equals z-component:
	roll = atan2(temp2[1], temp2[2]);
  cos_roll = cos(roll);
  sin_roll = sin(roll);
  cos_pitch = cos(pitch );
  sin_pitch = sin(pitch );
  
 */
  cos_roll = cos(gyro_angle[1]);
  sin_roll = sin(gyro_angle[1]);
  cos_pitch = cos(gyro_angle[0] );
  sin_pitch = sin(gyro_angle[0] );
  // Tilt compensated magnetic field X
  mag_x = magnetom[0] * cos_pitch + magnetom[1] * sin_roll * sin_pitch + magnetom[2] * cos_roll * sin_pitch;
  // Tilt compensated magnetic field Y
  mag_y = magnetom[1] * cos_roll - magnetom[2] * sin_roll;
  // Magnetic Heading
  MAG_Heading = atan2(- mag_y, mag_x);
}

void Compass_compensate(){
  float Ax2 = accel_[0]*accel_[0];float Ay2 = accel_[1]*accel_[1];float Az2 = accel_[2]*accel_[2];
  float sqrtnorm = sqrt(Ax2+Ay2+Az2);
  float Ax = accel_[0]/sqrtnorm;float Ay = accel_[1]/sqrtnorm;float Az = accel_[2]/sqrtnorm;
  float Axx= Ax*Ax;float Ayy= Ay*Ay; float Azz= Az*Az;
  float mag_y =  magnetom[1]*sqrt(1-Ax*Ax -Ay*Ay)  - magnetom[2] *Ay;
   float mag_x =  magnetom[0]*(1-Axx)-(magnetom[1]*Ax*Ay) -magnetom[2]*Ax*sqrt(1-Axx-Ayy); 
    MAG_Heading1 = atan2(- mag_y, mag_x);
}
