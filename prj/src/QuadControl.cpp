#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();


    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kdPosXY = config->Get(_config + ".kdPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  kdPosZ = config->Get(_config + ".kdPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);
  kdVelXY = config->Get(_config + ".kdVelXY", 0);
  kiVelXY = config->Get(_config + ".kiVelXY", 0);

  Kp_ang_vel = config->Get(_config + ".Kp_ang_vel", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  	
	float t1 = momentCmd.x;
	float t2 = momentCmd.y;
	float t3 = -momentCmd.z;
	float t4 = collThrustCmd;

	cmd.desiredThrustsN[0] = (t1 + t2 + t3 + t4) / 4.f;  // front left  - f1
	cmd.desiredThrustsN[1] = (-t1 + t2 - t3 + t4) / 4.f; // front right - f2
	cmd.desiredThrustsN[2] = (t1 - t2 - t3 + t4) / 4.f; // rear left   - f4
	cmd.desiredThrustsN[3] = (-t1 - t2 + t3 + t4) / 4.f; // rear right  - f3

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  V3F momentCmd;
 
  momentCmd = kpPQR * (pqrCmd - pqr);
  
  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F des_angles_I_frame, Quaternion<float> attitude)
{
  
  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

    
  //------------ euler angles variant--------
  // quaternion to euler (u(R(q)))
 

  V3D eulerAngles = attitude.ToEulerRPY();

  //---- Getting desired angles in inertial frame
  float desired_phi_x_inertial =  -des_angles_I_frame.y;
  float desired_phi_y_inertial =  -des_angles_I_frame.x; 

  desired_phi_x_inertial = CONSTRAIN(desired_phi_x_inertial, -maxTiltAngle, maxTiltAngle);
  desired_phi_y_inertial = CONSTRAIN(desired_phi_y_inertial, -maxTiltAngle, maxTiltAngle);

  
  // Translating desired angles from inertial frame to body frame
  //roll euler
  float desired_phi_x_euler = - ( R(0, 0)*desired_phi_x_inertial + R(0,1)*desired_phi_y_inertial);//0;
  // pitch euler
  float desired_phi_y_euler = R(1, 0)*desired_phi_x_inertial + R(1, 1)*desired_phi_y_inertial;//0;
   
  // Getting estimated euler angles
  float phi_x_euler = eulerAngles.x; // phi
  float phi_y_euler = eulerAngles.y; // thetta

  // Desired angular velocities in Euler angles
  float desired_phi_x_euler_dot = Kp_ang_vel *(desired_phi_x_euler - phi_x_euler);
  float desired_phi_y_euler_dot = Kp_ang_vel *(desired_phi_y_euler - phi_y_euler);
 

  // Translating to desired angular velocities in body frame
  pqrCmd.x = desired_phi_x_euler_dot;// *1 + desired_pitch_dot * 0; // + d_yaw*(-sin(pitch))
  pqrCmd.y = cos(phi_x_euler)*desired_phi_y_euler_dot; // + sin(roll)*cos(pitch) * d_yaw
  pqrCmd.z = 0;
    
  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float posZ)
{
	
  static float error_last = 0;
  static float i_err = 0;


  float z_err = posZCmd - posZ;
  float p_term = kpPosZ * z_err;


  float d_err = z_err - error_last;
  float d_term = kdPosZ * d_err;
  error_last = z_err;


  i_err += z_err;
  float i_term = i_err * KiPosZ;
  i_term = CONSTRAIN(i_term, -5, 5);   

  float thrust = - (p_term + d_term + i_term);// +i_term;// +accelZCmd;
    
  return thrust;
}

// returns a desired angles in global frame
V3F QuadControl::PositionControl(V3F posCmd, V3F pos, V3F vel)
{


 // PD position controller
	/*
  static V3F error_last(0,0,0);

  V3F angles_in_inertial_frame;
  V3F kp;
  kp.x = kpPosXY;
  kp.y = kpPosXY;
  kp.z = 0.f;

  V3F kd;
  kd.x = kdPosXY;
  kd.y = kdPosXY;
  kd.z = 0.f;

  V3F error = posCmd - pos;   
  V3F d_error = error - error_last;

  V3F d_term = kd * d_error;

  V3F p_term = kp * error;

  angles_in_inertial_frame =  p_term + d_term;  
  error_last = error;
  angles_in_inertial_frame.z = 0;
  */
  

// cascaded control position P -> velocity PID
  static V3F i_error_vel(0, 0, 0);

  V3F angles_in_inertial_frame;
  V3F kp;
  kp.x = kpPosXY;
  kp.y = kpPosXY;
  kp.z = 0.f;  

  V3F error_pos = posCmd - pos;
  V3F des_velocity = kp * error_pos;

  static V3F error_vel_last(0,0,0);
  V3F error_vel = des_velocity - estVel;
  V3F p_term_vel = kpVelXY * error_vel;
  
  
  V3F d_term_vel = kdVelXY * (error_vel - error_vel_last);
  error_vel_last = error_vel;

  i_error_vel += error_vel;
  i_error_vel.x = CONSTRAIN(i_error_vel.x, -55, 55);
  i_error_vel.x = CONSTRAIN(i_error_vel.y, -55, 55);

  V3F i_term_vel = kiVelXY * i_error_vel;

  angles_in_inertial_frame = p_term_vel + d_term_vel + i_term_vel;

  return angles_in_inertial_frame;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  float err = yawCmd - yaw;
  float yawRateCmd =  kpYaw * err;  
  return yawRateCmd;
}



VehicleCommand QuadControl::control_with_euler_angles(TrajectoryPoint& curTrajPoint)
{
	float collThrustCmd = AltitudeControl(curTrajPoint.position.z, estPos.z);

	// reserve some thrust margin for angle control
	float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
	collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust + thrustMargin)*4.f, (maxMotorThrust - thrustMargin)*4.f);

	V3F desAngles_I_frame = PositionControl(curTrajPoint.position, estPos, estVel);

	V3F desOmega = RollPitchControl(desAngles_I_frame, estAtt);
	desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

	V3F desMoment = BodyRateControl(desOmega, estOmega);

	return GenerateMotorCommands(collThrustCmd, desMoment);
}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  return control_with_euler_angles(curTrajPoint);
  
}
