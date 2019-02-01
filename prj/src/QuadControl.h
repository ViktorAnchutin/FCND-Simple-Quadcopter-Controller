#pragma once


#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"

class QuadControl : public BaseController
{
public:
  QuadControl(string config) : BaseController(config) { Init(); };

  virtual void Init();

  // returns a desired angles in global frame
  V3F PositionControl(V3F posCmd, V3F pos, V3F vel);

  virtual VehicleCommand RunControl(float dt, float sim_time);

  VehicleCommand GenerateMotorCommands(float collThrustCmd, V3F momentCmd);

  // returns desired yaw rate
  float YawControl(float yawCmd, float yaw);

  // returns desired moments
  V3F BodyRateControl(V3F pqrCmd, V3F pqr);

  // returns a desired roll and pitch rate 
  V3F RollPitchControl(V3F des_angles_inertial_frame, Quaternion<float> attitude);

  float AltitudeControl(float posZCmd, float posZ);

  VehicleCommand control_with_euler_angles(TrajectoryPoint& curTrajPoint);


  // -------------- PARAMETERS --------------

  // controller gains
  float kpPosXY, kdPosXY, kpPosZ;
  float kpVelXY, kpVelZ, kdVelXY, kiVelXY;
  float Kp_ang_vel;
  float kpBank, kpYaw;
  float KiPosZ, kdPosZ;
  V3F kpPQR;
  
  // limits & saturations
  float maxAscentRate, maxDescentRate;
  float maxSpeedXY;
  float maxAccelXY;
  float maxTiltAngle;
  float minMotorThrust, maxMotorThrust;

  
};
