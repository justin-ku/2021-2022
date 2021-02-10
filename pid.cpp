#include "main.h"

using namespace Conversions;
using namespace Math;

void resetEncoders() {
  leftE.reset();
	rightE.reset();
}

Pid::Pid(double kP, double kI, double kD)
  : m_kP(kP), m_kI(kI), m_kD(kD) {}

void Pid::initialize() {
  m_target = 0;
  m_error = 0;
  m_prevError = 0;
  m_integral = 0;
  m_derivative = 0;
  m_output = 0;
  m_initAngle = 0;
}

double Pid::updatePid(double error) {
  m_error = error;
  m_integral += m_error;
  m_derivative = m_error - m_prevError;
  m_prevError = m_error;

  if((m_error > 0 && m_prevError < 0) || (m_error < 0 && m_prevError > 0)) m_integral = 0;

  m_output = clamp(m_error * m_kP + m_integral * m_kI + m_derivative * m_kD, 127, -127);

  return m_output;
}

bool Pid::settleUtil(double errorThresh, int timeThresh) {
  return !((std::abs(m_error) <= errorThresh || m_time >= timeThresh));
}

int slew(int targetSpd, int side) {
  static int speed[2];
  const int accStep = 9;
  const int decStep = 256;
  int step;

  if(std::abs(speed[side]) < std::abs(targetSpd)) {
    step = accStep;
  } else {
    step = decStep;
  }

  if(targetSpd > speed[side] + step) {
    speed[side] += step;
  } else if(targetSpd < speed[side] - step) {
    speed[side] -= step;
  } else {
    speed[side] = targetSpd;
  }

  return speed[side];
}

void Pid::moveDistance(double distance, int timeLimit) {
  initialize();
  resetEncoders();

  double initRotation = imu.get_rotation();
  double target = inToTicks(distance);

  while(settleUtil(0, timeLimit)) {
    double avgDist = (leftE.get_value() + rightE.get_value())/2;
    double pidOutput = updatePid(target - avgDist);
    double swerveVal = imu.get_rotation() - initRotation;

    Chassis::setLeft(slew(pidOutput - swerveVal, leftSpeed));
    Chassis::setRight(slew(pidOutput + swerveVal, rightSpeed));

    m_time += 15;

    pros::delay(15);
  }
}

void Pid::turnAngle(double angle, int timeLimit) {
  double target = angle;
  m_initAngle = imu.get_rotation();

  while(settleUtil(0, timeLimit)) {
    double avgAngle = imu.get_rotation() - m_initAngle;
    double pidOutput = updatePid(target - avgAngle);

    Chassis::setDrive(pidOutput, -pidOutput);

    m_time += 15;

    pros::delay(15);
  }
}
