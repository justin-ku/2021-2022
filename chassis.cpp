#include "main.h"

using namespace Math;

void Chassis::setDrive(double left, double right) {
	LF = left;
	LB = left;
	RF = right;
	RB = right;
}

void Chassis::setLeft(double power){
  LF = power;
  LB = power;
}

void Chassis::setRight(double power){
  RF = power;
  RB = power;
}

void Chassis::initialize() {
	LF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	LF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	RF.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	RB.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	setDrive(0, 0);
}

double getCubicValue(int joystickValue) {
  double output = 0;

  if(std::abs(joystickValue) < 50 && std::abs(joystickValue) > 0){
	   output = sgn(joystickValue) * 30;
  }

  else {
	   output = std::pow(joystickValue, 3) / std::pow(joystickValue, 2);
  }

	return output;
}

void Chassis::opDrive() {
  setDrive(
    getCubicValue(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)),
    getCubicValue(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y))
  );
}

double Chassis::getWheelVelocity(int side) {
	static double wheelVelocities[2]{};
	static double prevPositions[2]{};
	static double prevTimes[2]{};
	double currPos;

	// convert raw encoder ticks to meters so we can measure using RPM
	// in. to m. --> divide by 39.37
	if(side) // right side (meters)
		currPos = Conversions::ticksToIn(rightE.get_value()) / 39.37;
	else // left side (meters)
		currPos = Conversions::ticksToIn(leftE.get_value()) / 39.37;

  double currTime = (double) pros::millis();

  double deltaPos = (currPos - prevPositions[side]) * (3.25 * Constants::PI / 360.0);
  double deltaTime = (currTime - prevTimes[side]) / (1000.0);
  double velocity = (deltaPos / deltaTime);

  prevPositions[side] = currPos;
  prevTimes[side] = currTime;

	wheelVelocities[side] = velocity;
  return wheelVelocities[side];
}
