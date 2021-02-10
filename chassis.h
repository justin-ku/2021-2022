#ifndef ROBOT_H
#define ROBOT_H

namespace Chassis {
  void setDrive(double left, double right);
  void setLeft(double power);
  void setRight(double power);
  void initialize();
  void opDrive();
  double getWheelVelocity(int side);
}

// include other subsystems ex. lift, intake, etc in the robot folder

#endif /* end of include guard: ROBOT_H */
