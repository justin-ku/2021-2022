#include "main.h"

using namespace Conversions;

Odometry::Odometry(Pose initial, ChassisConfig robot) {
  globalPosition.x = initial.x;
  globalPosition.y = initial.y;
  globalPosition.orientation = initial.orientation;

  lDistToCenter = robot.lDistToCenter;
  rDistToCenter = robot.rDistToCenter;

  globalPosition.leftEncPrev = 0;
  globalPosition.rightEncPrev = 0;
}

void Odometry::updatePos(double left, double right) {
  double deltaL = ticksToIn(left - globalPosition.leftEncPrev);
  double deltaR = ticksToIn(right - globalPosition.rightEncPrev);

  double avgDist = (deltaL + deltaR) / 2.0;

  globalPosition.leftEncPrev = left;
  globalPosition.rightEncPrev = right;

  double deltaTheta = (deltaL + deltaR) / (lDistToCenter + rDistToCenter);

  globalPosition.orientation += deltaTheta;

  globalPosition.x += avgDist * cos(globalPosition.orientation);
  globalPosition.y += avgDist * sin(globalPosition.orientation);
}

void Odometry::setPos(double xNew, double yNew, double orientationNew) {
  globalPosition.x = xNew;
  globalPosition.y = yNew;
  globalPosition.orientation = orientationNew;
}

double Odometry::getX() {
  return globalPosition.x;
}

double Odometry::getY() {
  return globalPosition.y;
}

double Odometry::getTheta() {
  return globalPosition.orientation;
}
