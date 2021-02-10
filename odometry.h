#ifndef ODOMETRY_H
#define ODOMETRY_H

struct Point {
  double x;
  double y;
};

// stores robot's initial position
struct Pose {
  double x;
  double y;
  double orientation;
};

// stores robot's position data (x, y, orientation)
struct Position {
  double x;
  double y;
  double orientation;
  double leftEncPrev;
  double rightEncPrev;
};

struct ChassisConfig {
  double lDistToCenter; // left tracking wheel to robot center
  double rDistToCenter; // right tracking wheel to robot center
  double trackWidth; // distance between left and right wheels
};

class Odometry {
private:
  Position globalPosition;
  ChassisConfig robot;

  double lDistToCenter;
  double rDistToCenter;
public:
  Odometry(Pose initial, ChassisConfig robot);
  void updatePos(double left, double right);
  void setPos(double xNew, double yNew, double orientationNew);
  double getX();
  double getY();
  double getTheta();
};

#endif /* end of include guard: ODOMETRY_H */
