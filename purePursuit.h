#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#include "autoLib/odometry.h" // for definition of a 'point'
#include <vector>
#include <array>

struct Vector {
  double x;
  double y;
};

struct Path {
  std::vector<Point> points;
  std::vector<double> distancesToEachPoint;
  std::vector<double> maxVelocitiesAtEachPoint;
};

class PurePursuit {
private:
  std::array<Path, 5> listPaths;
  Odometry odom;
  ChassisConfig robot;
  double maxVelocity;
  double maxAcceleration;
  enum Sides {left, right};
  double targetVelocities[2];
  double targetAcceleration[2];
  double trackWidth;
  double m_kv;
  double m_ka;
  double m_kp;

  std::vector<Point> injectPoints(std::initializer_list<Point> waypoints, int spacing = 6);
  std::vector<Point> smoothPath(std::vector<Point> path, double a, double b, double tolerance = 0.001);
  Path computeVelocities(std::vector<Point> path, double maxVelocity);
  double computeTargetVelocity(std::vector<double> maxVelocities);
  double rateLimiter(double input, double maxRate, char pathIndex);
  double findIntersection(Point initial, Point final, double lookAheadDistance);
  Point findLookAheadPoint(std::vector<Point> path, char pathIndex, double lookaheadDistance);
  double computeTargetAcceleration(double targetVelocity, bool isLeft, char pathIndex);

public:
  PurePursuit(Pose initial, ChassisConfig robot, double kv, double ka, double kp);
  void generatePath(std::initializer_list<Point> waypoints, double weightSmooth, double maxVelocity, char index);
  void runPath(char index, double lookAheadDistance, bool backwards = false);
};


#endif /* end of include guard: PURE_PURSUIT_H */
