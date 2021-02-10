#include "main.h"

PurePursuit::PurePursuit(Pose initial, ChassisConfig robot, double kv, double ka, double kp) :
  odom(initial, robot), trackWidth(robot.trackWidth), m_kv(kv), m_ka(kp), m_kp(kp) {}

std::vector<Point> PurePursuit::injectPoints(std::initializer_list<Point> waypoints, int spacing) {
  std::vector<Point> givenPath = waypoints;
  std::vector<Point> newPath;

  int numLineSegments = givenPath.size() - 1;

  for(int i = 0; i < numLineSegments; i++) {
    Point a = givenPath.at(i); // start point
    Point b = givenPath.at(i + 1); // end point

    // find line segment using start and end points
    Vector lineSeg{b.x - a.x, b.y - a.y};

    // find number of evenly spaced points
    int numPointsThatFit = std::ceil(VectorMath::mag(lineSeg) / spacing);

    // normalize & scale
    VectorMath::normalize(lineSeg);
    VectorMath::scale(lineSeg, spacing);

    // insert points into the new path
    for(int i = 0; i < numPointsThatFit; i++) {
      Point newPoint = Point{a.x + lineSeg.x * (double)i, a.y + lineSeg.y * (double)i};
      newPath.push_back(newPoint);
    }
  }

  // add the last point in the original path to new path
  newPath.push_back(givenPath.at(givenPath.size() - 1));

  return newPath;
}

/*
 * - larger b means a smoother path (0.75 <= b <= 0.98)
 * - a = 1 - b
 * - tolerance = 0.001
*/

std::vector<Point> PurePursuit::smoothPath(std::vector<Point> path, double a, double b, double tolerance) {
  std::vector<Point> newPath = path;
  double change = tolerance;

  while(change >= tolerance) {
    change = 0.0;
    for(int i = 1; i < path.size() - 1; i++) {
      Point aux = newPath.at(i);
      newPath.at(i).x += a * (path.at(i).x - newPath.at(i).x) + b * (newPath.at(i-1).x + newPath.at(i+1).x - (2.0 * newPath.at(i).x));
      newPath.at(i).y += a * (path.at(i).y - newPath.at(i).y) + b * (newPath.at(i-1).y + newPath.at(i+1).y - (2.0 * newPath.at(i).y));
			change += std::abs(aux.x - newPath.at(i).x);
      change += std::abs(aux.y - newPath.at(i).y);
    }
  }

  return newPath;
}

Path PurePursuit::computeVelocities(std::vector<Point> path, double maxVelocity) {
  // stores distance to each waypoint along the path
  std::vector<double> distToEachPoint;
  distToEachPoint.push_back(0.0); // first point

  // keep a running sum of the distances between points
  for(int i = 1; i < path.size(); i++) {
    double distToPoint = distToEachPoint.at(i - 1) + VectorMath::distanceFormula(path.at(i), path.at(i - 1));
    distToEachPoint.push_back(distToPoint);
  }

  // stores curvature at each waypoint
  std::vector<double> pathCurvatures;

  // start and end points don't have a point on either side so curvature is 0
  pathCurvatures.push_back(0.0);

  // calculate curvature at each waypoint
  for(int i = 1; i < path.size() - 1 ; i++) {
    // find the radius of the circle that intersects the point (p) and the two points (q, r) on either side of it
    Point p = path.at(i);
    double x1 = p.x + 0.001; // avoid divide by zero
    double y1 = p.y;
    Point q = path.at(i - 1);
    double x2 = q.x;
    double y2 = q.y;
    Point r = path.at(i + 1);
    double x3 = r.x;
    double y3 = r.y;

    double k1 = 0.5 * (pow(x1, 2) + pow(y1, 2) - pow(x2, 2) - pow(y2, 2)) / (x1 - x2);
    double k2 = (y1 - y2) / (x1 - x2);
    double b = 0.5 * (pow(x2, 2) - 2 * x2 * k1 + pow(y2, 2) - pow(x3, 2) + 2 * x3 * k1 - pow(y3, 2)) / (x3 * k2 - y3 + y2 - x2 * k2);
    double a = k1 - k2 * b;
    double radius = sqrt(pow(x1 - a, 2) + pow(y1 - b, 2));
    double curvatureAtPoint = 1 / radius;

    // if NaN, then curvature is 0 and path is a straight line
    if(std::isnan(radius)) {
      pathCurvatures.push_back(0.0);
    } else {
      pathCurvatures.push_back(curvatureAtPoint);
    }
  }

  // start and end points don't have a point on either side so curvature is 0
  pathCurvatures.push_back(0.0);

  // stores target velocities at each point
  std::vector<double> targetVelocities;

  // k value ranges from 1 - 5 depending how you want the robot to handle tight turns
  const double k = 1.0;

  // calculate max velocity at each point while taking account of curvature
  for(int i = 0; i < pathCurvatures.size(); i++) {
    // avoid division by 0
    if(pathCurvatures.at(i)) { // curvature is nonzero
      targetVelocities.push_back(Math::min(maxVelocity, k / pathCurvatures.at(i)));
    } else { // curvature is zero
      targetVelocities.push_back(maxVelocity);
    }
  }

  // we want the robot to stop when it reaches the final point
  targetVelocities.at(targetVelocities.size() - 1) = 0.0;

  // calculate new target velocities
  for(int i = targetVelocities.size() - 2; i > -1; i--) {
    // use kinematic equation for Vf to find target velocities
    double distance = VectorMath::distanceFormula(path.at(i + 1), path.at(i)); // find d
    double Vf = sqrt(pow(targetVelocities.at(i + 1), 2) + 2 * maxAcceleration * distance);
    // ensure that target velocities don't exceed max velocities
    targetVelocities.at(i) = Math::min(targetVelocities.at(i), Vf);
  }

  // generate a final path that stores the waypoints, distances, and velocity information
  Path output;

  output.points = path;
  output.distancesToEachPoint = distToEachPoint;
  output.maxVelocitiesAtEachPoint = targetVelocities;

  return output;
}

// length depends on number of paths?
std::vector<double> rateLimiterPrevTimes {0.0, 0.0, 0.0, 0.0, 0.0};
std::vector<double> prevRateLimiterOutput {0.0, 0.0, 0.0, 0.0, 0.0};
std::vector<double> rateLimiterOutput {0.0, 0.0, 0.0, 0.0, 0.0};

double PurePursuit::rateLimiter(double input, double maxRate, char pathIndex) {
  double currTime = (double)pros::millis();
  double maxChange = (currTime - rateLimiterPrevTimes.at(pathIndex)) * maxRate;
  rateLimiterOutput.at(pathIndex) += Math::clamp(input - prevRateLimiterOutput.at(pathIndex), -maxChange, maxChange);

  rateLimiterPrevTimes.at(pathIndex) = currTime;
  prevRateLimiterOutput.at(pathIndex) = rateLimiterOutput.at(pathIndex);

  return rateLimiterOutput.at(pathIndex);
}

/* find the lookahead point by finding the intersection point of the circle (of radius lookahead distance
 * centered at the robot’s location) and the path segments
 */
double PurePursuit::findIntersection(Point initial, Point final, double lookAheadDistance) {
  Vector d = Vector {final.x - initial.x, final.y - initial.y};
  Vector f = Vector {initial.x - odom.getX(), initial.y - odom.getY()};
  double a = VectorMath::dotProduct(d, d);
  double b = 2.0 * VectorMath::dotProduct(f, d);
  double r = lookAheadDistance;
  double c = VectorMath::dotProduct(f, f) - pow(r, 2);
  double discriminant = pow(b, 2) - 4.0 * a * c;

  if(discriminant < 0) {
    return -1;
  } else {
    discriminant = sqrt(discriminant);
    double t1 = (-b - discriminant) / (2 * a);
    double t2 = (-b + discriminant) / (2 * a);

    if(t1 >= 0 && t1 <= 1) {
      return t1;
    } else if(t2 >= 0 && t2 <= 1) {
      return t2;
    } else {
      return -1;
    }
  }
}

std::array<Point, 5> prevLookaheadPoints = {Point{0.0, 0.0}, Point{0.0, 0.0}, Point{0.0, 0.0}, Point{0.0, 0.0}, Point{0.0, 0.0}};
std::array<double, 5> prevFractionalIndexes = {0.0, 0.0, 0.0, 0.0, 0.0};

Point PurePursuit::findLookAheadPoint(std::vector<Point> path, char pathIndex, double lookaheadDistance) {
  std::vector<double> tValues;
  std::vector<double> fractionalIndexes;
  std::vector<Point> intersectionPoints;
  //calculate intersections and fractional indexes for every line segment in path
  for(int i = 1; i < path.size(); i++) {
    Point initial = path.at(i - 1);
    Point final = path.at(i);

    double tValue = findIntersection(initial, final, lookaheadDistance);

    if(tValue == -1) {

    } else {
      tValues.push_back(tValue);
      fractionalIndexes.push_back(tValue + (i - 1));
      Vector direction = Vector{final.x - initial.x, final.y - initial.y};
      Point intersectionPoint = Point{initial.x + direction.x * tValue, initial.y + direction.y * tValue};
      intersectionPoints.push_back(intersectionPoint);
    }
  }

  // determine first valid intersection
  double currFractionalIndex = -1;
  Point lookaheadPoint = prevLookaheadPoints.at(pathIndex);

  for(int i = 0; i < path.size(); i++) {
    double tValue = tValues.at(i);
    double currFractionalIndex = fractionalIndexes.at(i);
    //check if current fractional index is larger than the previous fractional index, and check if t is in [0, 1]
    if(currFractionalIndex > prevFractionalIndexes.at(pathIndex) && (0.0 <= tValue && tValue <= 1.0)) {
      lookaheadPoint = intersectionPoints.at(i);
      break;
    }
  }
  //update previous lookahead point and fractional index
  prevFractionalIndexes.at(pathIndex) = currFractionalIndex;
  prevLookaheadPoints.at(pathIndex) = lookaheadPoint;

  return lookaheadPoint;
}

std::array<double, 5> prevLeftTargetVelocities = {0.0, 0.0, 0.0, 0.0, 0.0};
std::array<double, 5> prevRightTargetVelocities = {0.0, 0.0, 0.0, 0.0, 0.0};
std::array<int, 5> prevLeftTime = {0, 0, 0, 0, 0};
std::array<int, 5> prevRightTime = {0, 0, 0, 0, 0};

double PurePursuit::computeTargetAcceleration(double targetVelocity, bool isLeft, char pathIndex) {
  double currTime = (double)pros::millis();
  double deltaV, deltaT;
  //check if we are calculating for left or right side
  if(isLeft) { //is left
    deltaV = targetVelocity - prevLeftTargetVelocities.at(pathIndex);
    deltaT = currTime - (double)prevLeftTime.at(pathIndex) / 1000.0;
  } else { //else, is right
    deltaV = targetVelocity - prevRightTargetVelocities.at(pathIndex);
    deltaT = currTime - (double)prevRightTime.at(pathIndex) / 1000.0;
  }
  // slope of velocity time graph = acceleration
  double acceleration = deltaV / deltaT;

  return acceleration;
}

void PurePursuit::runPath(char index, double lookAheadDistance, bool backwards) {
  //finding distance to last point in the path, used for stop condition
  Point last = listPaths.at(index).points.at(listPaths.at(index).points.size() - 1);
  Point robotPos{odom.getX(), odom.getY()};
  double distanceToLastPoint = VectorMath::distanceFormula(robotPos, last);
  double signedCurvature;

  // keep moving until robot is 3 inches away from endpoint
  while(distanceToLastPoint > 3.0) {
    odom.updatePos(leftE.get_value(), rightE.get_value());

    // find closest point's target velocity (going to be used later) and lookahead point
    double targetVelocity = computeTargetVelocity(listPaths.at(index).maxVelocitiesAtEachPoint);
    Point lookAheadPoint = findLookAheadPoint(listPaths.at(index).points, index, lookAheadDistance);

    // calculate arc from robot position to lookahead point
    // get robot position and orientation
    double robotAngle = odom.getTheta();
    double robotX = odom.getX();
    double robotY = odom.getY();

    // find equation of the "robot-line" and convert to the form ax + by + c
    double a = -tan(robotAngle);
    double b = 1.0;
    double c = tan(robotAngle) * (robotX - robotY);

    // point line distance formula
    double x = fabs(a * lookAheadPoint.x + b * lookAheadPoint.y + c) / sqrt(pow(a, 2) + pow(b, 2));

    // calculate curvature
    double curvature = (2 * x) / pow(lookAheadDistance, 2);

    //calculate side to turn to
    double Rx = robotX;
    double Ry = robotY;
    double Lx = lookAheadPoint.x;
    double Ly = lookAheadPoint.y;
    double Bx = Rx + cos(robotAngle);
    double By = Ry + sin(robotAngle);
    double side = Math::sgn(sin(robotAngle) * (Lx - Rx) - cos(robotAngle) * (Ly - Ry));

    // calculate signed curvature
    signedCurvature = curvature * side;

    // calculate target wheel velocities and accelerations
    double v = rateLimiter(targetVelocity, maxAcceleration, index);
    targetVelocities[left] = v * (2 + signedCurvature * robot.trackWidth) / 2;
    targetVelocities[right] = v * (2 - signedCurvature * robot.trackWidth) / 2;
    targetAcceleration[left] = computeTargetAcceleration(targetVelocities[left], true, index);
    targetAcceleration[right] = computeTargetAcceleration(targetVelocities[right], false, index);

    // calculate feedforward terms for each side
    double leftFF = m_kv * targetVelocities[left] + m_ka * targetAcceleration[left];
    double rightFF = m_kv * targetVelocities[right] + m_ka * targetAcceleration[right];

    // get left and right velocities
    double time = (double)pros::millis() / 1000.0; // get time in seconds
    double leftVelocity = Chassis::getWheelVelocity(left);
    double rightVelocity = Chassis::getWheelVelocity(right);

    // calculate feedback terms for each side
    double leftFB = m_kp * (targetVelocities[left] - leftVelocity);
    double rightFB = m_kp * (targetVelocities[right] - rightVelocity);

    // apply power to wheels
    Chassis::setDrive(leftFF + leftFB, rightFF + rightFB);
  }
}

void PurePursuit::generatePath(std::initializer_list<Point> waypoints, double weightSmooth, double maxVelocity, char index) {
  listPaths.at(index) = computeVelocities(smoothPath(injectPoints(waypoints), 1 - weightSmooth, weightSmooth), maxVelocity);
}
