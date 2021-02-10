#ifndef PID_H
#define PID_H

class Pid {
private:
  double m_kP;
  double m_kI;
  double m_kD;
  double m_target;
  double m_error;
  double m_prevError;
  double m_integral;
  double m_derivative;
  double m_output;
  int m_time;
  double m_initAngle;

  enum Speeds {
    leftSpeed,
    rightSpeed
  };

  enum Units {
    inches,
    degrees
  };

  double updatePid(double error);
  bool settleUtil(double errorThresh, int timeThresh);

public:
  Pid(double kP = 0, double kI = 0, double kD = 0);
  void initialize();
  void moveDistance(double distance, int timeLimit);
  void turnAngle(double angle, int timeLimit);
  void setTarget(double target, int timeLimit, int unit);
};

#endif /* end of include guard: PID_H */
