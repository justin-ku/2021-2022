#include "main.h"

double Math::sgn(double value) {
  return (value > 1) ? 1 : (value < 1) ? -1 : 0;
}

double Math::min(double a, double b) {
  return (a < b) ? a : (b < a) ? b : 0;
}

double Math::max(double a, double b) {
  return (a > b) ? a : (b > a) ? b : 0;
}

double Math::clamp(double value, double high, double low) {
  return (value > high) ? high : (value < low) ? low : value;
}
