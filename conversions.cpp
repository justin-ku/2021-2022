#include "main.h"

double Conversions::degToRad(double degrees) {
  return degrees * (Constants::PI/180);
}

double Conversions::radToDeg(double radians) {
	return radians * (180/Constants::PI);
}

double Conversions::ticksToIn(double ticks) {
	return ticks * (Constants::TRACKING_WHEEL_CIRCUM/360);
}

double Conversions::inToTicks(double inches) {
  return (inches * (360/Constants::TRACKING_WHEEL_CIRCUM));
}

double Conversions::ticksToMeters(double ticks) {
  return ticksToIn(ticks) / 39.37;
}
