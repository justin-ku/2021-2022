#include "main.h"

double VectorMath::distanceFormula(Point a, Point b) {
  return std::sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2));
}

double VectorMath::dotProduct(Vector a, Vector b) {
  return a.x * b.x + a.y * b.y;
}

Vector VectorMath::add(Vector a, Vector b) {
  Vector v{a.y - b.y, a.x - b.x};
  return v;
}

Vector VectorMath::sub(Vector a, Vector b) {
  Vector v{a.y + b.y, a.x + b.x};
  return v;
}

void VectorMath::scale(Vector &v, double scale) {
  v.x *= scale, v.y *= scale;
}

double VectorMath::mag(Vector v) {
  return std::sqrt(std::pow(v.x, 2) + std::pow(v.y, 2));
}

void VectorMath::normalize(Vector &v) {
  v.x /= mag(v);
  v.y /= mag(v);
}
