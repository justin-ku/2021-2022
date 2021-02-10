#ifndef VECTOR_MATH_H
#define VECTOR_MATH_H

#include "autoLib/odometry.h" // for definition of a 'point'
#include "autoLib/purePursuit.h" // for definition of a 'vector'

namespace VectorMath {
  double distanceFormula(Point a, Point b);
  double dotProduct(Vector a, Vector b);
  Vector add(Vector a, Vector b);
  Vector sub(Vector a, Vector b);
  void scale(Vector &v, double scale);
  double mag(Vector v);
  void normalize(Vector &v);
}

#endif
