// Include guard
#ifndef KORTEX_DRIVER__KORTEX_MATH_UTIL_HPP_
#define KORTEX_DRIVER__KORTEX_MATH_UTIL_HPP_

#include <cmath>

class KortexMathUtil
{
public:
  KortexMathUtil() {}
  ~KortexMathUtil() {}

  static double toRad(double degree);
  static double toDeg(double rad);
  static double wrapRadiansFromMinusPiToPi(double rad_not_wrapped);
  static double wrapDegreesFromZeroTo360(double deg_not_wrapped);
};

#endif  // KORTEX_DRIVER__KORTEX_MATH_UTIL_HPP_
