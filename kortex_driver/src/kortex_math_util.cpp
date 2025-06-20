// Copyright (c) 2019 Kinova inc. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

#include <kortex_driver/kortex_math_util.hpp>

double KortexMathUtil::toRad(double degree) { return degree * M_PI / 180.0; }

double KortexMathUtil::toDeg(double rad) { return rad * 180.0 / M_PI; }

double KortexMathUtil::wrapRadiansFromMinusPiToPi(double rad_not_wrapped)
{
  bool properly_wrapped = false;
  int number_of_turns = 0;
  do
  {
    if (rad_not_wrapped > M_PI)
    {
      number_of_turns += 1;
      rad_not_wrapped -= 2.0 * M_PI;
    }
    else if (rad_not_wrapped < -M_PI)
    {
      number_of_turns -= 1;
      rad_not_wrapped += 2.0 * M_PI;
    }
    else
    {
      properly_wrapped = true;
    }
  } while (!properly_wrapped);
  return rad_not_wrapped;
}

double KortexMathUtil::wrapDegreesFromZeroTo360(double deg_not_wrapped)
{
  bool properly_wrapped = false;
  int number_of_turns = 0;
  do
  {
    if (deg_not_wrapped > 360.0)
    {
      number_of_turns += 1;
      deg_not_wrapped -= 360.0;
    }
    else if (deg_not_wrapped < 0.0)
    {
      number_of_turns -= 1;
      deg_not_wrapped += 360.0;
    }
    else
    {
      properly_wrapped = true;
    }
  } while (!properly_wrapped);
  return deg_not_wrapped;
}
