#pragma once
#include <math.h>

double acosStable(double x) {
  if (x < -1) {
    assert(x >= -1.00001);
    x = -1;
  }
  if (x > 1) {
    assert(x <= 1.00001);
    x = 1;
  }
  return acos(x);
}
