#include <math.h>
double ceil(double x) {
  int ix = (int)x;
  if (ix < x) ix++;
  return (double) ix;
}

double floor(double x) {
  int ix = (int)x;
  return (double) ix;
}

double round(double x) {
  int ix = (int)(x + 0.5);
  return (double) ix;
}



