#include <stdio.h>
#include <math.h>

#define PI 3.14159265

float findAngle(float x1, float y1, float x2, float y2, float offsetX, float offsetY) {
  float xa, ya, xb, yb, result; 
  xa = x1 - offsetX;
  ya = y1 - offsetY;
  xb = x2 - offsetX;
  yb = y2 - offsetY;
  
  result = (atan2(yb,xb) - atan2(ya,xa)) * 180/PI;
  
  if(result < -180)
    result += 360;

  printf("The angle is: %1f degrees\n", result);
}

int main() {
  printf("Sample neck pitch -- forward:\n");
  findAngle(-54.0f, 54.0f, -54.0f, 46.0f, -50.0f, 50.0f);
 
  printf("Sample neck pitch -- backward:\n");
  findAngle(-54.0f, 54.0f, -54.0f, 46.0f, -50.0f, 50.0f);
 
  printf("Sample shoulder pitch -- upward:\n");
  findAngle(-54.0f, 54.0f, -54.0f, 46.0f, -50.0f, 50.0f);
 
  printf("Sample shoulder yaw -- forward:\n");
  findAngle(-54.0f, 54.0f, -54.0f, 46.0f, -50.0f, 50.0f);
 
  printf("Sample shoulder yaw -- backward:\n");
  findAngle(-54.0f, 54.0f, -54.0f, 46.0f, -50.0f, 50.0f);
 
  printf("Sample elbow pitch -- forward:\n");
  findAngle(-54.0f, 54.0f, -54.0f, 46.0f, -50.0f, 50.0f);
 
  return 0;
}
