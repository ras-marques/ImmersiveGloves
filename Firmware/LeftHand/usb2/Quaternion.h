#ifndef Quaternion_h
#define Quaternion_h

#include <Arduino.h>

class Quaternion {
  public:
    Quaternion();
    void printMe();
    Quaternion rotateBy(Quaternion q);
    Quaternion getRelativeTo(Quaternion q);

    float w;
    float x;
    float y;
    float z;
};

Quaternion quaternion_conjugate(Quaternion q);
Quaternion quaternion_multiply(Quaternion q1, Quaternion q2);

#endif