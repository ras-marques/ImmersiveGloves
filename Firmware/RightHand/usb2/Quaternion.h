#ifndef Quaternion_h
#define Quaternion_h

#include <Arduino.h>

class Quaternion {
  public:
    Quaternion();
    void printMe();
    void printMyEulerAngles_pry();
    void printMyEulerAngles_ypr();
    void printMyEulerAngles_ryp();
    Quaternion rotateBy(Quaternion q);
    Quaternion getRelativeTo(Quaternion q);

    float w;
    float x;
    float y;
    float z;
};

float getRoll_pry(Quaternion q);
float getYaw_pry(Quaternion q);
float getPitch_pry(Quaternion q);

float getRoll_ypr(Quaternion q);
float getYaw_ypr(Quaternion q);
float getPitch_ypr(Quaternion q);

float getRoll_ryp(Quaternion q);
float getYaw_ryp(Quaternion q);
float getPitch_ryp(Quaternion q);

Quaternion quaternion_conjugate(Quaternion q);
Quaternion quaternion_multiply(Quaternion q1, Quaternion q2);

#endif