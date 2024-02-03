#ifndef Finger_h
#define Finger_h

#include "Quaternion.h"
#include <Arduino.h>

class Finger {
  public:
    Finger();
    void begin(float degreesMin, float degreesMid, float degreesMax);
    void computeAxesValues(Quaternion relativeQuaternion, Quaternion sensorQuaternion);
    // void calibrateCurlDegrees(float degreesMin, float degreesMid, float degreesMax);
    void calibrateSplayDegrees(float degreesMin, float degreesMid, float degreesMax);
    
    float curlRadians, splayRadians;
    float curlDegrees, splayDegrees;
    float curlAxis, splayAxis;

  private:
    float toDegrees(float angleRadians);
    Quaternion quaternionFromAngle(float angle, int axis);
    float getPitch(Quaternion q);
    float getYaw(Quaternion q);
    float getRoll(Quaternion q);
    
    void computeToHandQuaternion(Quaternion relativeQuaternion, Quaternion fingerQuaternion);
    void computeCurlDegrees(void);
    void computeCurlAxis(void);
    void computeSplayDegrees(void);
    void computeSplayAxis(void);

    Quaternion handQuaternion;
    Quaternion fingerQuaternion;
    Quaternion toHandQuaternion;
    float curlDegreesMin, curlDegreesMid, curlDegreesMax;
    float splayDegreesMin, splayDegreesMid, splayDegreesMax;
};

#endif