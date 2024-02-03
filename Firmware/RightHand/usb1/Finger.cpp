#include "Finger.h"

Finger::Finger(){}

void Finger::begin(float degreesMin, float degreesMid, float degreesMax){
    handQuaternion.w = sqrt(2)/2;
    handQuaternion.x = 0;
    handQuaternion.y = 0;
    handQuaternion.z = -sqrt(2)/2;
    calibrateSplayDegrees(degreesMin, degreesMid, degreesMax);
}

float Finger::toDegrees(float angleRadians){
  return angleRadians*180/3.14;
}

Quaternion Finger::quaternionFromAngle(float angle, int axis){
  // angle is in radians, x axis is 0, y axis is 1, z axis is 2
  if (axis == 0){
    Quaternion q;
    q.w = cos(angle/2);
    q.x = sin(angle/2);
    q.y = 0;
    q.z = 0;
    return q;
  }
  if (axis == 1){
    Quaternion q;
    q.w = cos(angle/2);
    q.x = 0;
    q.y = sin(angle/2);
    q.z = 0;
    return q;
  }
  if (axis == 2){
    Quaternion q;
    q.w = cos(angle/2);
    q.x = 0;
    q.y = 0;
    q.z = sin(angle/2);
    return q;
  }
}

float Finger::getPitch(Quaternion q){
  return atan2(2 * (q.x * q.w - q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
}

float Finger::getYaw(Quaternion q){
  return atan2(2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q.y * q.y + q.z * q.z));
}

float Finger::getRoll(Quaternion q){
  return atan2(2 * (q.y * q.z + q.w * q.x), 1 - 2 * (q.x * q.x + q.y * q.y));
}

void Finger::computeToHandQuaternion(Quaternion relativeQuaternion, Quaternion sensorQuaternion){
    fingerQuaternion = quaternion_multiply(relativeQuaternion, sensorQuaternion);                     // rotate the sensorQuaternion to be in the coordinate frame where my calculations work
    toHandQuaternion = quaternion_multiply(handQuaternion, quaternion_conjugate(fingerQuaternion));   // get the relative quaternion between the reference IMU quaternion and the index IMU quaternion
}

void Finger::computeCurlDegrees(){
    curlRadians = getPitch(toHandQuaternion);  // get the curl angle in radians from the quaternion calculated above
    curlDegrees = toDegrees(curlRadians);          // convert the curl angle to degrees
}

void Finger::computeCurlAxis(){
    if(60 < curlDegrees && curlDegrees < 180) curlDegrees = -180;
    else if(0 < curlDegrees && curlDegrees <= 60) curlDegrees = 0;
    curlAxis = 1023. * (curlDegrees + 180)/180;
}

void Finger::computeSplayDegrees(){
    Quaternion curlQuaternion = quaternionFromAngle(curlRadians, 0);                                                      // create a quaternion that represents just the amount of curl in the x axis
    Quaternion decurledQuaternion = quaternion_multiply(curlQuaternion, fingerQuaternion);                                // rotate the indexQuaternion by the curl angle in the x axis
    Quaternion decurledToHandQuaternion = quaternion_multiply(handQuaternion, quaternion_conjugate(decurledQuaternion));  // get the relative quaternion between the reference IMU quaternion and the quaternion representing the index IMU rotated back by the curl angle
    splayRadians = getYaw(decurledToHandQuaternion);                                                                      // get the splay angle in radians from the quaternion calculated above
    splayDegrees = toDegrees(splayRadians);                                                                               // convert the splay angle to degrees
    // rotate everything 180 degrees
    if(splayDegrees > 0) splayDegrees = 180 - splayDegrees;
    else splayDegrees = - 180 - splayDegrees;
}

void Finger::computeSplayAxis(){
    if(splayDegrees <= splayDegreesMid) splayAxis = 512+512.*(splayDegrees-splayDegreesMid)/(splayDegreesMid-splayDegreesMin);
    else splayAxis = 512+512.*(splayDegrees-splayDegreesMid)/(splayDegreesMax-splayDegreesMid);
    if(splayAxis > 1023) splayAxis = 1023;
    else if(splayAxis < 0) splayAxis = 0;

    // scale the splay inversely by how much curl there is -> high curl, low splay
    // splayAxis -= 512;
    // splayAxis = splayAxis * (1023 - curlAxis)/1023.;
    // splayAxis += 512;

    if(curlAxis > 512){
      splayAxis -= 512;
      splayAxis = splayAxis * (1023 - curlAxis)/512.;
      splayAxis += 512;
    }
}

void Finger::computeAxesValues(Quaternion relativeQuaternion, Quaternion sensorQuaternion){
    computeToHandQuaternion(relativeQuaternion, sensorQuaternion);
    computeCurlDegrees();
    computeCurlAxis();
    computeSplayDegrees();
    computeSplayAxis();
}

// void Finger::calibrateCurlDegrees(float degreesMin, float degreesMid, float degreesMax){
//     curlDegreesMin = degreesMin;
//     curlDegreesMid = degreesMid;
//     curlDegreesMax = degreesMax;
// }

void Finger::calibrateSplayDegrees(float degreesMin, float degreesMid, float degreesMax){
    splayDegreesMin = degreesMin;
    splayDegreesMid = degreesMid;
    splayDegreesMax = degreesMax;
}