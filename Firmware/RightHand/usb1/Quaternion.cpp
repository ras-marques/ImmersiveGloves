#include "Quaternion.h"


float getPitch_pry(Quaternion q){
  return asin(2 * (q.x * q.z - q.w * q.y));
}
float getRoll_pry(Quaternion q){
  return atan2(2 * (q.y * q.z + q.w * q.x), (q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z));
}
float getYaw_pry(Quaternion q){
  return atan2(2 * (q.x * q.y + q.w * q.z), (q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z));
}

float getPitch_ypr(Quaternion q){
  return asin(-2 * (q.x * q.z - q.w * q.y));
}
float getRoll_ypr(Quaternion q){
  return atan2(2 * (q.w * q.z + q.x * q.y), (q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z));
}
float getYaw_ypr(Quaternion q){
  return atan2(2 * (q.w * q.x + q.y * q.z), (q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z));
}

float getRoll_ryp(Quaternion q){
  return asin(-2 * (q.x * q.y - q.w * q.z));
}
float getYaw_ryp(Quaternion q){
  return atan2(2 * (q.w * q.y + q.x * q.z), (q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z));
}
float getPitch_ryp(Quaternion q){
  return atan2(2 * (q.w * q.x + q.y * q.z), (q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z));
}

Quaternion quaternion_conjugate(Quaternion q){
  Quaternion output;
  output.w = q.w;
  output.x = -q.x;
  output.y = -q.y;
  output.z = -q.z;
  return output;
}

Quaternion quaternion_multiply(Quaternion q1, Quaternion q2){
  float w1 = q1.w;
  float x1 = q1.x;
  float y1 = q1.y;
  float z1 = q1.z;
  
  float w2 = q2.w;
  float x2 = q2.x;
  float y2 = q2.y;
  float z2 = q2.z;

  // Calculate the resulting quaternion components
  Quaternion output;
  output.w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
  output.x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
  output.y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
  output.z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
  return output;
}

Quaternion::Quaternion(){}

void Quaternion::printMyEulerAngles_pry(){
  Quaternion self;
  self.w = w;
  self.x = x;
  self.y = y;
  self.z = z;
  Serial.print("pyr: p ");
  Serial.print(getPitch_pry(self)*180/3.14);
  Serial.print("\tr ");
  Serial.print(getRoll_pry(self)*180/3.14);
  Serial.print("\ty ");
  Serial.println(getYaw_pry(self)*180/3.14);
}

void Quaternion::printMyEulerAngles_ypr(){
  Quaternion self;
  self.w = w;
  self.x = x;
  self.y = y;
  self.z = z;
  Serial.print("ypr: p ");
  Serial.print(getPitch_ypr(self)*180/3.14);
  Serial.print("\tr ");
  Serial.print(getRoll_ypr(self)*180/3.14);
  Serial.print("\ty ");
  Serial.println(getYaw_ypr(self)*180/3.14);
}

void Quaternion::printMyEulerAngles_ryp(){
  Quaternion self;
  self.w = w;
  self.x = x;
  self.y = y;
  self.z = z;
  Serial.print("ryp: p ");
  Serial.print(getPitch_ryp(self)*180/3.14);
  Serial.print("\tr ");
  Serial.print(getRoll_ryp(self)*180/3.14);
  Serial.print("\ty ");
  Serial.println(getYaw_ryp(self)*180/3.14);
}

void Quaternion::printMe(){
  Serial.print(w);
  Serial.print(",");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(",");
  Serial.println(z);
}

Quaternion Quaternion::rotateBy(Quaternion q){
  Quaternion self;
  self.w = w;
  self.x = x;
  self.y = y;
  self.z = z;
  return quaternion_multiply(q, self);
}

Quaternion Quaternion::getRelativeTo(Quaternion q){
  Quaternion self;
  self.w = w;
  self.x = x;
  self.y = y;
  self.z = z;
  return quaternion_multiply(q, quaternion_conjugate(self));
}
