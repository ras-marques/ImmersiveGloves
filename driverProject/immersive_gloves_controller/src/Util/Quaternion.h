#define _USE_MATH_DEFINES

#include "openvr_driver.h"

double DegToRad(int degrees);
double RadToDeg(double rad);

// get the quaternion for roation from a matrix
vr::HmdQuaternion_t GetRotation(const vr::HmdMatrix34_t& matrix);
vr::HmdVector3_t GetPosition(const vr::HmdMatrix34_t& matrix);
vr::HmdVector3_t CombinePosition(const vr::HmdMatrix34_t& matrix, const vr::HmdVector3_t& vec);

// returns the result of multiplying two quaternions, effectively applying a roatation on a quaternion
vr::HmdQuaternion_t MultiplyQuaternion(const vr::HmdQuaternion_t& q, const vr::HmdQuaternion_t& r);

vr::HmdQuaternion_t EulerToQuaternion(double roll, double pitch, double yaw);
vr::HmdMatrix33_t GetRotationMatrix(const vr::HmdMatrix34_t& matrix);
vr::HmdVector3_t MultiplyMatrix(const vr::HmdMatrix33_t& matrix, const vr::HmdVector3_t& vector);
vr::HmdQuaternion_t InverseQuaternion(const vr::HmdQuaternion_t& q);
vr::HmdQuaternion_t EulerToQuaternionYawRollPitch(const vr::HmdVector3_t euler);
vr::HmdVector3_t QuaternionToEulerYawRollPitch(const vr::HmdQuaternion_t q);

vr::HmdQuaternion_t operator-(const vr::HmdQuaternion_t& q);

vr::HmdQuaternion_t operator*(const vr::HmdQuaternion_t& lhs, const vr::HmdQuaternion_t& rhs);
vr::HmdVector3_t operator+(const vr::HmdMatrix34_t& matrix, const vr::HmdVector3_t& vec);
vr::HmdVector3_t operator*(const vr::HmdMatrix33_t& matrix, const vr::HmdVector3_t& vec);
vr::HmdVector3_t operator-(const vr::HmdVector3_t& vec, const vr::HmdMatrix34_t& matrix);
vr::HmdVector3_t operator+(const vr::HmdVector3_t& vec1, const vr::HmdVector3_t& vec2);
vr::HmdVector3d_t operator+(const vr::HmdVector3d_t& vec1, const vr::HmdVector3d_t& vec2);
vr::HmdVector3d_t operator-(const vr::HmdVector3d_t& vec1, const vr::HmdVector3d_t& vec2);
vr::HmdVector3d_t operator*(const vr::HmdVector3d_t& vec, const vr::HmdQuaternion_t& q);
vr::HmdVector3_t operator*(const vr::HmdVector3_t& vec, const vr::HmdQuaternion_t& q);

bool operator==(const vr::HmdVector3d_t& v1, const vr::HmdVector3d_t& v2);
bool operator==(const vr::HmdQuaternion_t& q1, const vr::HmdQuaternion_t& q2);