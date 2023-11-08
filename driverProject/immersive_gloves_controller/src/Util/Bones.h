#pragma once

#include <array>
#include <memory>

#include "Encode/EncodingManager.h"
#include "Util/AnimLoader.h"
#include "openvr_driver.h"

#include <thread>

const short NUM_BONES = static_cast<short>(HandSkeletonBone::_Count);

class BoneAnimator {
 public:
  explicit BoneAnimator(const std::string& curlAnimFileName, const std::string& accessoryAnimFileName);
  void ComputeSkeletonTransforms(vr::VRBoneTransform_t* skeleton, const VRCommInputData_t& inputData);
  void LoadDefaultSkeletonByHand(vr::VRBoneTransform_t* skeleton, const bool rightHand);
  void LoadGripLimitSkeletonByHand(vr::VRBoneTransform_t* skeleton, const bool rightHand);
  float RemapValue(float low1, float high1, float low2, float high2, float value1);
  float RemapCurl(float pull, float force, float pullThreshold);
  std::pair<std::pair<int, int>, float> TrackpadFindFrame(
      float x, float y, float norm_x, float norm_y, float lowerCenterThreshold, float upperCenterThreshold, bool trackpadClick, bool isRightHand);

 private:
  void ApplyTransformForBone(
      vr::VRBoneTransform_t& bone, const HandSkeletonBone& boneIndex, const AnimationData& animationData, const float interp, const bool rightHand) const;

  std::unique_ptr<IModelManager> curlModelManager_;
  std::unique_ptr<IModelManager> accessoryModelManager_;
  
  const float touchThreshold = 0.65f;
  const float lowerCenterThreshold = 0.25f;
  const float upperCenterThreshold = 0.75f;
  const float proximityValThreshold = 0.1f;

  bool loaded_;
  std::vector<float> keyframes_;

  //motion smoothing
  std::array<float, NUM_BONES> accumulator_;
};