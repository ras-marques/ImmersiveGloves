#pragma once

#include <atomic>
#include <functional>
#include <map>
#include <memory>

#include "Hooks/HookReceiver.h"
#include "openvr_driver.h"

struct TrackerStatus {
  bool trackerConnected;
  vr::ETrackedControllerRole role;
};

struct InputComponentInfo {
  InputComponentInfo() : valid(false), deviceId(-1), name(""), lastValue(-1){};
  InputComponentInfo(std::string name, int deviceId) : name(name), deviceId(deviceId), lastValue(-1), valid(true){};

  int deviceId;
  std::string name;
  int lastValue;
  bool valid;
};

class TrackerDiscovery : IHookReceiver {
 public:
  TrackerDiscovery(vr::IVRDriverContext *context) : m_context(context){};

  void StartDiscovery(std::function<void(vr::ETrackedControllerRole role, int deviceId)> callback);

  void CreateBooleanComponent(vr::PropertyContainerHandle_t ulContainer, const char *pchName, vr::VRInputComponentHandle_t *pHandle) override;
  void UpdateBooleanComponent(vr::VRInputComponentHandle_t ulComponent, bool bNewValue, double fTimeOffset) override;
  void CreateScalarComponent(vr::PropertyContainerHandle_t ulContainer, const char* pchName, vr::VRInputComponentHandle_t* pHandle) override;
  void UpdateScalarComponent(vr::VRInputComponentHandle_t ulComponent, float fNewValue, double fTimeOffset) override;
  void TrackedDeviceAdded(const char *pchDeviceSerialNumber, vr::ETrackedDeviceClass eDeviceClass, vr::ITrackedDeviceServerDriver *pDriver) override;

 private:
  int FindTrackerDeviceIdByContainer(vr::PropertyContainerHandle_t ulContainer);
  void UpdateHandSerialNumber(bool isRight, std::string serialNum);

  std::function<void(vr::ETrackedControllerRole role, int deviceId)> m_callback;
  vr::IVRDriverContext *m_context;

  std::map<vr::VRInputComponentHandle_t, InputComponentInfo> m_inputComponentDeviceIdMap;
  std::map<vr::PropertyContainerHandle_t, int> m_propertyContainerDeviceIdMap;
  std::map<int, TrackerStatus> m_trackerIdStatus;

  std::atomic<bool> m_active;
};