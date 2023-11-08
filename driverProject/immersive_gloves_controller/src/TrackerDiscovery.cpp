#include "TrackerDiscovery.h"

#include <array>

#include "Hooks/InterfaceHookInjector.h"
#include "Util/DriverLog.h"

static const std::string isRightHandedInputName = "/input/is_right_handed/click";
static const std::string trackerConnectionInputName = "/input/tracker_connection/click";

std::string eteeTrackerLeftSerNum, eteeTrackerRightSerNum;

static std::array<std::string, 2> expectedInputNames = {isRightHandedInputName, trackerConnectionInputName};

int TrackerDiscovery::FindTrackerDeviceIdByContainer(vr::PropertyContainerHandle_t ulContainer) {
  if (m_propertyContainerDeviceIdMap.count(ulContainer) > 0) return m_propertyContainerDeviceIdMap.at(ulContainer);

  vr::ETrackedPropertyError err;
  std::string manufacturer = vr::VRProperties()->GetStringProperty(ulContainer, vr::ETrackedDeviceProperty::Prop_ManufacturerName_String, &err);

  if (manufacturer != "TG0") return -1;  // only look for etee trackers

  for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
    if (vr::VRProperties()->TrackedDeviceToPropertyContainer(i) == ulContainer) {
      DriverLog("Etee tracker discovered with id: %i", i);
      m_propertyContainerDeviceIdMap.insert({ulContainer, i});
      return i;
    }
  }

  return -1;
}

void TrackerDiscovery::StartDiscovery(std::function<void(vr::ETrackedControllerRole role, int deviceId)> callback) {
  m_callback = callback;

  InjectHooks(this, m_context);

  m_active = true;
}

void TrackerDiscovery::TrackedDeviceAdded(const char* pchDeviceSerialNumber, vr::ETrackedDeviceClass eDeviceClass, vr::ITrackedDeviceServerDriver* pDriver) {}

void TrackerDiscovery::CreateBooleanComponent(vr::PropertyContainerHandle_t ulContainer, const char* pchName, vr::VRInputComponentHandle_t* pHandle) {
  if (std::find(expectedInputNames.begin(), expectedInputNames.end(), std::string(pchName)) != expectedInputNames.end()) {
    int deviceId = FindTrackerDeviceIdByContainer(ulContainer);
    if (deviceId != -1) {
      m_inputComponentDeviceIdMap.insert_or_assign(*pHandle, InputComponentInfo(std::string(pchName), deviceId));
    }
  }
}

void TrackerDiscovery::CreateScalarComponent(vr::PropertyContainerHandle_t ulContainer, const char* pchName, vr::VRInputComponentHandle_t* pHandle) {
    if (std::find(expectedInputNames.begin(), expectedInputNames.end(), std::string(pchName)) != expectedInputNames.end()) {
        int deviceId = FindTrackerDeviceIdByContainer(ulContainer);
        if (deviceId != -1) {
            m_inputComponentDeviceIdMap.insert_or_assign(*pHandle, InputComponentInfo(std::string(pchName), deviceId));
        }
    }
}

void TrackerDiscovery::UpdateHandSerialNumber(bool isRight, std::string serialNum) {
  if (isRight) {
    eteeTrackerRightSerNum = serialNum;
  } else {
    eteeTrackerLeftSerNum = serialNum;
  } 
}

// This function only runs when the controller is connected to the tracker,
// and the controller data is sent through SPI to the tracker and subsequently to SteamVR
void TrackerDiscovery::UpdateBooleanComponent(vr::VRInputComponentHandle_t ulComponent, bool bNewValue, double fTimeOffset) {
  if (m_inputComponentDeviceIdMap.count(ulComponent) > 0) {
    InputComponentInfo& inputInfo = m_inputComponentDeviceIdMap.at(ulComponent);

    // Check the device serial number
    vr::PropertyContainerHandle_t container = vr::VRProperties()->TrackedDeviceToPropertyContainer(inputInfo.deviceId);
    std::string deviceSerialNumber = vr::VRProperties()->GetStringProperty(container, vr::ETrackedDeviceProperty::Prop_SerialNumber_String);

    // If the tracker data values have changed
    DriverLog(
        "Tracker was updated: Id: %i, Serial Number: %s, Name: %s, Value: %s",
        inputInfo.deviceId,
        deviceSerialNumber.c_str(),
        inputInfo.name.c_str(),
        bNewValue ? "True" : "False");

    if (!m_trackerIdStatus.count(inputInfo.deviceId)) m_trackerIdStatus[inputInfo.deviceId] = {};

    TrackerStatus& status = m_trackerIdStatus.at(inputInfo.deviceId);

    // If the input being read is isRightHanded
    if (inputInfo.name == isRightHandedInputName) {
      status.role = bNewValue ? vr::TrackedControllerRole_RightHand : vr::TrackedControllerRole_LeftHand;

      bool isRightHandedVal = bNewValue ? true : false;
      UpdateHandSerialNumber(isRightHandedVal, deviceSerialNumber);  // Update serial number of handed trackers
    }

    // If the input being read is trackerConnection
    if (inputInfo.name == trackerConnectionInputName) {
      status.trackerConnected = bNewValue;
    }

    inputInfo.lastValue = bNewValue;

    if (!status.trackerConnected || !status.role) return;  // only update state if a tracker is connected

    m_callback(status.role, inputInfo.deviceId);
  }
}

// This function only runs when the controller is connected to the tracker,
// and the controller data is sent through SPI to the tracker and subsequently to SteamVR
void TrackerDiscovery::UpdateScalarComponent(vr::VRInputComponentHandle_t ulComponent, float fNewValue, double fTimeOffset) {
    if (m_inputComponentDeviceIdMap.count(ulComponent) > 0) {
        InputComponentInfo& inputInfo = m_inputComponentDeviceIdMap.at(ulComponent);

        // Check the device serial number
        vr::PropertyContainerHandle_t container = vr::VRProperties()->TrackedDeviceToPropertyContainer(inputInfo.deviceId);
        std::string deviceSerialNumber = vr::VRProperties()->GetStringProperty(container, vr::ETrackedDeviceProperty::Prop_SerialNumber_String);

        // If the tracker data values have changed
        DriverLog(
            "Tracker was updated: Id: %i, Serial Number: %s, Name: %s, Value: %f",
            inputInfo.deviceId,
            deviceSerialNumber.c_str(),
            inputInfo.name.c_str(),
            fNewValue);

        if (!m_trackerIdStatus.count(inputInfo.deviceId)) m_trackerIdStatus[inputInfo.deviceId] = {};

        //TrackerStatus& status = m_trackerIdStatus.at(inputInfo.deviceId);

        // If the input being read is isRightHanded
        //if (inputInfo.name == isRightHandedInputName) {
        //    status.role = bNewValue ? vr::TrackedControllerRole_RightHand : vr::TrackedControllerRole_LeftHand;

        //    bool isRightHandedVal = bNewValue ? true : false;
        //    UpdateHandSerialNumber(isRightHandedVal, deviceSerialNumber);  // Update serial number of handed trackers
        //}

        //// If the input being read is trackerConnection
        //if (inputInfo.name == trackerConnectionInputName) {
        //    status.trackerConnected = bNewValue;
        //}

        //inputInfo.lastValue = bNewValue;

        //if (!status.trackerConnected || !status.role) return;  // only update state if a tracker is connected

        //m_callback(status.role, inputInfo.deviceId);
    }
}