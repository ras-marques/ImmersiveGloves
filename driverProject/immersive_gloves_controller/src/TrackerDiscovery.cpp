#include "TrackerDiscovery.h"

#include <array>

#include "Hooks/InterfaceHookInjector.h"
//#include "Util/DriverLog.h"

static const std::string indexValueInputName = "/input/index/value";
static const std::string middleValueInputName = "/input/middle/value";
static const std::string ringValueInputName = "/input/ring/value";
static const std::string pinkyValueInputName = "/input/pinky/value";

std::string tundraTrackerLeftSerNum, tundraTrackerRightSerNum;

static std::array<std::string, 4> expectedInputNames = { indexValueInputName, middleValueInputName, ringValueInputName, pinkyValueInputName };

int TrackerDiscovery::FindTrackerDeviceIdByContainer(vr::PropertyContainerHandle_t ulContainer) {
  if (m_propertyContainerDeviceIdMap.count(ulContainer) > 0) return m_propertyContainerDeviceIdMap.at(ulContainer);

  vr::ETrackedPropertyError err;
  std::string manufacturer = vr::VRProperties()->GetStringProperty(ulContainer, vr::ETrackedDeviceProperty::Prop_ManufacturerName_String, &err);

  //char logstring[50] = {};
  //sprintf_s(logstring, "manufacturer %s", manufacturer);
  vr::VRDriverLog()->Log(manufacturer.c_str());

  if (manufacturer != "Immersive Devices") return -1;  // only look for Immersive Gloves Controllers

  for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
    if (vr::VRProperties()->TrackedDeviceToPropertyContainer(i) == ulContainer) {
      //DriverLog("Etee tracker discovered with id: %i", i);
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
    vr::VRDriverLog()->Log(pchName);
    if (std::find(expectedInputNames.begin(), expectedInputNames.end(), std::string(pchName)) != expectedInputNames.end()) {
        int deviceId = FindTrackerDeviceIdByContainer(ulContainer);


        if (deviceId != -1) {
            m_inputComponentDeviceIdMap.insert_or_assign(*pHandle, InputComponentInfo(std::string(pchName), deviceId));
        }
    }
}

void TrackerDiscovery::UpdateHandSerialNumber(bool isRight, std::string serialNum) {
  if (isRight) {
    tundraTrackerRightSerNum = serialNum;
  } else {
    tundraTrackerLeftSerNum = serialNum;
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
    //DriverLog(
    //    "Tracker was updated: Id: %i, Serial Number: %s, Name: %s, Value: %s",
    //    inputInfo.deviceId,
    //    deviceSerialNumber.c_str(),
    //    inputInfo.name.c_str(),
    //    bNewValue ? "True" : "False");

    if (!m_trackerIdStatus.count(inputInfo.deviceId)) m_trackerIdStatus[inputInfo.deviceId] = {};

    TrackerStatus& status = m_trackerIdStatus.at(inputInfo.deviceId);

    // If the input being read is isRightHanded
    //if (inputInfo.name == isRightHandedInputName) {
    //  status.role = bNewValue ? vr::TrackedControllerRole_RightHand : vr::TrackedControllerRole_LeftHand;

    //  bool isRightHandedVal = bNewValue ? true : false;
    //  UpdateHandSerialNumber(isRightHandedVal, deviceSerialNumber);  // Update serial number of handed trackers
    //}

    // If the input being read is trackerConnection
    //if (inputInfo.name == trackerConnectionInputName) {
    //  status.trackerConnected = bNewValue;
    //}

    inputInfo.lastValue = bNewValue;

    if (!status.trackerConnected || !status.role) return;  // only update state if a tracker is connected

    m_callback(status.role, inputInfo.deviceId);
  }
}

// This function only runs when the controller is connected to the tracker,
// and the controller data is sent through SPI to the tracker and subsequently to SteamVR
void TrackerDiscovery::UpdateScalarComponent(vr::VRInputComponentHandle_t ulComponent, float fNewValue, double fTimeOffset) {
    if (m_inputComponentDeviceIdMap.count(ulComponent) > 0) {

        char logstring[50] = {};
        if ((int)(ulComponent) == 1) sprintf_s(logstring, "index %f ", fNewValue);
        else if ((int)(ulComponent) == 2) sprintf_s(logstring, "middle %f ", fNewValue);
        else if ((int)(ulComponent) == 3) sprintf_s(logstring, "ring %f ", fNewValue);
        else if ((int)(ulComponent) == 4) sprintf_s(logstring, "pinky %f ", fNewValue);
        vr::VRDriverLog()->Log(logstring);

        InputComponentInfo& inputInfo = m_inputComponentDeviceIdMap.at(ulComponent);
        vr::VRDriverLog()->Log(inputInfo.name.c_str()); //this prints /input/index/value, /input/middle/value, etc

        // Check the device serial number
        vr::PropertyContainerHandle_t container = vr::VRProperties()->TrackedDeviceToPropertyContainer(inputInfo.deviceId);
        std::string deviceSerialNumber = vr::VRProperties()->GetStringProperty(container, vr::ETrackedDeviceProperty::Prop_SerialNumber_String);
        int role = vr::VRProperties()->GetInt32Property(container, vr::ETrackedDeviceProperty::Prop_ControllerRoleHint_Int32);
        //this will be 1 if left, 2 if right
        
        struct InputData {
            float flexion[5][4];
            float splay[5];
            float joyX;
            float joyY;
            bool joyButton;
            bool trgButton;
            bool aButton;
            bool bButton;
            bool grab;
            bool pinch;
            bool menu;
            bool calibrate;

            float trgValue;
        };
        float flex = 0.5;
        InputData leftData;
        leftData.flexion[0][0] = 0; leftData.flexion[0][1] = 0; leftData.flexion[0][2] = 0; leftData.flexion[0][3] = 0;
        leftData.flexion[1][0] = flex; leftData.flexion[1][1] = flex; leftData.flexion[1][2] = flex; leftData.flexion[1][3] = flex;
        leftData.flexion[2][0] = flex; leftData.flexion[2][1] = flex; leftData.flexion[2][2] = flex; leftData.flexion[2][3] = flex;
        leftData.flexion[3][0] = 0; leftData.flexion[3][1] = 0; leftData.flexion[3][2] = 0; leftData.flexion[3][3] = 0;
        leftData.flexion[4][0] = 0; leftData.flexion[4][1] = 0; leftData.flexion[4][2] = 0; leftData.flexion[4][3] = 0;
        leftData.splay[0] = 0;
        leftData.splay[1] = 0;
        leftData.splay[2] = 0;
        leftData.splay[3] = 0;
        leftData.splay[4] = 0;
        leftData.joyX = 0;
        leftData.joyY = 0;
        leftData.joyButton;
        leftData.trgButton;
        leftData.aButton = false;
        leftData.bButton = false;
        leftData.bButton = false;
        leftData.grab = false;
        leftData.pinch = false;
        leftData.menu = false;
        leftData.calibrate = false;
        leftData.trgValue = false;

        HANDLE hPipeLeft = CreateFile(TEXT("\\\\.\\pipe\\vrapplication\\input\\glove\\v2\\left"),
            GENERIC_READ | GENERIC_WRITE,
            0,
            NULL,
            OPEN_EXISTING,
            0,
            NULL);
        DWORD dwWritten;
        WriteFile(hPipeLeft,
            &leftData,
            sizeof(InputData),
            &dwWritten,
            NULL);

        if (!m_trackerIdStatus.count(inputInfo.deviceId)) m_trackerIdStatus[inputInfo.deviceId] = {}; // I don't know what this does...

        TrackerStatus& status = m_trackerIdStatus.at(inputInfo.deviceId);
        if (role == 1) {
            status.role = vr::TrackedControllerRole_LeftHand;
            UpdateHandSerialNumber(false, deviceSerialNumber);  // Update serial number of handed trackers
        }
        else if (role == 2) {
            status.role = vr::TrackedControllerRole_RightHand;
            UpdateHandSerialNumber(true, deviceSerialNumber);  // Update serial number of handed trackers
        }

        m_callback(status.role, inputInfo.deviceId);
    }
}