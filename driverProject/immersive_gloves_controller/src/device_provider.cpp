#include "device_provider.h"

vr::EVRInitError DeviceProvider::Init(vr::IVRDriverContext* pDriverContext) {
    VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
    vr::VRDriverLog()->Log("Hello world!");

    my_left_device_ = std::make_unique<ControllerDevice>(vr::TrackedControllerRole_LeftHand);
    vr::VRServerDriverHost()->TrackedDeviceAdded("IMMRSV-GLV-LEFT",
        vr::TrackedDeviceClass_Controller,
        my_left_device_.get());

    my_right_device_ = std::make_unique<ControllerDevice>(vr::TrackedControllerRole_RightHand);
    vr::VRServerDriverHost()->TrackedDeviceAdded("IMMRSV-GLV-RIGHT",
        vr::TrackedDeviceClass_Controller,
        my_right_device_.get());
    
    my_left_device_->hPipe = CreateFile(TEXT("\\\\.\\pipe\\vrapplication\\input\\glove\\v2\\left"),
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        0,
        NULL);

    my_right_device_->hPipe = CreateFile(TEXT("\\\\.\\pipe\\vrapplication\\input\\glove\\v2\\right"),
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        0,
        NULL);

    /*my_left_device_->PrintDeviceId();
    my_right_device_->PrintDeviceId();*/

    m_trackerDiscovery = std::make_unique<TrackerDiscovery>(pDriverContext);
    m_trackerDiscovery->StartDiscovery([&](vr::ETrackedControllerRole role, int deviceId, std::string inputName, float data) {
        //vr::VRDriverLog()->Log("Callback!");
        //my_left_device_->data.splay[0] = 1.;
        if (role == 1) {
            //vr::VRDriverLog()->Log(inputName.c_str());
            if (inputName == "/input/thumb/value") {
                my_left_device_->data.flexion[0][0] = data;
                my_left_device_->data.flexion[0][1] = data;
                my_left_device_->data.flexion[0][2] = data;
                my_left_device_->data.flexion[0][3] = data;
            }
            else if (inputName == "/input/index/value") {
                my_left_device_->data.flexion[1][0] = data;
                my_left_device_->data.flexion[1][1] = data;
                my_left_device_->data.flexion[1][2] = data;
                my_left_device_->data.flexion[1][3] = data;
            }
            else if (inputName == "/input/middle/value") {
                my_left_device_->data.flexion[2][0] = data;
                my_left_device_->data.flexion[2][1] = data;
                my_left_device_->data.flexion[2][2] = data;
                my_left_device_->data.flexion[2][3] = data;
            }
            else if (inputName == "/input/ring/value") {
                my_left_device_->data.flexion[3][0] = data;
                my_left_device_->data.flexion[3][1] = data;
                my_left_device_->data.flexion[3][2] = data;
                my_left_device_->data.flexion[3][3] = data;
            }
            else if (inputName == "/input/pinky/value") {
                my_left_device_->data.flexion[4][0] = data;
                my_left_device_->data.flexion[4][1] = data;
                my_left_device_->data.flexion[4][2] = data;
                my_left_device_->data.flexion[4][3] = data;
            }
            else if (inputName == "/input/thumbsplay/value") {
                my_left_device_->data.splay[0] = (float)(2 * (data - 0.5));
            }
            else if (inputName == "/input/indexsplay/value") {
                my_left_device_->data.splay[1] = (float)(2 * (data - 0.5));
            }
            else if (inputName == "/input/middlesplay/value") {
                //char logstring[50] = {};
                //sprintf_s(logstring, "middlesplay: %f", (float)(2 * (data - 0.5)));
                //vr::VRDriverLog()->Log(logstring);
                my_left_device_->data.splay[2] = (float)(2 * (data - 0.5));
            }
            else if (inputName == "/input/ringsplay/value") {
                my_left_device_->data.splay[3] = (float)(2 * (data - 0.5));
            }
            else if (inputName == "/input/pinkysplay/value") {
                my_left_device_->data.splay[4] = (float)(2 * (data - 0.5));
            }
            else if (inputName == "/input/thumbstickx/value") {
                my_left_device_->data.joyX = (float)(2 * (data - 0.5));
            }
            else if (inputName == "/input/thumbsticky/value") {
                my_left_device_->data.joyY = (float)(2 * (data - 0.5));
            }
            else if (inputName == "/input/thumbstick/click") {
                my_left_device_->data.joyButton = (bool)(data);
            }
            else if (inputName == "/input/a/click") {
                my_left_device_->data.aButton = (bool)(data);
            }
            else if (inputName == "/input/b/click") {
                my_left_device_->data.bButton = (bool)(data);
            }
            else if (inputName == "/input/system/click") {
                my_left_device_->data.menu = (bool)(data);
            }
            my_left_device_->WritePipe();
        }
        else if (role == 2) {
            if (inputName == "/input/thumb/value") {
                my_right_device_->data.flexion[0][0] = data;
                my_right_device_->data.flexion[0][1] = data;
                my_right_device_->data.flexion[0][2] = data;
                my_right_device_->data.flexion[0][3] = data;
            }
            else if (inputName == "/input/index/value") {
                my_right_device_->data.flexion[1][0] = data;
                my_right_device_->data.flexion[1][1] = data;
                my_right_device_->data.flexion[1][2] = data;
                my_right_device_->data.flexion[1][3] = data;
            }
            else if (inputName == "/input/middle/value") {
                my_right_device_->data.flexion[2][0] = data;
                my_right_device_->data.flexion[2][1] = data;
                my_right_device_->data.flexion[2][2] = data;
                my_right_device_->data.flexion[2][3] = data;
            }
            else if (inputName == "/input/ring/value") {
                my_right_device_->data.flexion[3][0] = data;
                my_right_device_->data.flexion[3][1] = data;
                my_right_device_->data.flexion[3][2] = data;
                my_right_device_->data.flexion[3][3] = data;
            }
            else if (inputName == "/input/pinky/value") {
                my_right_device_->data.flexion[4][0] = data;
                my_right_device_->data.flexion[4][1] = data;
                my_right_device_->data.flexion[4][2] = data;
                my_right_device_->data.flexion[4][3] = data;
            }
            else if (inputName == "/input/thumbsplay/value") {
                my_right_device_->data.splay[0] = (float)(2 * (data - 0.5));
            }
            else if (inputName == "/input/indexsplay/value") {
                my_right_device_->data.splay[1] = (float)(2 * (data - 0.5));
            }
            else if (inputName == "/input/middlesplay/value") {
                my_right_device_->data.splay[2] = (float)(2 * (data - 0.5));
            }
            else if (inputName == "/input/ringsplay/value") {
                my_right_device_->data.splay[3] = (float)(2 * (data - 0.5));
            }
            else if (inputName == "/input/pinkysplay/value") {
                my_right_device_->data.splay[4] = (float)(2 * (data - 0.5));
            }
            else if (inputName == "/input/thumbstickx/value") {
                my_right_device_->data.joyX = (float)(2 * (data - 0.5));
            }
            else if (inputName == "/input/thumbsticky/value") {
                my_right_device_->data.joyY = (float)(2 * (data - 0.5));
            }
            else if (inputName == "/input/thumbstick/click") {
                my_right_device_->data.joyButton = (bool)(data);
            }
            else if (inputName == "/input/a/click") {
                my_right_device_->data.aButton = (bool)(data);
            }
            else if (inputName == "/input/b/click") {
                my_right_device_->data.bButton = (bool)(data);
            }
            else if (inputName == "/input/system/click") {
                my_right_device_->data.menu = (bool)(data);
            }
            my_right_device_->WritePipe();
        }

        if (my_left_device_ != nullptr && my_left_device_->GetDeviceRole() == role) {
            my_left_device_->SetTrackerId(deviceId, false);
        }

        if (my_right_device_ != nullptr && my_right_device_->GetDeviceRole() == role) {
            my_right_device_->SetTrackerId(deviceId, true);
        }
    });

    return vr::VRInitError_None;
}

void DeviceProvider::Cleanup() {
    VR_CLEANUP_SERVER_DRIVER_CONTEXT();
}

const char* const* DeviceProvider::GetInterfaceVersions() {
    return vr::k_InterfaceVersions;
}

void DeviceProvider::RunFrame() {
    vr::VREvent_t vrevent;
    while (vr::VRServerDriverHost()->PollNextEvent(&vrevent, sizeof(vrevent))) {
        my_left_device_->HandleEvent(vrevent);
        my_right_device_->HandleEvent(vrevent);
    }

    if (my_left_device_ != nullptr) {
        my_left_device_->RunFrame();
    }

    if (my_right_device_ != nullptr) {
        my_right_device_->RunFrame();
    }
}

bool DeviceProvider::ShouldBlockStandbyMode() {
    return false;
}

void DeviceProvider::EnterStandby() {

}

void DeviceProvider::LeaveStandby() {

}