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
    m_trackerDiscovery->StartDiscovery([&](vr::ETrackedControllerRole role, int deviceId, int componentIndex, float data) {
        //vr::VRDriverLog()->Log("Callback!");
        if (role == 1) {
            float flex = data;
            if (componentIndex == 1) {
                my_left_device_->data.flexion[1][0] = flex;
                my_left_device_->data.flexion[1][1] = flex;
                my_left_device_->data.flexion[1][2] = flex;
                my_left_device_->data.flexion[1][3] = flex;
            }
            else if (componentIndex == 2) {
                my_left_device_->data.flexion[2][0] = flex;
                my_left_device_->data.flexion[2][1] = flex;
                my_left_device_->data.flexion[2][2] = flex;
                my_left_device_->data.flexion[2][3] = flex;
            }
            else if (componentIndex == 3) {
                my_left_device_->data.flexion[3][0] = flex;
                my_left_device_->data.flexion[3][1] = flex;
                my_left_device_->data.flexion[3][2] = flex;
                my_left_device_->data.flexion[3][3] = flex;
            }
            else if (componentIndex == 4) {
                my_left_device_->data.flexion[4][0] = flex;
                my_left_device_->data.flexion[4][1] = flex;
                my_left_device_->data.flexion[4][2] = flex;
                my_left_device_->data.flexion[4][3] = flex;
            }
            my_left_device_->WritePipe();
        }
        else if (role == 2) {
            float flex = data;
            if (componentIndex == 1) {
                my_right_device_->data.flexion[1][0] = flex;
                my_right_device_->data.flexion[1][1] = flex;
                my_right_device_->data.flexion[1][2] = flex;
                my_right_device_->data.flexion[1][3] = flex;
            }
            else if (componentIndex == 2) {
                my_right_device_->data.flexion[2][0] = flex;
                my_right_device_->data.flexion[2][1] = flex;
                my_right_device_->data.flexion[2][2] = flex;
                my_right_device_->data.flexion[2][3] = flex;
            }
            else if (componentIndex == 3) {
                my_right_device_->data.flexion[3][0] = flex;
                my_right_device_->data.flexion[3][1] = flex;
                my_right_device_->data.flexion[3][2] = flex;
                my_right_device_->data.flexion[3][3] = flex;
            }
            else if (componentIndex == 4) {
                my_right_device_->data.flexion[4][0] = flex;
                my_right_device_->data.flexion[4][1] = flex;
                my_right_device_->data.flexion[4][2] = flex;
                my_right_device_->data.flexion[4][3] = flex;
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