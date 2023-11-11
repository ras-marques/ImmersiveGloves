#pragma once

#include <array>
#include "openvr_driver.h"
#include <windows.h>

enum InputHandles {
	kInputHandle_index_value,
	kInputHandle_middle_value,
	kInputHandle_ring_value,
	kInputHandle_pinky_value,
	kInputHandle_COUNT
};

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

class ControllerDevice : public vr::ITrackedDeviceServerDriver {
public:
	ControllerDevice(vr::ETrackedControllerRole role);

	// Inherited via ITrackedDeviceServerDriver
	virtual vr::EVRInitError Activate(uint32_t unObjectId) override;
	virtual void Deactivate() override;
	virtual void EnterStandby() override;
	virtual void* GetComponent(const char* pchComponentNameAndVersion) override;
	virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override;
	virtual vr::DriverPose_t GetPose() override;

	void RunFrame();
	void HandleEvent(const vr::VREvent_t& vrevent);
	void SetTrackerId(short deviceId, bool isRightHand);
	vr::ETrackedControllerRole GetDeviceRole() const;
	void WritePipe();

	InputData leftData, rightData;

private:
	std::array<vr::VRInputComponentHandle_t, kInputHandle_COUNT> input_handles_;

	vr::ETrackedControllerRole role_;
	vr::TrackedDeviceIndex_t device_id_;

	HANDLE hPipeLeft, hPipeRight;
	DWORD dwWritten;
};