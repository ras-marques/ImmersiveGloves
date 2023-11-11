#include "controller_device.h"

ControllerDevice::ControllerDevice(vr::ETrackedControllerRole role) : role_(role), device_id_(vr::k_unTrackedDeviceIndexInvalid) {};

vr::EVRInitError ControllerDevice::Activate(uint32_t unObjectId) {
	vr::VRDriverLog()->Log("ControllerDevice::Activate");

	//hPipeLeft = CreateFile(TEXT("\\\\.\\pipe\\vrapplication\\input\\glove\\v2\\left"),
	//	GENERIC_READ | GENERIC_WRITE,
	//	0,
	//	NULL,
	//	OPEN_EXISTING,
	//	0,
	//	NULL);
	//hPipeRight = CreateFile(TEXT("\\\\.\\pipe\\vrapplication\\input\\glove\\v2\\right"),
	//	GENERIC_READ | GENERIC_WRITE,
	//	0,
	//	NULL,
	//	OPEN_EXISTING,
	//	0,
	//	NULL);

	//const vr::PropertyContainerHandle_t container = vr::VRProperties()->TrackedDeviceToPropertyContainer(unObjectId);
	//vr::VRProperties()->SetInt32Property(container, vr::Prop_ControllerRoleHint_Int32, role_);

	//vr::VRProperties()->SetStringProperty(container, vr::Prop_ModelNumber_String, "MySampleControllerModel_1");

	//vr::VRProperties()->SetStringProperty(container, vr::Prop_InputProfilePath_String,
	//	"{immersive_gloves_controller}/input/immersive_gloves_controller_profile.json");

	//vr::VRDriverInput()->CreateScalarComponent(container, "/input/index/value", &input_handles_[kInputHandle_index_value],
	//	vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);

	//vr::VRDriverInput()->CreateScalarComponent(container, "/input/middle/value", &input_handles_[kInputHandle_middle_value],
	//	vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);

	//vr::VRDriverInput()->CreateScalarComponent(container, "/input/ring/value", &input_handles_[kInputHandle_ring_value],
	//	vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);

	//vr::VRDriverInput()->CreateScalarComponent(container, "/input/pinky/value", &input_handles_[kInputHandle_pinky_value],
	//	vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedOneSided);

	device_id_ = unObjectId;
	char logstring[50] = {};
	sprintf_s(logstring, "device_id_: %d", device_id_);
	vr::VRDriverLog()->Log(logstring);
	return vr::VRInitError_None;
}

void ControllerDevice::RunFrame() {
	//vr::VRServerDriverHost()->TrackedDevicePoseUpdated(device_id_, GetPose(), sizeof(vr::DriverPose_t));

	//vr::VRDriverInput()->UpdateScalarComponent(input_handles_[kInputHandle_index_value], 1, 0.0);
	//vr::VRDriverInput()->UpdateScalarComponent(input_handles_[kInputHandle_middle_value], 0.75, 0.0);
	//vr::VRDriverInput()->UpdateScalarComponent(input_handles_[kInputHandle_ring_value], 0.5, 0.0);
	//vr::VRDriverInput()->UpdateScalarComponent(input_handles_[kInputHandle_pinky_value], 0.25, 0.0);
}

void ControllerDevice::PrintDeviceId() {
	char logstring[50] = {};
	sprintf_s(logstring, "print device_id_ id: %d", device_id_);
	vr::VRDriverLog()->Log(logstring);
}

void ControllerDevice::WritePipe() {
	//vr::VRDriverLog()->Log("WRITING");
	char logstring[50] = {};
	sprintf_s(logstring, "id: %d %f %f %f %f", device_id_, data.flexion[1][0], data.flexion[2][0], data.flexion[3][0], data.flexion[4][0]);
	vr::VRDriverLog()->Log(logstring);

	if (hPipe != INVALID_HANDLE_VALUE) {
		WriteFile(hPipe,
			&data,
			sizeof(InputData),
			&dwWritten,
			NULL);
	}
	else {
		vr::VRDriverLog()->Log("Tried to write to pipe but handle is invalid! Recreating pipe");
		CloseHandle(hPipe);
		if (role_ == vr::TrackedControllerRole_LeftHand) {
			hPipe = CreateFile(TEXT("\\\\.\\pipe\\vrapplication\\input\\glove\\v2\\left"),
				GENERIC_READ | GENERIC_WRITE,
				0,
				NULL,
				OPEN_EXISTING,
				0,
				NULL);
		}
		else if (role_ == vr::TrackedControllerRole_RightHand) {
			hPipe = CreateFile(TEXT("\\\\.\\pipe\\vrapplication\\input\\glove\\v2\\right"),
				GENERIC_READ | GENERIC_WRITE,
				0,
				NULL,
				OPEN_EXISTING,
				0,
				NULL);
		}
		WriteFile(hPipe,
			&data,
			sizeof(InputData),
			&dwWritten,
			NULL);
	}
}

void ControllerDevice::HandleEvent(const vr::VREvent_t& vrevent) {
	//char logstring[100] = {};
	//sprintf_s(logstring, "device_id_ %d - trackedDeviceIndex %d - vrevent.eventType %d", device_id_, vrevent.trackedDeviceIndex, vrevent.eventType);
	//vr::VRDriverLog()->Log(logstring);

	//switch (vrevent.eventType) {
	//case vr::VREvent_PropertyChanged: {
	//	vr::VRDriverLog()->Log("Buzz!");
	//}
	//case vr::VREvent_Input_HapticVibration: {
	//	if (vrevent.data.hapticVibration.componentHandle == input_handles_[kInputHandle_haptic]) {
	//		vr::VRDriverLog()->Log("Buzz!");
	//	}
	//	break;
	//}
	//}
}

void ControllerDevice::SetTrackerId(short deviceId, bool isRightHand) {
	//m_controllerPose->SetShadowEteeTracker(deviceId, isRightHand);
}

vr::ETrackedControllerRole ControllerDevice::GetDeviceRole() const {
	return role_;
}

void ControllerDevice::Deactivate() {
	//CloseHandle(hPipeLeft);  // close pipes
	//CloseHandle(hPipeRight); // close pipes
}

void ControllerDevice::EnterStandby() {
}

void* ControllerDevice::GetComponent(const char* pchComponentNameAndVersion) {
	return nullptr;
}

void ControllerDevice::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) {
	if (unResponseBufferSize >= 1)
		pchResponseBuffer[0] = 0;
}

// 3x3 or 3x4 matrix
template < class T >
vr::HmdQuaternion_t HmdQuaternion_FromMatrix(const T& matrix)
{
	vr::HmdQuaternion_t q{};

	q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
	q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;

	q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
	q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
	q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);

	return q;
}

vr::DriverPose_t ControllerDevice::GetPose() {
	vr::DriverPose_t pose = { 0 };

	pose.poseIsValid = true;
	pose.result = vr::TrackingResult_Running_OK;
	pose.deviceIsConnected = true;

	pose.qWorldFromDriverRotation.w = 1.f;
	pose.qDriverFromHeadRotation.w = 1.f;

	pose.qRotation.w = 1.f;

	//declare an array of poses for all tracked devices
	vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
	//fill the array of poses
	vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0.f, poses, vr::k_unMaxTrackedDeviceCount);

	//now, this must be improved later, but currently, the left glove must be the 8th device to be added to steam vr list of devices, and the right glove must be the 9th
	//this works on my system specifically, since this is my list of devices:
	// 0 - headset
	// 1 - IMMRSV-GLV-LEFT - the virtual device I add that will receive data from the tracker of the left glove
	// 2 - IMMRSV-GLV-RIGHT - the virtual device I add that will receive data from the tracker of the right glove
	// 3 - Base Station 1
	// 4 - Base Station 2
	// 5 - Base Station 3
	// 6 - Base Station 4
	// 7 - Tracker from the left glove
	// 8 - Tracker from the right glove
	if (role_ == vr::TrackedControllerRole_LeftHand) {
		//compute the orientation of the glove in respect to the headset using poses[7]
		const vr::HmdQuaternion_t hmd_orientation = HmdQuaternion_FromMatrix(poses[7].mDeviceToAbsoluteTracking);
		//fill the qRotation part of the pose
		pose.qRotation = hmd_orientation;

		//fill the vecPosition part of the pose using the poses[7]
		pose.vecPosition[0] = poses[7].mDeviceToAbsoluteTracking.m[0][3];
		pose.vecPosition[1] = poses[7].mDeviceToAbsoluteTracking.m[1][3];
		pose.vecPosition[2] = poses[7].mDeviceToAbsoluteTracking.m[2][3];
	} else if (role_ == vr::TrackedControllerRole_RightHand) {
		//compute the orientation of the glove in respect to the headset using poses[8]
		const vr::HmdQuaternion_t hmd_orientation = HmdQuaternion_FromMatrix(poses[8].mDeviceToAbsoluteTracking);
		//fill the qRotation part of the pose
		pose.qRotation = hmd_orientation;

		//fill the vecPosition part of the pose using poses[8]
		pose.vecPosition[0] = poses[8].mDeviceToAbsoluteTracking.m[0][3];
		pose.vecPosition[1] = poses[8].mDeviceToAbsoluteTracking.m[1][3];
		pose.vecPosition[2] = poses[8].mDeviceToAbsoluteTracking.m[2][3];
	}

	return pose;
}