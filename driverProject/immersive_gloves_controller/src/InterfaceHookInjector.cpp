#include "Hooks/InterfaceHookInjector.h"

//#include "Util/DriverLog.h"
#include "Hooks/HookReceiver.h"
#include "Hooks/Hooking.h"

static IHookReceiver *HookReceiver = nullptr;

static Hook<void *(*)(vr::IVRDriverContext *, const char *, vr::EVRInitError *)> GetGenericInterfaceHook("IVRDriverContext::GetGenericInterface");

static Hook<vr::EVRInputError (*)(vr::IVRDriverInput *, vr::PropertyContainerHandle_t, const char *, vr::VRInputComponentHandle_t *)> CreateBooleanComponent003(
    "IVRDriverInput003::CreateBooleanComponent");

static Hook<vr::EVRInputError (*)(vr::IVRDriverInput *, vr::VRInputComponentHandle_t, bool, double)> UpdateBooleanComponentHook003(
    "IVRDriverInput003::UpdateBooleanComponent");

static Hook<vr::EVRInputError(*)(vr::IVRDriverInput*, vr::PropertyContainerHandle_t, const char*, vr::VRInputComponentHandle_t*)> CreateScalarComponent003(
    "IVRDriverInput003::CreateScalarComponent");

static Hook<vr::EVRInputError(*)(vr::IVRDriverInput*, vr::VRInputComponentHandle_t, bool, double)> UpdateScalarComponentHook003(
    "IVRDriverInput003::UpdateScalarComponent");

static Hook<bool (*)(vr::IVRServerDriverHost *, const char *pchDeviceSerialNumber, vr::ETrackedDeviceClass eDeviceClass, vr::ITrackedDeviceServerDriver *pDriver)>
    TrackedDeviceAdded006("IVRServerDriverHost006::TrackedDeviceAdded");

static bool DetourTrackedDeviceAdded006(
    vr::IVRServerDriverHost *_this, const char *pchDeviceSerialNumber, vr::ETrackedDeviceClass eDeviceClass, vr::ITrackedDeviceServerDriver *pDriver) {
  //DriverLog("Hook: TrackedDeviceAdded called, Serial: %s", pchDeviceSerialNumber);

  HookReceiver->TrackedDeviceAdded(pchDeviceSerialNumber, eDeviceClass, pDriver);

  auto retval = TrackedDeviceAdded006.originalFunc(_this, pchDeviceSerialNumber, eDeviceClass, pDriver);
  return retval;
}

static vr::EVRInputError DetourCreateBooleanComponent(
    vr::IVRDriverInput *_this, vr::PropertyContainerHandle_t ulContainer, const char *pchName, vr::VRInputComponentHandle_t *pHandle) {

  auto retval = CreateBooleanComponent003.originalFunc(_this, ulContainer, pchName, pHandle);

  HookReceiver->CreateBooleanComponent(ulContainer, pchName, pHandle);

  return retval;
}

static vr::EVRInputError DetourUpdateBooleanComponent(vr::IVRDriverInput *_this, vr::VRInputComponentHandle_t ulComponent, bool bNewValue, double fTimeOffset) {
  HookReceiver->UpdateBooleanComponent(ulComponent, bNewValue, fTimeOffset);

  auto retval = UpdateBooleanComponentHook003.originalFunc(_this, ulComponent, bNewValue, fTimeOffset);

  return retval;
}

static vr::EVRInputError DetourCreateScalarComponent(
    vr::IVRDriverInput* _this, vr::PropertyContainerHandle_t ulContainer, const char* pchName, vr::VRInputComponentHandle_t* pHandle) {

    auto retval = CreateScalarComponent003.originalFunc(_this, ulContainer, pchName, pHandle);

    HookReceiver->CreateScalarComponent(ulContainer, pchName, pHandle);

    return retval;
}

static vr::EVRInputError DetourUpdateScalarComponent(vr::IVRDriverInput* _this, vr::VRInputComponentHandle_t ulComponent, float fNewValue, double fTimeOffset) {
    HookReceiver->UpdateScalarComponent(ulComponent, fNewValue, fTimeOffset);

    auto retval = UpdateScalarComponentHook003.originalFunc(_this, ulComponent, fNewValue, fTimeOffset);

    return retval;
}

static void *DetourGetGenericInterface(vr::IVRDriverContext *_this, const char *pchInterfaceVersion, vr::EVRInitError *peError) {
  // DebugDriverLog("ServerTrackedDeviceProvider::DetourGetGenericInterface(%s)", pchInterfaceVersion);
  auto originalInterface = GetGenericInterfaceHook.originalFunc(_this, pchInterfaceVersion, peError);

  std::string iface(pchInterfaceVersion);
  if (iface == "IVRDriverInput_003") {
    if (!IHook::Exists(CreateBooleanComponent003.name)) {
      CreateBooleanComponent003.CreateHookInObjectVTable(originalInterface, 0, &DetourCreateBooleanComponent);
      IHook::Register(&CreateBooleanComponent003);
    }
    if (!IHook::Exists(UpdateBooleanComponentHook003.name)) {
      UpdateBooleanComponentHook003.CreateHookInObjectVTable(originalInterface, 1, &DetourUpdateBooleanComponent);
      IHook::Register(&UpdateBooleanComponentHook003);
    }
    if (!IHook::Exists(CreateScalarComponent003.name)) {
        CreateScalarComponent003.CreateHookInObjectVTable(originalInterface, 2, &DetourCreateScalarComponent);
        IHook::Register(&CreateScalarComponent003);
    }
    if (!IHook::Exists(UpdateScalarComponentHook003.name)) {
        UpdateScalarComponentHook003.CreateHookInObjectVTable(originalInterface, 3, &DetourUpdateScalarComponent);
        IHook::Register(&UpdateScalarComponentHook003);
    }
  } else if (iface == "IVRServerDriverHost_006") {
    if (!IHook::Exists(TrackedDeviceAdded006.name)) {
      TrackedDeviceAdded006.CreateHookInObjectVTable(originalInterface, 0, &DetourTrackedDeviceAdded006);
      IHook::Register(&TrackedDeviceAdded006);
    }
  }

  return originalInterface;
}

void InjectHooks(IHookReceiver *hookReceiver, vr::IVRDriverContext *pDriverContext) {
  HookReceiver = hookReceiver;

  auto err = MH_Initialize();

  if (err == MH_OK) {
    GetGenericInterfaceHook.CreateHookInObjectVTable(pDriverContext, 0, &DetourGetGenericInterface);
    IHook::Register(&GetGenericInterfaceHook);
  } else {
    //DriverLog("MH_Initialize error: %s", MH_StatusToString(err));
  }
}

void DisableHooks() {
  IHook::DestroyAll();
  MH_Uninitialize();
}