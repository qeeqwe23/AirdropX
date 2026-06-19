#define S_FUNCTION_NAME  sfun_airdropx_jsbsim
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// JSBSim CMake installs headers under include/JSBSim by default.
#include <JSBSim/FGFDMExec.h>
#include <JSBSim/initialization/FGInitialCondition.h>
#include <JSBSim/simgear/misc/sg_path.hxx>

namespace {

constexpr int kInputWidth = 6;
constexpr int kOutputWidth = 20;
constexpr double kTrimThrottle = 0.80;
constexpr double kFallbackTrimElevator = 0.00110;
constexpr double kElevatorCmdLimit = 1.0;

enum InputIndex {
    kElevatorDelta = 0,
    kThrottleCmd = 1,
    kWindSpeedMps = 2,
    kWindDirFromDeg = 3,
    kDropCmd = 4,
    kResetCmd = 5,
};

enum OutputIndex {
    kTime = 0,
    kAltitudeM = 1,
    kVzUpMps = 2,
    kAirspeedMps = 3,
    kGroundspeedMps = 4,
    kPitchDeg = 5,
    kRollDeg = 6,
    kHeadingDeg = 7,
    kQbarPa = 8,
    kMassKg = 9,
    kCgXM = 10,
    kPosNM = 11,
    kPosEM = 12,
    kElevatorCmdNorm = 13,
    kThrottleNorm = 14,
    kWindNMps = 15,
    kWindEMps = 16,
    kDropCount = 17,
    kValid = 18,
    kReserved = 19,
};

std::string mxStringParam(SimStruct* S, int idx)
{
    const mxArray* p = ssGetSFcnParam(S, idx);
    const mxArray* value = p;
    mxArray* converted = nullptr;

    if (!mxIsChar(value)) {
        if (mxIsClass(value, "string")) {
            mxArray* rhs[1] = {const_cast<mxArray*>(value)};
            mxArray* lhs[1] = {nullptr};
            if (mexCallMATLAB(1, lhs, 1, rhs, "char") != 0 || lhs[0] == nullptr) {
                throw std::runtime_error("Failed to convert MATLAB string parameter to char.");
            }
            converted = lhs[0];
            value = converted;
        } else {
            throw std::runtime_error("S-function parameter must be char or MATLAB string.");
        }
    }
    char* raw = mxArrayToString(value);
    if (converted) {
        mxDestroyArray(converted);
    }
    if (!raw) {
        throw std::runtime_error("Failed to read S-function string parameter.");
    }
    std::string out(raw);
    mxFree(raw);
    return out;
}

double mxScalarParam(SimStruct* S, int idx)
{
    const mxArray* p = ssGetSFcnParam(S, idx);
    if (!mxIsDouble(p) || mxGetNumberOfElements(p) != 1) {
        throw std::runtime_error("S-function parameter must be a scalar double.");
    }
    return mxGetScalar(p);
}

double clip(double v, double lo, double hi)
{
    return std::max(lo, std::min(hi, v));
}

class AirdropXJsbsimPlant {
public:
    AirdropXJsbsimPlant(std::string projectRoot, std::string aircraftName,
                        std::string icName, double dt)
        : projectRoot_(std::move(projectRoot)),
          aircraftName_(std::move(aircraftName)),
          icName_(std::move(icName)),
          dt_(dt)
    {
        reset();
    }

    void reset()
    {
        fdm_ = std::make_unique<JSBSim::FGFDMExec>();

        // JSBSim searches aircraft/, engine/, and systems/ below root.
        fdm_->SetRootDir(SGPath(projectRoot_));
        fdm_->SetAircraftPath(SGPath("aircraft"));
        fdm_->SetEnginePath(SGPath("engine"));
        fdm_->SetSystemsPath(SGPath("systems"));

        if (!fdm_->LoadModel(aircraftName_)) {
            throw std::runtime_error("JSBSim LoadModel failed: " + aircraftName_);
        }

        fdm_->Setdt(dt_);

        if (!icName_.empty()) {
            if (!fdm_->GetIC()->Load(SGPath(icName_), false)) {
                throw std::runtime_error("JSBSim IC load failed: " + icName_);
            }
        }

        if (!fdm_->RunIC()) {
            throw std::runtime_error("JSBSim RunIC failed.");
        }

        time_ = 0.0;
        posNIntM_ = 0.0;
        posEIntM_ = 0.0;
        totalMassKg_ = 3423.0;
        cgXM_ = 5.2366;
        dropCount_ = 0;
        prevDropCmd_ = false;
        prevResetCmd_ = false;
        lastElevatorCmd_ = 0.0;
        lastThrottleCmd_ = kTrimThrottle;

        const auto engineSnapshot = warmupEngine();
        trimElevator_ = autoTrimSettle();
        if (std::abs(trimElevator_) < 0.005) {
            trimElevator_ = kFallbackTrimElevator;
        }
        restoreInitialCondition();
        applyEngineSnapshot(engineSnapshot);
        setElevator(trimElevator_);
        setThrottle(kTrimThrottle);
        lastElevatorCmd_ = trimElevator_;
        lastThrottleCmd_ = kTrimThrottle;
        updateMassCg();
    }

    void step(const double* u)
    {
        const bool resetCmd = u[kResetCmd] > 0.5;
        if (resetCmd && !prevResetCmd_) {
            prevResetCmd_ = resetCmd;
            reset();
            return;
        }
        prevResetCmd_ = resetCmd;

        setWind(u[kWindSpeedMps], u[kWindDirFromDeg]);

        const double elevatorCmd = clip(trimElevator_ + u[kElevatorDelta],
                                        -kElevatorCmdLimit, kElevatorCmdLimit);
        const double throttleCmd = clip(u[kThrottleCmd], 0.0, 1.0);

        setElevator(elevatorCmd);
        setThrottle(throttleCmd);

        const bool dropCmd = u[kDropCmd] > 0.5;
        if (dropCmd && !prevDropCmd_) {
            triggerDrop();
        }
        prevDropCmd_ = dropCmd;

        if (!fdm_->Run()) {
            throw std::runtime_error("JSBSim Run failed.");
        }

        time_ += dt_;
        posNIntM_ += getOrDefault("velocities/v-north-fps", 0.0) * 0.3048 * dt_;
        posEIntM_ += getOrDefault("velocities/v-east-fps", 0.0) * 0.3048 * dt_;
        lastElevatorCmd_ = elevatorCmd;
        lastThrottleCmd_ = throttleCmd;
    }

    void outputs(double* y) const
    {
        y[kTime] = time_;
        y[kAltitudeM] = getOrDefault("position/h-agl-ft", 0.0) * 0.3048;
        y[kVzUpMps] = -getOrDefault("velocities/v-down-fps", 0.0) * 0.3048;
        y[kAirspeedMps] = getOrDefault("velocities/vtrue-fps", 0.0) * 0.3048;
        y[kGroundspeedMps] = std::hypot(getOrDefault("velocities/v-north-fps", 0.0),
                                        getOrDefault("velocities/v-east-fps", 0.0)) * 0.3048;
        y[kPitchDeg] = getOrDefault("attitude/theta-deg", 0.0);
        y[kRollDeg] = getOrDefault("attitude/phi-deg", 0.0);
        y[kHeadingDeg] = getOrDefault("attitude/psi-deg", 0.0);
        y[kQbarPa] = getOrDefault("aero/qbar-psf", 0.0) * 47.88025898;
        y[kMassKg] = totalMassKg_;
        y[kCgXM] = cgXM_;
        y[kPosNM] = posNIntM_;
        y[kPosEM] = posEIntM_;
        y[kElevatorCmdNorm] = lastElevatorCmd_;
        y[kThrottleNorm] = lastThrottleCmd_;
        y[kWindNMps] = windNMps_;
        y[kWindEMps] = windEMps_;
        y[kDropCount] = static_cast<double>(dropCount_);
        y[kValid] = 1.0;
        y[kReserved] = 0.0;
    }

private:
    bool setIfExists(const std::string& prop, double value)
    {
        try {
            fdm_->SetPropertyValue(prop, value);
            return true;
        } catch (...) {
            return false;
        }
    }

    double getOrDefault(const std::string& prop, double fallback) const
    {
        try {
            return fdm_->GetPropertyValue(prop);
        } catch (...) {
            return fallback;
        }
    }

    using PropertySnapshot = std::vector<std::pair<std::string, double>>;

    PropertySnapshot warmupEngine()
    {
        setIfExists("propulsion/set-running", -1.0);
        setThrottle(kTrimThrottle);
        for (int i = 0; i < 300; ++i) {
            fdm_->Run();
        }
        return readEngineSnapshot();
    }

    PropertySnapshot readEngineSnapshot() const
    {
        PropertySnapshot snapshot;
        const std::vector<std::string> props = {
            "propulsion/engine[0]/n1",
            "propulsion/engine[0]/n2",
            "propulsion/engine[0]/thrust-lbs",
            "propulsion/engine[0]/fuel-flow-rate-pps",
            "propulsion/engine[0]/set-running",
            "propulsion/engine/n1",
            "propulsion/engine/n2",
            "propulsion/engine/thrust-lbs",
            "propulsion/engine/fuel-flow-rate-pps",
            "propulsion/engine/set-running"
        };
        for (const auto& p : props) {
            try {
                snapshot.emplace_back(p, fdm_->GetPropertyValue(p));
            } catch (...) {
            }
        }
        return snapshot;
    }

    void applyEngineSnapshot(const PropertySnapshot& snapshot)
    {
        for (const auto& item : snapshot) {
            setIfExists(item.first, item.second);
        }
    }

    void restoreInitialCondition()
    {
        if (!icName_.empty()) {
            if (!fdm_->GetIC()->Load(SGPath(icName_), false)) {
                throw std::runtime_error("JSBSim IC reload failed: " + icName_);
            }
        }
        if (!fdm_->RunIC()) {
            throw std::runtime_error("JSBSim RunIC after warmup failed.");
        }
        time_ = 0.0;
        posNIntM_ = 0.0;
        posEIntM_ = 0.0;
    }

    double readInitialTrim() const
    {
        double trim = getOrDefault("fcs/elevator-pos-norm", 0.0);
        if (std::abs(trim) < 1.0e-4) {
            trim = 0.045;
        }
        return trim;
    }

    double autoTrimSettle()
    {
        double trim = readInitialTrim();
        const int steps = std::max(1, static_cast<int>(3.0 / dt_));
        for (int i = 0; i < steps; ++i) {
            const double vzMps = getOrDefault("velocities/h-dot-fps", 0.0) * 0.3048;
            const double qDps = getOrDefault("velocities/q-rad_sec", 0.0) * 57.29577951308232;
            trim += (0.06 * vzMps + 0.01 * qDps) * dt_;
            trim = clip(trim, -0.5, 0.5);
            setElevator(trim);
            if (!fdm_->Run()) {
                break;
            }
        }
        return trim;
    }

    void setElevator(double elevator)
    {
        setIfExists("fcs/elevator-cmd-norm", elevator);
        setIfExists("fcs/elevator-pos-norm", elevator);
    }

    void setThrottle(double throttle)
    {
        const std::vector<std::string> props = {
            "fcs/throttle-cmd-norm",
            "fcs/throttle-pos-norm",
            "propulsion/engine[0]/throttle-cmd-norm",
            "propulsion/engine/throttle-cmd-norm"
        };
        for (const auto& p : props) {
            setIfExists(p, throttle);
        }
    }

    void setWind(double windSpeedMps, double windDirFromDeg)
    {
        const double ws = std::max(0.0, windSpeedMps);
        const double dirToDeg = std::fmod(windDirFromDeg + 180.0, 360.0);
        const double pi = 3.14159265358979323846;
        const double rad = dirToDeg * pi / 180.0;
        windNMps_ = ws * std::cos(rad);
        windEMps_ = ws * std::sin(rad);
        setIfExists("atmosphere/wind-north-fps", windNMps_ / 0.3048);
        setIfExists("atmosphere/wind-east-fps", windEMps_ / 0.3048);
        setIfExists("atmosphere/wind-down-fps", 0.0);
    }

    void triggerDrop()
    {
        if (dropCount_ >= 4) {
            return;
        }

        const std::string prop = "inertia/pointmass-weight-lbs[" + std::to_string(dropCount_) + "]";
        setIfExists(prop, 0.0);
        ++dropCount_;
        updateMassCg();
    }

    void updateMassCg()
    {
        constexpr double emptyMassKg = 2223.0;
        constexpr double emptyCgXM = 5.279;
        const double cargoMassKg[4] = {300.0, 300.0, 300.0, 300.0};
        const double cargoXM[4] = {4.826, 5.131, 5.436, 5.740};

        double totalMass = emptyMassKg;
        double totalMoment = emptyMassKg * emptyCgXM;
        for (int i = dropCount_; i < 4; ++i) {
            totalMass += cargoMassKg[i];
            totalMoment += cargoMassKg[i] * cargoXM[i];
        }
        totalMassKg_ = totalMass;
        cgXM_ = totalMoment / totalMass;
    }

    std::string projectRoot_;
    std::string aircraftName_;
    std::string icName_;
    double dt_ = 1.0 / 120.0;
    std::unique_ptr<JSBSim::FGFDMExec> fdm_;
    double time_ = 0.0;
    double posNIntM_ = 0.0;
    double posEIntM_ = 0.0;
    double totalMassKg_ = 3423.0;
    double cgXM_ = 5.2366;
    double windNMps_ = 0.0;
    double windEMps_ = 0.0;
    int dropCount_ = 0;
    bool prevDropCmd_ = false;
    bool prevResetCmd_ = false;
    double lastElevatorCmd_ = 0.0;
    double lastThrottleCmd_ = kTrimThrottle;
    double trimElevator_ = kFallbackTrimElevator;
};

} // namespace

static void mdlInitializeSizes(SimStruct* S)
{
    ssSetNumSFcnParams(S, 4); // projectRoot, aircraftName, icName, dt
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, kInputWidth);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortRequiredContiguous(S, 0, 1);

    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, kOutputWidth);

    ssSetNumPWork(S, 1);
    ssSetNumSampleTimes(S, 1);
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

static void mdlInitializeSampleTimes(SimStruct* S)
{
    const double dt = mxScalarParam(S, 3);
    ssSetSampleTime(S, 0, dt);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_START
static void mdlStart(SimStruct* S)
{
    try {
        const std::string projectRoot = mxStringParam(S, 0);
        const std::string aircraftName = mxStringParam(S, 1);
        const std::string icName = mxStringParam(S, 2);
        const double dt = mxScalarParam(S, 3);
        auto* plant = new AirdropXJsbsimPlant(projectRoot, aircraftName, icName, dt);
        ssSetPWorkValue(S, 0, plant);
    } catch (const std::exception& e) {
        ssSetErrorStatus(S, e.what());
    }
}

static void mdlOutputs(SimStruct* S, int_T)
{
    auto* plant = static_cast<AirdropXJsbsimPlant*>(ssGetPWorkValue(S, 0));
    if (!plant) {
        ssSetErrorStatus(S, "AirdropX JSBSim plant is not initialized.");
        return;
    }

    const double* u = static_cast<const double*>(ssGetInputPortSignal(S, 0));
    double* y = ssGetOutputPortRealSignal(S, 0);

    try {
        plant->step(u);
        plant->outputs(y);
    } catch (const std::exception& e) {
        ssSetErrorStatus(S, e.what());
    }
}

static void mdlTerminate(SimStruct* S)
{
    auto* plant = static_cast<AirdropXJsbsimPlant*>(ssGetPWorkValue(S, 0));
    delete plant;
    ssSetPWorkValue(S, 0, nullptr);
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
