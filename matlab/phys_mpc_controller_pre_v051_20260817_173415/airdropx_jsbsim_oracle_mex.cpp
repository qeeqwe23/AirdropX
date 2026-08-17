#include "mex.h"

#include <JSBSim/FGFDMExec.h>
#include <JSBSim/initialization/FGInitialCondition.h>
#include <JSBSim/initialization/FGTrim.h>
#include <JSBSim/models/FGPropulsion.h>
#include <JSBSim/models/propulsion/FGTank.h>
#include <JSBSim/models/FGFCS.h>
#include <JSBSim/models/FGPropagate.h>
#include <JSBSim/simgear/misc/sg_path.hxx>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr double kFtToM = 0.3048;
constexpr double kSlugToKg = 14.5939029372;
constexpr double kSlugFt2ToKgM2 = 1.3558179483314004;
constexpr double kLbfToN = 4.4482216152605;
constexpr double kLbfFtToNm = 1.3558179483314004;
constexpr double kPpsToKgps = 0.45359237;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr int kStateN = 7;
constexpr int kInputN = 2;
constexpr int kTrimSeedN = 5;

std::string mxToString(const mxArray* a)
{
    const mxArray* value = a;
    mxArray* converted = nullptr;
    if (!mxIsChar(value)) {
        if (mxIsClass(value, "string")) {
            mxArray* rhs[1] = {const_cast<mxArray*>(value)};
            mxArray* lhs[1] = {nullptr};
            if (mexCallMATLAB(1, lhs, 1, rhs, "char") != 0 || lhs[0] == nullptr)
                throw std::runtime_error("Failed to convert MATLAB string to char.");
            converted = lhs[0];
            value = converted;
        } else {
            throw std::runtime_error("Expected char or string scalar.");
        }
    }
    char* p = mxArrayToString(value);
    if (converted) mxDestroyArray(converted);
    if (!p) throw std::runtime_error("Failed to convert MATLAB string.");
    std::string s(p);
    mxFree(p);
    return s;
}

double scalar(const mxArray* a, const char* name)
{
    if (!mxIsDouble(a) || mxIsComplex(a) || mxGetNumberOfElements(a) != 1)
        throw std::runtime_error(std::string(name) + " must be a real double scalar.");
    const double v = mxGetScalar(a);
    if (!std::isfinite(v)) throw std::runtime_error(std::string(name) + " must be finite.");
    return v;
}

std::vector<double> vectorN(const mxArray* a, int n, const char* name)
{
    if (!mxIsDouble(a) || mxIsComplex(a) || static_cast<int>(mxGetNumberOfElements(a)) != n)
        throw std::runtime_error(std::string(name) + " has wrong size.");
    const double* p = mxGetPr(a);
    std::vector<double> out(p, p+n);
    for (double v : out) {
        if (!std::isfinite(v)) throw std::runtime_error(std::string(name) + " contains non-finite values.");
    }
    return out;
}

mxArray* vectorMx(const std::vector<double>& v)
{
    mxArray* a = mxCreateDoubleMatrix(v.size(), 1, mxREAL);
    std::copy(v.begin(), v.end(), mxGetPr(a));
    return a;
}

class Oracle {
public:
    Oracle(std::string projectRoot, std::string aircraftName, std::string icName, double dt)
        : projectRoot_(std::move(projectRoot)), aircraftName_(std::move(aircraftName)),
          icName_(std::move(icName)), dt_(dt)
    {
        if (!(dt_ > 0.0)) throw std::runtime_error("dt must be positive.");
        load();
    }

    void load()
    {
        fdm_ = std::make_unique<JSBSim::FGFDMExec>();
        // Must be set before LoadModel so the model/IC reports do not flood MATLAB stdout.
        fdm_->SetDebugLevel(0);
        fdm_->SetRootDir(SGPath(projectRoot_));
        fdm_->SetAircraftPath(SGPath("aircraft"));
        fdm_->SetEnginePath(SGPath("engine"));
        fdm_->SetSystemsPath(SGPath("systems"));
        if (!fdm_->LoadModel(aircraftName_))
            throw std::runtime_error("JSBSim LoadModel failed: " + aircraftName_);
        fdm_->SetDebugLevel(0);
        fdm_->Setdt(dt_);
        loadIC();
        if (!fdm_->RunIC()) throw std::runtime_error("JSBSim RunIC failed.");
        if (!std::isfinite(get("aero/alphadot-rad_sec", std::numeric_limits<double>::quiet_NaN())))
            throw std::runtime_error("Aircraft/JSBSim build does not expose required aero/alphadot-rad_sec.");

        pointMassLbs_.clear();
        for (int i=0; i<4; ++i) {
            pointMassLbs_.push_back(get("inertia/pointmass-weight-lbs[" + std::to_string(i) + "]", 0.0));
        }
        if (pointMassLbs_.size()!=4 || std::any_of(pointMassLbs_.begin(),pointMassLbs_.end(),[](double w){return !std::isfinite(w) || w<=0.0;}))
            throw std::runtime_error("AirdropX cfg0..4 requires exactly four positive JSBSim cargo point masses.");
        fuelLbs_.clear();
        auto prop = fdm_->GetPropulsion();
        if (!prop) throw std::runtime_error("JSBSim propulsion model is unavailable.");
        if (prop->GetNumEngines() == 0) throw std::runtime_error("Aircraft has no propulsion engine.");
        for (size_t i=0; i<prop->GetNumTanks(); ++i) {
            fuelLbs_.push_back(prop->GetTank(static_cast<unsigned int>(i))->GetContents());
        }
        if (fuelLbs_.empty()) throw std::runtime_error("AirdropX physics oracle found no JSBSim fuel tanks.");

        // Fuel is a measured/scheduled LPV parameter, not a hidden state of the
        // local Ts map. Freeze consumption inside every oracle propagation and
        // let the runtime scheduler update mass/fuel between MPC samples.
        prop->SetFuelFreeze(true);

        // Establish a valid 80%-throttle running-engine baseline. Important:
        // FGPropulsion::InitRunning(-1) internally sets throttle to 1.0 before its
        // first steady-state solve, so the requested throttle MUST be re-applied
        // afterwards and GetSteadyState() called again.
        establishEngineSteadyState(0.80);
        baseN1_ = readN1(80.0);
        baseN2_ = readN2(80.0);
        restoreBaseIC();
    }

    std::pair<std::vector<double>, mxArray*> eval(const std::vector<double>& x,
                                                   const std::vector<double>& u,
                                                   int cfgId, double fuelScale,
                                                   double Ts)
    {
        validateCfgFuelTime(cfgId, fuelScale, Ts);
        const int nSteps = stepsFor(Ts);
        prepareState(x, u, cfgId, fuelScale, false);
        for (int k=0; k<nSteps; ++k) {
            if (!fdm_->Run()) throw std::runtime_error("JSBSim Run failed during oracle eval.");
        }
        return {readState(x[5],x[6]), diagnostics()};
    }

    // Evaluate the exact Ts-map with engine spool eliminated from the trim search.
    // N1/N2 are obtained from JSBSim propulsion steady-state physics for each throttle.
    mxArray* steadyEval(double H, double V, double theta,
                        double elevator, double throttle,
                        int cfgId, double fuelScale, double Ts)
    {
        validateCfgFuelTime(cfgId, fuelScale, Ts);
        if (!(H > -1000.0 && V > 0.1)) throw std::runtime_error("Invalid trim H/V.");
        const int nSteps = stepsFor(Ts);
        std::vector<double> xseed = {H,V,0.0,theta,0.0,baseN1_,baseN2_};
        std::vector<double> u = {elevator,throttle};
        prepareState(xseed, u, cfgId, fuelScale, true);
        const std::vector<double> x0 = readState(baseN1_,baseN2_);
        mxArray* diag0 = diagnostics();
        for (int k=0; k<nSteps; ++k) {
            if (!fdm_->Run()) {
                mxDestroyArray(diag0);
                throw std::runtime_error("JSBSim Run failed during steady_eval.");
            }
        }
        const std::vector<double> x1 = readState(x0[5],x0[6]);
        mxArray* diag1 = diagnostics();

        const char* fields[] = {"x0","xnext","diag0","diag1"};
        mxArray* s = mxCreateStructMatrix(1,1,4,fields);
        mxSetField(s,0,"x0",vectorMx(x0));
        mxSetField(s,0,"xnext",vectorMx(x1));
        mxSetField(s,0,"diag0",diag0);
        mxSetField(s,0,"diag1",diag1);
        return s;
    }

    // JSBSim's native longitudinal trim is used only as a physics-informed seed.
    // The MATLAB layer independently polishes/verifies the exact discrete Ts-map.
    mxArray* builtinTrim(double H, double V, int cfgId, double fuelScale,
                         const std::vector<double>& seed)
    {
        validateCfgFuelTime(cfgId, fuelScale, dt_);
        if (static_cast<int>(seed.size()) != kTrimSeedN) throw std::runtime_error("trim seed must have 5 elements.");
        std::vector<double> xseed = {H,V,0.0,seed[0],0.0,seed[3],seed[4]};
        std::vector<double> useed = {seed[1],seed[2]};

        // First initialize the requested flight condition/configuration.
        prepareState(xseed, useed, cfgId, fuelScale, true);

        JSBSim::FGTrim trim(fdm_.get(), JSBSim::tLongitudinal);
        trim.ClearDebug();
        trim.SetGammaFallback(false); // level-flight trim must not silently change gamma.
        trim.SetMaxCycles(120);
        trim.SetMaxCyclesPerAxis(200);
        // JSBSim 1.3.1's predefined tLongitudinal mode actually pairs qdot
        // with the *pitch-trim* command internally. AirdropX exposes elevator
        // as the longitudinal control input and does not expose pitch trim as
        // a separate actuator. Replace only that axis so the seed is generated
        // in the same input coordinates used by the plant/MPC.
        if (!trim.EditState(JSBSim::tQdot, JSBSim::tElevator))
            throw std::runtime_error("FGTrim could not remap qdot to primary elevator.");
        const bool ok = trim.DoTrim();

        // Ensure propulsion is left on the throttle selected by FGTrim and in
        // a consistent engine steady state before the seed is exported.
        const double trimThrottle = getThrottle(seed[2]);
        establishEngineSteadyState(trimThrottle);
        // FGTrim iterates the aircraft to equilibrium, but the model contains
        // alpha-rate aerodynamic terms. Re-settle the zero-dt derivative loop
        // after the final engine steady-state call so the exported seed and
        // diagnostics are in the same algebraically consistent coordinates as
        // every later Oracle evaluation.
        settleAndPrimeCurrentDynamics();

        const double gamma = get("flight-path/gamma-rad",0.0);
        const double alpha = get("aero/alpha-rad",0.0);
        const double theta = get("attitude/theta-rad", alpha + gamma);
        auto fcs = fdm_->GetFCS();
        const double elevator = fcs ? fcs->GetDeCmd() : seed[1];
        const double throttle = getThrottle(seed[2]);
        const std::vector<double> x = {H,V,gamma,theta,0.0,readN1(seed[3]),readN2(seed[4])};
        const std::vector<double> u = {elevator,throttle};
        const std::vector<double> z = {theta,elevator,throttle,x[5],x[6]};

        const char* fields[] = {"success","z","x","u","diag"};
        mxArray* s = mxCreateStructMatrix(1,1,5,fields);
        mxSetField(s,0,"success",mxCreateLogicalScalar(ok));
        mxSetField(s,0,"z",vectorMx(z));
        mxSetField(s,0,"x",vectorMx(x));
        mxSetField(s,0,"u",vectorMx(u));
        mxSetField(s,0,"diag",diagnostics());
        return s;
    }

    mxArray* info() const
    {
        const char* fields[] = {"dt","base_n1","base_n2","pointmass_lbs","fuel_lbs","state_definition","input_definition","version"};
        mxArray* s = mxCreateStructMatrix(1,1,8,fields);
        mxSetField(s,0,"dt",mxCreateDoubleScalar(dt_));
        mxSetField(s,0,"base_n1",mxCreateDoubleScalar(baseN1_));
        mxSetField(s,0,"base_n2",mxCreateDoubleScalar(baseN2_));
        mxSetField(s,0,"pointmass_lbs",vectorMx(pointMassLbs_));
        mxSetField(s,0,"fuel_lbs",vectorMx(fuelLbs_));
        mxSetField(s,0,"state_definition",mxCreateString("[h_m Va_mps gamma_rad theta_rad q_radps N1 N2]"));
        mxSetField(s,0,"input_definition",mxCreateString("[elevator_abs_norm throttle_norm]"));
        mxSetField(s,0,"version",mxCreateString("AirdropX JSBSim physics oracle v0.3.3"));
        return s;
    }

private:
    void loadIC()
    {
        if (!icName_.empty()) {
            if (!fdm_->GetIC()->Load(SGPath(icName_), false))
                throw std::runtime_error("JSBSim IC load failed: " + icName_);
        }
    }

    double get(const std::string& p, double fallback) const
    {
        try {
            const double v = fdm_->GetPropertyValue(p);
            return std::isfinite(v) ? v : fallback;
        } catch (...) { return fallback; }
    }

    bool set(const std::string& p, double v, bool required=true)
    {
        try {
            fdm_->SetPropertyValue(p,v);
            return true;
        } catch (...) {
            if (required) throw std::runtime_error("Required JSBSim property is not writable: " + p);
            return false;
        }
    }

    void restoreBaseIC()
    {
        loadIC();
        if (!fdm_->RunIC()) throw std::runtime_error("RunIC after warmup failed.");
    }

    int stepsFor(double Ts) const
    {
        const double nReal = Ts/dt_;
        const int nSteps = static_cast<int>(std::llround(nReal));
        if (nSteps < 1 || std::abs(nReal - nSteps) > 1e-9)
            throw std::runtime_error("Ts must be an integer multiple of oracle dt.");
        return nSteps;
    }

    void validateCfgFuelTime(int cfgId, double fuelScale, double Ts) const
    {
        if (cfgId < 0 || cfgId > 4) throw std::runtime_error("cfgId must be 0..4.");
        if (!(fuelScale >= 0.0 && fuelScale <= 1.2)) throw std::runtime_error("fuelScale must be in [0,1.2].");
        if (!(Ts > 0.0)) throw std::runtime_error("Ts must be positive.");
        (void)stepsFor(Ts);
    }

    void applyMassConfiguration(int cfgId, double fuelScale)
    {
        constexpr double kEchoTol = 1e-8;
        for (int i=0; i<4; ++i) {
            const std::string propName = "inertia/pointmass-weight-lbs[" + std::to_string(i) + "]";
            const double expected = (i < cfgId) ? 0.0 : pointMassLbs_.at(i);
            set(propName, expected);
            const double actual = get(propName, std::numeric_limits<double>::quiet_NaN());
            if (!std::isfinite(actual) || std::abs(actual-expected) > kEchoTol*std::max(1.0,std::abs(expected)))
                throw std::runtime_error("Point-mass write/readback verification failed: " + propName);
        }

        auto prop = fdm_->GetPropulsion();
        if (!prop) throw std::runtime_error("Propulsion unavailable while applying fuel configuration.");
        if (prop->GetNumTanks() != fuelLbs_.size())
            throw std::runtime_error("Fuel tank count changed after model initialization.");
        prop->SetFuelFreeze(true);
        for (size_t i=0; i<fuelLbs_.size(); ++i) {
            auto tank = prop->GetTank(static_cast<unsigned int>(i));
            const double expected = fuelScale*fuelLbs_[i];
            if (expected > tank->GetCapacity() + 1e-9)
                throw std::runtime_error("fuelScale requests fuel above physical tank capacity at tank " + std::to_string(i));
            tank->SetContents(expected);
            const double actual = tank->GetContents();
            if (!std::isfinite(actual) || std::abs(actual-expected) > kEchoTol*std::max(1.0,std::abs(expected)))
                throw std::runtime_error("Fuel write/readback verification failed at tank " + std::to_string(i));
        }
    }

    void verifyMassConfiguration(int cfgId, double fuelScale) const
    {
        constexpr double kEchoTol = 1e-8;
        for (int i=0; i<4; ++i) {
            const std::string propName = "inertia/pointmass-weight-lbs[" + std::to_string(i) + "]";
            const double expected = (i < cfgId) ? 0.0 : pointMassLbs_.at(i);
            const double actual = get(propName, std::numeric_limits<double>::quiet_NaN());
            if (!std::isfinite(actual) || std::abs(actual-expected) > kEchoTol*std::max(1.0,std::abs(expected)))
                throw std::runtime_error("Point-mass configuration changed during zero-dt refresh: " + propName);
        }
        auto prop = fdm_->GetPropulsion();
        if (!prop || prop->GetNumTanks() != fuelLbs_.size())
            throw std::runtime_error("Fuel configuration cannot be verified.");
        for (size_t i=0; i<fuelLbs_.size(); ++i) {
            const double expected = fuelScale*fuelLbs_[i];
            const double actual = prop->GetTank(static_cast<unsigned int>(i))->GetContents();
            if (!std::isfinite(actual) || std::abs(actual-expected) > kEchoTol*std::max(1.0,std::abs(expected)))
                throw std::runtime_error("Fuel configuration changed during zero-dt refresh at tank " + std::to_string(i));
        }
    }

    void setIC(const std::vector<double>& x)
    {
        auto ic = fdm_->GetIC();
        if (!ic) throw std::runtime_error("Initial-condition object unavailable.");

        // Every oracle evaluation starts from a clean IC object. This prevents
        // path-dependent leftovers (wind, prior speed mode, prior attitude, etc.)
        // from a previous persistent-oracle call.
        ic->InitializeIC();

        // For the declared zero-wind longitudinal state, alpha = theta-gamma.
        // JSBSim's alpha/gamma IC setters maintain a kinematically consistent
        // attitude/velocity set. We verify the resulting declared state after
        // RunIC() instead of relying on this relation silently.
        const double alpha = x[3] - x[2];
        set("ic/h-agl-ft", x[0]/kFtToM);
        set("ic/vt-fps", x[1]/kFtToM);
        set("ic/gamma-rad", x[2]);
        set("ic/alpha-rad", alpha);
        set("ic/beta-rad", 0.0);
        set("ic/phi-rad", 0.0);
        set("ic/psi-true-rad", 0.0);
        set("ic/p-rad_sec", 0.0);
        set("ic/q-rad_sec", x[4]);
        set("ic/r-rad_sec", 0.0);
        set("ic/vw-north-fps", 0.0, false);
        set("ic/vw-east-fps", 0.0, false);
        set("ic/vw-down-fps", 0.0, false);
    }

    void verifyDeclaredKinematics(const std::vector<double>& x) const
    {
        // Tight relative-to-finite-difference tolerances, but not bitwise:
        // RunIC traverses unit conversions and quaternion transforms.
        const double actual[5] = {
            get("position/h-agl-ft", 0.0)*kFtToM,
            get("velocities/vtrue-fps", 0.0)*kFtToM,
            get("flight-path/gamma-rad", 0.0),
            get("attitude/theta-rad", get("attitude/theta-deg",0.0)*kDegToRad),
            get("velocities/q-rad_sec", 0.0)
        };
        const double tol[5] = {1e-4, 1e-6, 1e-8, 1e-8, 1e-10};
        const char* name[5] = {"h","Va","gamma","theta","q"};
        for (int i=0; i<5; ++i) {
            if (!std::isfinite(actual[i]) || std::abs(actual[i]-x[i]) > tol[i]) {
                throw std::runtime_error(std::string("IC reconstruction/readback failed for ") + name[i] +
                    ": target=" + std::to_string(x[i]) +
                    ", actual=" + std::to_string(actual[i]) +
                    ", tol=" + std::to_string(tol[i]));
            }
        }
    }

    void prepareState(const std::vector<double>& x, const std::vector<double>& u,
                      int cfgId, double fuelScale, bool steadyEngine)
    {
        // RunIC() alone does NOT reset all JSBSim model memories. In JSBSim 1.3.1
        // FGFDMExec::RunIC() reconstructs the IC and initializes only Input/Output,
        // while FGFCS::InitModel() is what resets channel components (filters,
        // actuators, integrators, etc.). A persistent oracle therefore needs a full
        // model reset before every map evaluation, otherwise an A->B->A call sequence
        // can leak hidden FCS/propulsion state into the final A evaluation.
        //
        // DONT_EXECUTE_RUN_IC is intentional: reset every model first, then let this
        // routine overwrite the declared state/config/control before calling RunIC.
        fdm_->ResetToInitialConditions(JSBSim::FGFDMExec::DONT_EXECUTE_RUN_IC);

        setIC(x);
        if (!fdm_->RunIC()) throw std::runtime_error("RunIC failed while preparing oracle state.");

        // Apply cfg/fuel, verify direct write/readback, then run a zero-dt model
        // pass so mass balance, atmosphere and force inputs see the new parameters.
        // RunIC() does not call FGPropulsion::InitModel(), so tank contents are
        // not reset here; fuel is additionally frozen for this local LPV map.
        applyMassConfiguration(cfgId, fuelScale);
        if (!fdm_->RunIC()) throw std::runtime_error("RunIC failed after mass configuration.");
        verifyMassConfiguration(cfgId, fuelScale);
        verifyDeclaredKinematics(x);

        neutralizeUnusedFCS();
        setElevator(u[0]);
        // Initialize every hidden propulsion variable at the steady state that
        // corresponds to the REQUESTED throttle, not at InitRunning's 1.0 default.
        establishEngineSteadyState(u[1]);
        if (!steadyEngine) {
            // The declared engine states are then imposed as the only dynamic
            // propulsion perturbations. The MATLAB semigroup test verifies that
            // N1/N2 are sufficient to close this reconstructed state.
            setEngineState(x[5], x[6]);
        }
        // Re-apply controls because the propulsion steady-state routine modifies
        // internal engine/FCS quantities while converging.
        neutralizeUnusedFCS();
        setElevator(u[0]);
        setThrottle(u[1]);

        // Critical: refresh FCS -> aero/propulsion -> aircraft -> accelerations
        // with dt=0 *after* the final control/state writes, then prime the
        // multistep integrator derivative history from those current forces.
        // Without this, the first real step can integrate stale pre-command
        // accelerations and create a spurious attitude transient.
        settleAndPrimeCurrentDynamics();
        verifyControlEcho(u);
        verifyMassConfiguration(cfgId, fuelScale);
        verifyDeclaredKinematics(x);

        if (!steadyEngine) {
            const double r1=readN1(std::numeric_limits<double>::quiet_NaN());
            const double r2=readN2(std::numeric_limits<double>::quiet_NaN());
            if (!std::isfinite(r1) || !std::isfinite(r2) ||
                std::abs(r1-x[5])>1e-8 || std::abs(r2-x[6])>1e-8)
                throw std::runtime_error("Engine state changed during zero-dt algebraic settling.");
        }
    }

    void neutralizeUnusedFCS()
    {
        auto fcs=fdm_->GetFCS();
        if (!fcs) throw std::runtime_error("FCS unavailable while neutralizing unused controls.");
        // The declared oracle input is [elevator, throttle]. Keep every other
        // pilot trim/control channel deterministic and zero so no persistent
        // hidden FCS command can leak from FGTrim or a previous oracle call.
        fcs->SetPitchTrimCmd(0.0);
        fcs->SetRollTrimCmd(0.0);
        fcs->SetYawTrimCmd(0.0);
        fcs->SetDaCmd(0.0);
        fcs->SetDrCmd(0.0);
    }

    std::array<double,7> currentAlgebraicSnapshot() const
    {
        return {
            get("aero/alphadot-rad_sec", std::numeric_limits<double>::quiet_NaN()),
            get("accelerations/udot-ft_sec2", std::numeric_limits<double>::quiet_NaN())*kFtToM,
            get("accelerations/wdot-ft_sec2", std::numeric_limits<double>::quiet_NaN())*kFtToM,
            get("accelerations/qdot-rad_sec2", std::numeric_limits<double>::quiet_NaN()),
            get("forces/fbx-total-lbs", std::numeric_limits<double>::quiet_NaN())*kLbfToN,
            get("forces/fbz-total-lbs", std::numeric_limits<double>::quiet_NaN())*kLbfToN,
            get("moments/m-total-lbsft", std::numeric_limits<double>::quiet_NaN())*kLbfFtToNm
        };
    }

    static double algebraicSnapshotDelta(const std::array<double,7>& a,
                                         const std::array<double,7>& b)
    {
        // Numerical convergence scales only; these are not controller tuning
        // parameters or trim acceptance thresholds.  The first four entries
        // are the derivative feedback loop, while force/moment entries catch
        // a stale aerodynamic evaluation even if an acceleration happens to
        // cancel numerically.
        constexpr double scale[7] = {
            1.0e-3,  // alphadot rad/s
            10.0,    // udot m/s^2
            10.0,    // wdot m/s^2
            1.0,     // qdot rad/s^2
            1.0e5,   // Fx N
            1.0e5,   // Fz N
            1.0e5    // My N*m
        };
        double e=0.0;
        for (size_t i=0;i<a.size();++i) {
            if (!std::isfinite(a[i]) || !std::isfinite(b[i]))
                return std::numeric_limits<double>::infinity();
            e=std::max(e,std::abs(a[i]-b[i])/scale[i]);
        }
        return e;
    }

    void settleAndPrimeCurrentDynamics()
    {
        // JSBSim evaluates models in the order
        //   Auxiliary -> Aerodynamics -> Aircraft -> Accelerations.
        // FGAuxiliary computes alphadot from the *previous* Accelerations::UVWdot,
        // while the MQ9 aerodynamic model uses aero/alphadot-rad_sec in both lift
        // and pitch-moment terms.  A single zero-dt Run therefore leaves an
        // algebraic one-pass lag: the newly computed acceleration is not yet the
        // acceleration that generated the alphadot used by that same aero pass.
        //
        // With integration suspended the physical state is frozen, so repeated
        // zero-dt model passes are a fixed-point iteration of this internal
        // derivative/aerodynamic loop.  Only after successive passes agree do we
        // initialize FGPropagate's derivative history and allow real integration.
        if (fdm_->IntegrationSuspended())
            throw std::runtime_error("Oracle algebraic settle entered with integration already suspended.");

        constexpr int kMaxIterations=80;
        constexpr int kMinIterations=3;
        constexpr double kConvergenceTol=1.0e-9;

        lastAlgebraicIterations_=0;
        lastAlgebraicError_=std::numeric_limits<double>::infinity();
        lastAlgebraicConverged_=false;

        fdm_->SuspendIntegration();
        try {
            std::array<double,7> prev{};
            bool havePrev=false;
            for (int iter=1; iter<=kMaxIterations; ++iter) {
                if (!fdm_->Run())
                    throw std::runtime_error("JSBSim zero-dt refresh failed while settling current dynamics.");
                const auto cur=currentAlgebraicSnapshot();
                lastAlgebraicIterations_=iter;
                if (std::any_of(cur.begin(),cur.end(),[](double v){return !std::isfinite(v);}))
                    throw std::runtime_error("Required JSBSim alpha-rate/acceleration/force property is unavailable or non-finite during algebraic settle.");
                if (havePrev) {
                    lastAlgebraicError_=algebraicSnapshotDelta(cur,prev);
                    if (iter>=kMinIterations && lastAlgebraicError_<=kConvergenceTol) {
                        lastAlgebraicConverged_=true;
                        break;
                    }
                }
                prev=cur;
                havePrev=true;
            }

            if (!lastAlgebraicConverged_) {
                const auto cur=currentAlgebraicSnapshot();
                throw std::runtime_error(
                    "JSBSim zero-dt algebraic dynamics did not converge: iterations=" +
                    std::to_string(lastAlgebraicIterations_) +
                    ", scaled_delta=" + std::to_string(lastAlgebraicError_) +
                    ", alphadot=" + std::to_string(cur[0]) +
                    ", udot=" + std::to_string(cur[1]) +
                    ", wdot=" + std::to_string(cur[2]) +
                    ", qdot=" + std::to_string(cur[3]));
            }

            auto propagate = fdm_->GetPropagate();
            if (!propagate)
                throw std::runtime_error("FGPropagate unavailable while priming current dynamics.");
            propagate->InitializeDerivatives();
        } catch (...) {
            fdm_->ResumeIntegration();
            throw;
        }
        fdm_->ResumeIntegration();
    }

    void verifyControlEcho(const std::vector<double>& u) const
    {
        constexpr double kTol = 1e-9;
        auto fcs = fdm_->GetFCS();
        if (!fcs) throw std::runtime_error("FCS unavailable while verifying control echo.");

        const double deCmd = fcs->GetDeCmd();
        const double dePosNorm = get("fcs/elevator-pos-norm", std::numeric_limits<double>::quiet_NaN());
        const double throttleCmd = fcs->GetThrottleCmd(0);
        const double throttlePos = fcs->GetThrottlePos(0);

        auto bad=[&](double actual,double expected,const char* name) {
            if (!std::isfinite(actual) || std::abs(actual-expected) > kTol*std::max(1.0,std::abs(expected)))
                throw std::runtime_error(std::string("Control echo mismatch for ") + name +
                    ": expected=" + std::to_string(expected) +
                    ", actual=" + std::to_string(actual));
        };
        bad(deCmd,u[0],"elevator-cmd-norm");
        bad(dePosNorm,u[0],"elevator-pos-norm");
        bad(throttleCmd,u[1],"throttle-cmd-norm");
        bad(throttlePos,u[1],"throttle-pos-norm");
    }

    void setEngineState(double n1, double n2)
    {
        bool ok1 = set("propulsion/engine[0]/n1", n1, false);
        bool ok2 = set("propulsion/engine[0]/n2", n2, false);
        if (!ok1) ok1 = set("propulsion/engine/n1", n1, false);
        if (!ok2) ok2 = set("propulsion/engine/n2", n2, false);
        if (!ok1 || !ok2)
            throw std::runtime_error("N1/N2 are unavailable or not writable; cannot form engine-augmented state.");
        const double r1=readN1(std::numeric_limits<double>::quiet_NaN());
        const double r2=readN2(std::numeric_limits<double>::quiet_NaN());
        if (!std::isfinite(r1) || !std::isfinite(r2) || std::abs(r1-n1)>1e-9 || std::abs(r2-n2)>1e-9)
            throw std::runtime_error("N1/N2 write/readback verification failed; engine state is not controllably reconstructible.");
    }

    void setElevator(double e)
    {
        if (!(e >= -1.0 && e <= 1.0)) throw std::runtime_error("elevator must be in [-1,1].");
        auto fcs=fdm_->GetFCS();
        if (!fcs) throw std::runtime_error("FCS unavailable while setting elevator.");
        fcs->SetDeCmd(e);
        // Match the existing AirdropX plant semantics: the command and normalized
        // surface position are both imposed, so the physics oracle does not add an
        // actuator state that the Simulink plant itself does not currently model.
        set("fcs/elevator-pos-norm", e);
        if (std::abs(fcs->GetDeCmd()-e)>1e-12)
            throw std::runtime_error("Elevator command write/readback verification failed.");
    }

    void setThrottle(double t)
    {
        if (!(t >= 0.0 && t <= 1.0)) throw std::runtime_error("throttle must be in [0,1].");
        auto fcs = fdm_->GetFCS();
        auto prop = fdm_->GetPropulsion();
        if (!fcs || !prop) throw std::runtime_error("FCS/propulsion unavailable while setting throttle.");
        if (prop->GetNumEngines() == 0) throw std::runtime_error("Aircraft has no propulsion engine.");

        // Set both command and actual FCS position for every engine. FGPropulsion
        // owns a separate public input cache used directly by Engine::Calculate(),
        // so synchronize that cache explicitly as well.
        fcs->SetThrottleCmd(-1,t);
        fcs->SetThrottlePos(-1,t);
        if (prop->in.ThrottleCmd.size() < prop->GetNumEngines() ||
            prop->in.ThrottlePos.size() < prop->GetNumEngines())
            throw std::runtime_error("FGPropulsion throttle input vectors are not initialized.");
        for (size_t i=0; i<prop->GetNumEngines(); ++i) {
            prop->in.ThrottleCmd[i]=t;
            prop->in.ThrottlePos[i]=t;
        }
    }

    void establishEngineSteadyState(double throttle)
    {
        auto prop = fdm_->GetPropulsion();
        if (!prop) throw std::runtime_error("Propulsion unavailable.");
        prop->InitRunning(-1);       // engine-specific running initialization
        setThrottle(throttle);       // undo InitRunning's forced 1.0 throttle
        (void)prop->GetSteadyState();// converge hidden propulsion states at requested throttle
        setThrottle(throttle);       // leave FCS + propulsion cache exactly synchronized
    }

    double getThrottle(double fallback) const
    {
        auto fcs = fdm_->GetFCS();
        double t = fallback;
        if (fcs && fdm_->GetPropulsion() && fdm_->GetPropulsion()->GetNumEngines()>0)
            t=fcs->GetThrottleCmd(0);
        if (!std::isfinite(t)) t=fallback;
        return std::max(0.0,std::min(1.0,t));
    }

    double readN1(double fallback) const
    {
        return get("propulsion/engine[0]/n1", get("propulsion/engine/n1", fallback));
    }

    double readN2(double fallback) const
    {
        return get("propulsion/engine[0]/n2", get("propulsion/engine/n2", fallback));
    }

    std::vector<double> readState(double n1Fallback, double n2Fallback) const
    {
        std::vector<double> xn(kStateN,0.0);
        xn[0] = get("position/h-agl-ft", 0.0)*kFtToM;
        xn[1] = get("velocities/vtrue-fps", 0.0)*kFtToM;
        xn[2] = get("flight-path/gamma-rad", 0.0);
        xn[3] = get("attitude/theta-rad", get("attitude/theta-deg",0.0)*kDegToRad);
        xn[4] = get("velocities/q-rad_sec", 0.0);
        xn[5] = readN1(n1Fallback);
        xn[6] = readN2(n2Fallback);
        for (double v : xn) {
            if (!std::isfinite(v)) throw std::runtime_error("Oracle produced non-finite state.");
        }
        return xn;
    }

    mxArray* diagnostics() const
    {
        const char* f[] = {"mass_kg","cg_x_m","Iyy_kgm2","alpha_rad","alphadot_radps","gamma_rad","theta_rad",
                           "phi_rad","q_radps","p_radps","r_radps",
                           "udot_mps2","wdot_mps2","qdot_radps2","fbx_N","fbz_N","my_Nm","thrust_N",
                           "elevator_cmd_norm","elevator_pos_norm","elevator_pos_rad",
                           "throttle_cmd_norm","throttle_pos_norm",
                           "n1","n2","fuel_flow_kgps","qbar_Pa","fuel_kg","fuel_frozen",
                           "algebraic_settle_iterations","algebraic_settle_error","algebraic_settle_converged"};
        mxArray* s=mxCreateStructMatrix(1,1,32,f);
        auto put=[&](const char* n,double v){mxSetField(s,0,n,mxCreateDoubleScalar(v));};
        put("mass_kg", get("inertia/mass-slugs",0.0)*kSlugToKg);
        put("cg_x_m", get("inertia/cg-x-in",0.0)*0.0254);
        put("Iyy_kgm2", get("inertia/iyy-slugs_ft2",0.0)*kSlugFt2ToKgM2);
        put("alpha_rad", get("aero/alpha-rad",0.0));
        put("alphadot_radps", get("aero/alphadot-rad_sec",std::numeric_limits<double>::quiet_NaN()));
        put("gamma_rad", get("flight-path/gamma-rad",0.0));
        put("theta_rad", get("attitude/theta-rad", get("attitude/theta-deg",0.0)*kDegToRad));
        put("phi_rad", get("attitude/phi-rad", get("attitude/phi-deg",0.0)*kDegToRad));
        put("q_radps", get("velocities/q-rad_sec",0.0));
        put("p_radps", get("velocities/p-rad_sec",0.0));
        put("r_radps", get("velocities/r-rad_sec",0.0));
        put("udot_mps2", get("accelerations/udot-ft_sec2",0.0)*kFtToM);
        put("wdot_mps2", get("accelerations/wdot-ft_sec2",0.0)*kFtToM);
        put("qdot_radps2", get("accelerations/qdot-rad_sec2",0.0));
        put("fbx_N", get("forces/fbx-total-lbs",0.0)*kLbfToN);
        put("fbz_N", get("forces/fbz-total-lbs",0.0)*kLbfToN);
        put("my_Nm", get("moments/m-total-lbsft",0.0)*kLbfFtToNm);
        put("thrust_N", get("propulsion/engine[0]/thrust-lbs", get("propulsion/engine/thrust-lbs",0.0))*kLbfToN);
        auto fcs = fdm_->GetFCS();
        put("elevator_cmd_norm", fcs ? fcs->GetDeCmd() : get("fcs/elevator-cmd-norm",0.0));
        put("elevator_pos_norm", get("fcs/elevator-pos-norm",0.0));
        put("elevator_pos_rad", get("fcs/elevator-pos-rad",0.0));
        put("throttle_cmd_norm", (fcs && fdm_->GetPropulsion() && fdm_->GetPropulsion()->GetNumEngines()>0) ? fcs->GetThrottleCmd(0) : 0.0);
        put("throttle_pos_norm", (fcs && fdm_->GetPropulsion() && fdm_->GetPropulsion()->GetNumEngines()>0) ? fcs->GetThrottlePos(0) : 0.0);
        put("n1", readN1(0.0));
        put("n2", readN2(0.0));
        put("fuel_flow_kgps", get("propulsion/engine[0]/fuel-flow-rate-pps", get("propulsion/engine/fuel-flow-rate-pps",0.0))*kPpsToKgps);
        put("qbar_Pa", get("aero/qbar-psf",0.0)*47.88025898);
        auto prop=fdm_->GetPropulsion();
        double fuelLbs=0.0;
        if (prop) for (size_t i=0;i<prop->GetNumTanks();++i) fuelLbs += prop->GetTank(static_cast<unsigned int>(i))->GetContents();
        put("fuel_kg",fuelLbs*0.45359237);
        put("fuel_frozen",prop && prop->GetFuelFreeze() ? 1.0 : 0.0);
        put("algebraic_settle_iterations",static_cast<double>(lastAlgebraicIterations_));
        put("algebraic_settle_error",lastAlgebraicError_);
        put("algebraic_settle_converged",lastAlgebraicConverged_ ? 1.0 : 0.0);
        return s;
    }

    std::string projectRoot_, aircraftName_, icName_;
    double dt_;
    std::unique_ptr<JSBSim::FGFDMExec> fdm_;
    std::vector<double> pointMassLbs_, fuelLbs_;
    double baseN1_=80.0, baseN2_=80.0;
    int lastAlgebraicIterations_=0;
    double lastAlgebraicError_=std::numeric_limits<double>::infinity();
    bool lastAlgebraicConverged_=false;
};

std::unique_ptr<Oracle> gOracle;
void cleanup() { gOracle.reset(); }

} // namespace

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
    try {
        if (nrhs < 1) throw std::runtime_error("First argument must be a command.");
        const std::string cmd = mxToString(prhs[0]);

        if (cmd == "init") {
            if (nrhs != 5) throw std::runtime_error("init(projectRoot, aircraftName, icName, dt)");
            gOracle = std::make_unique<Oracle>(mxToString(prhs[1]),mxToString(prhs[2]),mxToString(prhs[3]),scalar(prhs[4],"dt"));
            mexAtExit(cleanup);
            if (nlhs > 0) plhs[0]=gOracle->info();
            return;
        }
        if (cmd == "close") { gOracle.reset(); return; }
        if (cmd == "info") {
            if (!gOracle) throw std::runtime_error("Oracle not initialized.");
            if (nlhs > 0) plhs[0]=gOracle->info();
            return;
        }
        if (cmd == "eval") {
            if (!gOracle) throw std::runtime_error("Oracle not initialized.");
            if (nrhs != 6) throw std::runtime_error("eval(x,u,cfgId,fuelScale,Ts)");
            auto x=vectorN(prhs[1],kStateN,"x");
            auto u=vectorN(prhs[2],kInputN,"u");
            const double cfgRaw=scalar(prhs[3],"cfgId");
            if (std::abs(cfgRaw-std::round(cfgRaw))>1e-12) throw std::runtime_error("cfgId must be an integer.");
            int cfg=static_cast<int>(std::llround(cfgRaw));
            double fuel=scalar(prhs[4],"fuelScale");
            double Ts=scalar(prhs[5],"Ts");
            auto out=gOracle->eval(x,u,cfg,fuel,Ts);
            if (nlhs > 0) plhs[0]=vectorMx(out.first);
            if (nlhs > 1) plhs[1]=out.second; else mxDestroyArray(out.second);
            return;
        }
        if (cmd == "steady_eval") {
            if (!gOracle) throw std::runtime_error("Oracle not initialized.");
            if (nrhs != 9) throw std::runtime_error("steady_eval(H,V,theta,elevator,throttle,cfgId,fuelScale,Ts)");
            const double H=scalar(prhs[1],"H");
            const double V=scalar(prhs[2],"V");
            const double theta=scalar(prhs[3],"theta");
            const double elevator=scalar(prhs[4],"elevator");
            const double throttle=scalar(prhs[5],"throttle");
            const double cfgRaw=scalar(prhs[6],"cfgId");
            if (std::abs(cfgRaw-std::round(cfgRaw))>1e-12) throw std::runtime_error("cfgId must be an integer.");
            const int cfg=static_cast<int>(std::llround(cfgRaw));
            const double fuel=scalar(prhs[7],"fuelScale");
            const double Ts=scalar(prhs[8],"Ts");
            mxArray* out=gOracle->steadyEval(H,V,theta,elevator,throttle,cfg,fuel,Ts);
            if (nlhs>0) plhs[0]=out; else mxDestroyArray(out);
            return;
        }
        if (cmd == "builtin_trim") {
            if (!gOracle) throw std::runtime_error("Oracle not initialized.");
            if (nrhs != 6) throw std::runtime_error("builtin_trim(H,V,cfgId,fuelScale,zSeed)");
            const double H=scalar(prhs[1],"H");
            const double V=scalar(prhs[2],"V");
            const double cfgRaw=scalar(prhs[3],"cfgId");
            if (std::abs(cfgRaw-std::round(cfgRaw))>1e-12) throw std::runtime_error("cfgId must be an integer.");
            const int cfg=static_cast<int>(std::llround(cfgRaw));
            const double fuel=scalar(prhs[4],"fuelScale");
            const auto seed=vectorN(prhs[5],kTrimSeedN,"zSeed");
            mxArray* out=gOracle->builtinTrim(H,V,cfg,fuel,seed);
            if (nlhs>0) plhs[0]=out; else mxDestroyArray(out);
            return;
        }
        if (cmd == "version") {
            if (nlhs>0) plhs[0]=mxCreateString("AirdropX JSBSim physics oracle v0.3.3");
            return;
        }
        throw std::runtime_error("Unknown command: " + cmd);
    } catch (const std::exception& e) {
        mexErrMsgIdAndTxt("AirdropX:PhysMPC:Oracle", "%s", e.what());
    }
}
