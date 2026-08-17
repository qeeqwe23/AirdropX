#include "mex.h"

#include <JSBSim/FGFDMExec.h>
#include <JSBSim/initialization/FGInitialCondition.h>
#include <JSBSim/simgear/misc/sg_path.hxx>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr double kFtToM = 0.3048;
constexpr double kLbToKg = 0.45359237;
constexpr double kSlugFt2ToKgM2 = 1.3558179483314004;
constexpr double kLbfToN = 4.4482216152605;
constexpr double kLbfFtToNm = 1.3558179483314004;
constexpr double kPpsToKgps = 0.45359237;
constexpr int kStateN = 7;
constexpr int kInputN = 2;

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
    return mxGetScalar(a);
}

std::vector<double> vectorN(const mxArray* a, int n, const char* name)
{
    if (!mxIsDouble(a) || mxIsComplex(a) || static_cast<int>(mxGetNumberOfElements(a)) != n)
        throw std::runtime_error(std::string(name) + " has wrong size.");
    const double* p = mxGetPr(a);
    return std::vector<double>(p, p+n);
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
        fdm_->SetRootDir(SGPath(projectRoot_));
        fdm_->SetAircraftPath(SGPath("aircraft"));
        fdm_->SetEnginePath(SGPath("engine"));
        fdm_->SetSystemsPath(SGPath("systems"));
        if (!fdm_->LoadModel(aircraftName_))
            throw std::runtime_error("JSBSim LoadModel failed: " + aircraftName_);
        fdm_->Setdt(dt_);
        if (!icName_.empty()) {
            if (!fdm_->GetIC()->Load(SGPath(icName_), false))
                throw std::runtime_error("JSBSim IC load failed: " + icName_);
        }
        if (!fdm_->RunIC()) throw std::runtime_error("JSBSim RunIC failed.");

        pointMassLbs_.clear();
        for (int i=0; i<4; ++i) {
            pointMassLbs_.push_back(get("inertia/pointmass-weight-lbs[" + std::to_string(i) + "]", 0.0));
        }
        fuelLbs_.clear();
        for (int i=0; i<16; ++i) {
            const std::string p = "propulsion/tank[" + std::to_string(i) + "]/contents-lbs";
            if (!propertyExists(p)) break;
            fuelLbs_.push_back(get(p, 0.0));
        }

        // Reproduce the existing project convention: bring engines to a valid running state once.
        set("propulsion/set-running", -1.0, false);
        setThrottle(0.80);
        for (int i=0; i<300; ++i) {
            if (!fdm_->Run()) throw std::runtime_error("Engine warmup failed.");
        }
        baseN1_ = get("propulsion/engine[0]/n1", get("propulsion/engine/n1", 80.0));
        baseN2_ = get("propulsion/engine[0]/n2", get("propulsion/engine/n2", 80.0));
        restoreBaseIC();
    }

    std::pair<std::vector<double>, mxArray*> eval(const std::vector<double>& x,
                                                   const std::vector<double>& u,
                                                   int cfgId, double fuelScale,
                                                   double Ts)
    {
        if (cfgId < 0 || cfgId > 4) throw std::runtime_error("cfgId must be 0..4.");
        if (!(fuelScale >= 0.0 && fuelScale <= 1.2)) throw std::runtime_error("fuelScale must be in [0,1.2].");
        if (!(Ts > 0.0)) throw std::runtime_error("Ts must be positive.");
        const double nReal = Ts/dt_;
        const int nSteps = static_cast<int>(std::llround(nReal));
        if (nSteps < 1 || std::abs(nReal - nSteps) > 1e-9)
            throw std::runtime_error("Ts must be an integer multiple of oracle dt.");

        setIC(x);
        if (!fdm_->RunIC()) throw std::runtime_error("RunIC failed during oracle eval.");

        // RunIC may reset fuel/engine internals. Apply cfg/fuel and engine states afterwards
        // so the first dynamic substep sees exactly the requested scheduling point.
        applyMassConfiguration(cfgId, fuelScale);
        set("propulsion/set-running", -1.0, false);
        setEngineState(x[5], x[6]);
        setElevator(u[0]);
        setThrottle(u[1]);

        for (int k=0; k<nSteps; ++k) {
            if (!fdm_->Run()) throw std::runtime_error("JSBSim Run failed during oracle eval.");
        }

        std::vector<double> xn(kStateN,0.0);
        xn[0] = get("position/h-agl-ft", 0.0)*kFtToM;
        xn[1] = get("velocities/vtrue-fps", 0.0)*kFtToM;
        xn[2] = get("flight-path/gamma-rad", 0.0);
        xn[3] = get("attitude/theta-rad", get("attitude/theta-deg",0.0)*3.14159265358979323846/180.0);
        xn[4] = get("velocities/q-rad_sec", 0.0);
        xn[5] = get("propulsion/engine[0]/n1", get("propulsion/engine/n1", x[5]));
        xn[6] = get("propulsion/engine[0]/n2", get("propulsion/engine/n2", x[6]));

        return {xn, diagnostics()};
    }

    mxArray* info() const
    {
        const char* fields[] = {"dt","base_n1","base_n2","pointmass_lbs","fuel_lbs","state_definition","input_definition"};
        mxArray* s = mxCreateStructMatrix(1,1,7,fields);
        mxSetField(s,0,"dt",mxCreateDoubleScalar(dt_));
        mxSetField(s,0,"base_n1",mxCreateDoubleScalar(baseN1_));
        mxSetField(s,0,"base_n2",mxCreateDoubleScalar(baseN2_));
        mxArray* pm=mxCreateDoubleMatrix(pointMassLbs_.size(),1,mxREAL);
        std::copy(pointMassLbs_.begin(),pointMassLbs_.end(),mxGetPr(pm));
        mxSetField(s,0,"pointmass_lbs",pm);
        mxArray* fu=mxCreateDoubleMatrix(fuelLbs_.size(),1,mxREAL);
        std::copy(fuelLbs_.begin(),fuelLbs_.end(),mxGetPr(fu));
        mxSetField(s,0,"fuel_lbs",fu);
        mxSetField(s,0,"state_definition",mxCreateString("[h_m Va_mps gamma_rad theta_rad q_radps N1 N2]"));
        mxSetField(s,0,"input_definition",mxCreateString("[elevator_abs_norm throttle_norm]"));
        return s;
    }

private:
    bool propertyExists(const std::string& p) const
    {
        try { (void)fdm_->GetPropertyValue(p); return true; }
        catch (...) { return false; }
    }

    double get(const std::string& p, double fallback) const
    {
        try { return fdm_->GetPropertyValue(p); }
        catch (...) { return fallback; }
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
        if (!icName_.empty()) {
            if (!fdm_->GetIC()->Load(SGPath(icName_), false))
                throw std::runtime_error("JSBSim IC reload failed.");
        }
        if (!fdm_->RunIC()) throw std::runtime_error("RunIC after warmup failed.");
    }

    void applyMassConfiguration(int cfgId, double fuelScale)
    {
        for (int i=0; i<4; ++i) {
            const double w = (i < cfgId) ? 0.0 : pointMassLbs_.at(i);
            set("inertia/pointmass-weight-lbs[" + std::to_string(i) + "]", w);
        }
        for (size_t i=0; i<fuelLbs_.size(); ++i) {
            set("propulsion/tank[" + std::to_string(i) + "]/contents-lbs", fuelScale*fuelLbs_[i]);
        }
    }

    void setIC(const std::vector<double>& x)
    {
        // Planar longitudinal state. alpha = theta - gamma is enforced kinematically.
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

    void setEngineState(double n1, double n2)
    {
        bool ok1 = set("propulsion/engine[0]/n1", n1, false);
        bool ok2 = set("propulsion/engine[0]/n2", n2, false);
        if (!ok1) ok1 = set("propulsion/engine/n1", n1, false);
        if (!ok2) ok2 = set("propulsion/engine/n2", n2, false);
        if (!ok1 || !ok2) throw std::runtime_error("N1/N2 are unavailable or not writable; cannot form a Markov engine-augmented state.");
    }

    void setElevator(double e)
    {
        e=std::max(-1.0,std::min(1.0,e));
        set("fcs/elevator-cmd-norm", e, false);
        set("fcs/elevator-pos-norm", e, false);
    }

    void setThrottle(double t)
    {
        t=std::max(0.0,std::min(1.0,t));
        set("fcs/throttle-cmd-norm", t, false);
        set("fcs/throttle-pos-norm", t, false);
        set("propulsion/engine[0]/throttle-cmd-norm", t, false);
        set("propulsion/engine/throttle-cmd-norm", t, false);
    }

    mxArray* diagnostics() const
    {
        const char* f[] = {"mass_kg","cg_x_m","Iyy_kgm2","alpha_rad","gamma_rad","theta_rad","q_radps",
                           "udot_mps2","wdot_mps2","qdot_radps2","fbx_N","fbz_N","my_Nm","thrust_N",
                           "n1","n2","fuel_flow_kgps","qbar_Pa"};
        mxArray* s=mxCreateStructMatrix(1,1,18,f);
        auto put=[&](const char* n,double v){mxSetField(s,0,n,mxCreateDoubleScalar(v));};
        put("mass_kg", get("inertia/mass-slugs",0.0)*14.5939029372);
        put("cg_x_m", get("inertia/cg-x-in",0.0)*0.0254);
        put("Iyy_kgm2", get("inertia/iyy-slugs_ft2",0.0)*kSlugFt2ToKgM2);
        put("alpha_rad", get("aero/alpha-rad",0.0));
        put("gamma_rad", get("flight-path/gamma-rad",0.0));
        put("theta_rad", get("attitude/theta-rad", get("attitude/theta-deg",0.0)*3.14159265358979323846/180.0));
        put("q_radps", get("velocities/q-rad_sec",0.0));
        put("udot_mps2", get("accelerations/udot-ft_sec2",0.0)*kFtToM);
        put("wdot_mps2", get("accelerations/wdot-ft_sec2",0.0)*kFtToM);
        put("qdot_radps2", get("accelerations/qdot-rad_sec2",0.0));
        put("fbx_N", get("forces/fbx-total-lbs",0.0)*kLbfToN);
        put("fbz_N", get("forces/fbz-total-lbs",0.0)*kLbfToN);
        put("my_Nm", get("moments/m-total-lbsft",0.0)*kLbfFtToNm);
        put("thrust_N", get("propulsion/engine[0]/thrust-lbs", get("propulsion/engine/thrust-lbs",0.0))*kLbfToN);
        put("n1", get("propulsion/engine[0]/n1", get("propulsion/engine/n1",0.0)));
        put("n2", get("propulsion/engine[0]/n2", get("propulsion/engine/n2",0.0)));
        put("fuel_flow_kgps", get("propulsion/engine[0]/fuel-flow-rate-pps", get("propulsion/engine/fuel-flow-rate-pps",0.0))*kPpsToKgps);
        put("qbar_Pa", get("aero/qbar-psf",0.0)*47.88025898);
        return s;
    }

    std::string projectRoot_, aircraftName_, icName_;
    double dt_;
    std::unique_ptr<JSBSim::FGFDMExec> fdm_;
    std::vector<double> pointMassLbs_, fuelLbs_;
    double baseN1_=80.0, baseN2_=80.0;
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
            int cfg=static_cast<int>(std::llround(scalar(prhs[3],"cfgId")));
            double fuel=scalar(prhs[4],"fuelScale");
            double Ts=scalar(prhs[5],"Ts");
            auto out=gOracle->eval(x,u,cfg,fuel,Ts);
            if (nlhs > 0) {
                plhs[0]=mxCreateDoubleMatrix(kStateN,1,mxREAL);
                std::copy(out.first.begin(),out.first.end(),mxGetPr(plhs[0]));
            }
            if (nlhs > 1) plhs[1]=out.second; else mxDestroyArray(out.second);
            return;
        }
        if (cmd == "version") {
            if (nlhs>0) plhs[0]=mxCreateString("AirdropX JSBSim discrete physics oracle v0.2");
            return;
        }
        throw std::runtime_error("Unknown command: " + cmd);
    } catch (const std::exception& e) {
        mexErrMsgIdAndTxt("AirdropX:PhysMPC:Oracle", "%s", e.what());
    }
}
