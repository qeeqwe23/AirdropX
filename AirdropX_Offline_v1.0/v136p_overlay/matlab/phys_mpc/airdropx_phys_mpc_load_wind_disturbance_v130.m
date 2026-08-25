function sched=airdropx_phys_mpc_load_wind_disturbance_v130(sched,calibrationPath)
%AIRDROPX_PHYS_MPC_LOAD_WIND_DISTURBANCE_V130 Attach calibrated Gw to all cfg controllers.
arguments
    sched (1,1) struct
    calibrationPath (1,1) string
end
if ~isfile(calibrationPath), error("AirdropX:WindMPC:MissingCalibration","Missing %s",calibrationPath); end
S=load(calibrationPath,"W"); W=S.W;
if ~isfield(W,"pass") || ~W.pass || ~contains(string(W.version),"v1.3.0")
    error("AirdropX:WindMPC:BadCalibration","Wind disturbance calibration is invalid.");
end
if abs(W.H-sched.H)>1e-9 || abs(W.V-sched.V)>1e-9 || abs(W.FuelScale-sched.fuelScale)>1e-9
    error("AirdropX:WindMPC:CalibrationPointMismatch","Calibration H/V/fuel does not match scheduled mission.");
end
for cfg=0:4
    C=W.cfg{cfg+1};
    if C.cfg~=cfg, error("AirdropX:WindMPC:CalibrationCfgMismatch","Expected cfg%d calibration.",cfg); end
    sched.models{cfg+1}.ctrl=airdropx_phys_mpc_enable_disturbance_v130(sched.models{cfg+1}.ctrl,double(C.Gw(:)));
    sched.models{cfg+1}.wind_calibration=C;
end
sched.wind_calibration=W;
end
