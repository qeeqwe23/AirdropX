function M=airdropx_gui_prepare_model_v140(projectRoot,H,V,fuelScale,policy)
%AIRDROPX_GUI_PREPARE_MODEL_V140 Resolve or build an exact H/V physics+wind model pair.
arguments
    projectRoot (1,1) string
    H (1,1) double {mustBeFinite,mustBePositive}
    V (1,1) double {mustBeFinite,mustBePositive}
    fuelScale (1,1) double {mustBeFinite} = 1.0
    policy (1,1) string = "auto_cache"
end
baseBank=fullfile(projectRoot,"matlab","results","physics_mpc_v033","physics_bank.mat");
baseWind=fullfile(projectRoot,"matlab","results","physics_mpc_v130_wind_disturbance_calibration","wind_disturbance_model_v130.mat");
[bankOK,bankHas]=localBankHas(baseBank,H,V,fuelScale);
windOK=localWindMatches(baseWind,H,V,fuelScale);
if bankOK && bankHas && windOK
    M=struct("bankPath",baseBank,"windPath",baseWind,"source","certified_default","built",false); return
end
if policy=="certified_only"
    error("AirdropX:GUI:NoCertifiedPoint", ...
        "No complete certified H=%.6g V=%.6g fuel=%.4g model+wind pair exists. Switch GUI H/V policy to auto-cache or build the point first.",H,V,fuelScale);
end
cacheRoot=fullfile(projectRoot,"matlab","results","offline_gui_v140_model_cache",sprintf("H%07.2f_V%06.2f_F%04.2f",H,V,fuelScale));
if ~isfolder(cacheRoot), mkdir(cacheRoot); end
cacheBank=fullfile(cacheRoot,"physics_bank.mat"); cacheWind=fullfile(cacheRoot,"wind_disturbance_model_v130.mat");
% Reuse a complete base bank when it already contains the selected exact H/V.
% Only the H/V-specific disturbance calibration then needs to be cached.
if bankOK && bankHas
    selectedBank=baseBank;
else
    selectedBank=cacheBank;
    [~,cacheHas]=localBankHas(cacheBank,H,V,fuelScale);
    if ~cacheHas
        if exist("airdropx_phys_build_bank","file")~=2
            error("AirdropX:GUI:MissingBankBuilder","airdropx_phys_build_bank.m is required for a new H/V point.");
        end
        fprintf("[GUI_MODEL] building exact physics vertices H=%.6g V=%.6g\n",H,V);
        R=airdropx_phys_build_bank(projectRoot,Heights=H,Speeds=V,CfgIds=0:4,FuelScales=fuelScale,OutputRoot=cacheRoot,StopOnFailure=true); %#ok<NASGU>
        [~,cacheHas]=localBankHas(cacheBank,H,V,fuelScale);
        if ~cacheHas, error("AirdropX:GUI:BankBuildFailed","Generated bank does not contain all five exact cfg vertices."); end
    end
end
cacheWindOK=localWindMatches(cacheWind,H,V,fuelScale);
if ~cacheWindOK
    fprintf("[GUI_MODEL] calibrating longitudinal disturbance map H=%.6g V=%.6g\n",H,V);
    airdropx_phys_mpc_wind_disturbance_calibrate_v130(projectRoot,selectedBank,cacheWind,H=H,V=V,FuelScale=fuelScale,Horizon=100);
end
M=struct("bankPath",selectedBank,"windPath",cacheWind,"source","gui_exact_cache","built",true);
end

function [ok,has]=localBankHas(path,H,V,fuel)
ok=isfile(path); has=false; if ~ok, return; end
try
    S=load(path,"rows"); r=S.rows;
    m=abs(r.H_m-H)<=1e-9 & abs(r.V_mps-V)<=1e-9 & abs(r.fuel_scale-fuel)<=1e-12 & ismember(r.cfg,0:4) & logical(r.pass);
    has=sum(m)==5 && numel(unique(r.cfg(m)))==5;
catch, has=false; end
end
function tf=localWindMatches(path,H,V,fuel)
tf=false; if ~isfile(path), return; end
try
    S=load(path,"W"); W=S.W; tf=isfield(W,"pass") && W.pass && abs(W.H-H)<=1e-9 && abs(W.V-V)<=1e-9 && abs(W.FuelScale-fuel)<=1e-12;
catch, tf=false; end
end
