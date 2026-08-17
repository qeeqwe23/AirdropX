function audit=airdropx_phys_oracle_config_audit(x,u,p,info,opts)
%AIRDROPX_PHYS_ORACLE_CONFIG_AUDIT Verify cfg/fuel scheduling reaches JSBSim mass balance.
% This is a structural smoke gate, not a flight-performance test. It verifies
% that cfg0..4 remove the intended point masses and that fuelScale changes the
% physical JSBSim mass by the corresponding tank-content amount. Fuel is frozen
% inside each local oracle map, because fuel/mass are LPV scheduling parameters.
arguments
    x (7,1) double {mustBeFinite}
    u (2,1) double {mustBeFinite}
    p (1,1) struct
    info (1,1) struct
    opts.MassAbsTol_kg (1,1) double {mustBePositive} = 0.02
    opts.FuelProbeFraction (1,1) double {mustBePositive} = 0.10
end
assert(opts.FuelProbeFraction<=0.25,"FuelProbeFraction must be <= 0.25.");
localRequire(p,{'cfgId','Ts'});
if ~isfield(p,"fuelScale") || isempty(p.fuelScale), p.fuelScale=1.0; end
localRequire(info,{'pointmass_lbs','fuel_lbs'});
pm=double(info.pointmass_lbs(:)); fuel=double(info.fuel_lbs(:));
if numel(pm)~=4 || any(~isfinite(pm)) || any(pm<=0)
    error("AirdropX:PhysMPC:BadPointMassBaseline", ...
        "Oracle must expose exactly four positive cargo point masses for cfg0..4.");
end
if isempty(fuel) || any(~isfinite(fuel)) || any(fuel<0)
    error("AirdropX:PhysMPC:BadFuelBaseline","Oracle fuel baseline is invalid.");
end
lb2kg=0.45359237;

mass=zeros(5,1); cg=zeros(5,1); Iyy=zeros(5,1); frozen=false(5,1);
diags=cell(5,1);
for cfg=0:4
    pc=p; pc.cfgId=cfg;
    [~,d]=airdropx_phys_step(x,u,pc);
    diags{cfg+1}=d;
    mass(cfg+1)=d.mass_kg; cg(cfg+1)=d.cg_x_m; Iyy(cfg+1)=d.Iyy_kgm2;
    frozen(cfg+1)=isfield(d,"fuel_frozen") && logical(d.fuel_frozen);
end
expected=mass(1)-[0;cumsum(pm(1:4))]*lb2kg;
cfgErr=mass-expected;
cfgPass=all(isfinite([mass;cg;Iyy])) && all(frozen) && ...
    max(abs(cfgErr))<=opts.MassAbsTol_kg && all(diff(mass)<0);

fuelProbe=struct("checked",false,"pass",true,"scale",NaN,"expected_delta_kg",NaN, ...
    "actual_delta_kg",NaN,"error_kg",NaN,"diag",struct());
if p.fuelScale>0 && sum(fuel)>0
    probeScale=p.fuelScale*(1-opts.FuelProbeFraction);
    pp=p; pp.cfgId=0; pp.fuelScale=probeScale;
    [~,df]=airdropx_phys_step(x,u,pp);
    expectedDelta=(p.fuelScale-probeScale)*sum(fuel)*lb2kg;
    actualDelta=mass(1)-df.mass_kg;
    e=actualDelta-expectedDelta;
    fuelProbe=struct("checked",true,"pass",isfinite(e) && abs(e)<=opts.MassAbsTol_kg && ...
        isfield(df,"fuel_frozen") && logical(df.fuel_frozen), ...
        "scale",probeScale,"expected_delta_kg",expectedDelta, ...
        "actual_delta_kg",actualDelta,"error_kg",e,"diag",df);
end

audit=struct("cfg_mass_kg",mass,"cfg_expected_mass_kg",expected,"cfg_mass_error_kg",cfgErr, ...
    "cfg_cg_x_m",cg,"cfg_Iyy_kgm2",Iyy,"cfg_fuel_frozen",frozen,"cfg_diags",{diags}, ...
    "fuel_probe",fuelProbe,"cfg_pass",cfgPass,"pass",cfgPass && fuelProbe.pass);
if ~audit.pass
    error("AirdropX:PhysMPC:ConfigurationAuditFailed", ...
        "cfg/fuel mass audit failed: cfgPass=%d maxCfgMassErr=%.6g kg fuelPass=%d fuelErr=%.6g kg.", ...
        cfgPass,max(abs(cfgErr)),fuelProbe.pass,fuelProbe.error_kg);
end
end

function localRequire(s,names)
for k=1:numel(names)
    n=char(names{k});
    if ~isfield(s,n) || isempty(s.(n))
        error("AirdropX:PhysMPC:MissingField","Required field %s is missing.",n);
    end
end
end
