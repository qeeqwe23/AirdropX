function r=airdropx_phys_wind_oracle_selftest_v121(projectRoot,bankPath)
%AIRDROPX_PHYS_WIND_ORACLE_SELFTEST_V121 Runtime smoke before precision missions.
arguments
    projectRoot (1,1) string
    bankPath (1,1) string = ""
end
if bankPath=="", bankPath=fullfile(projectRoot,"matlab","results","physics_mpc_v033","physics_bank.mat"); end
sched=airdropx_phys_mpc_build_cfg_schedule(bankPath,200,50,1.0,Horizon=100);
info=airdropx_phys_wind_oracle_init_v121(projectRoot); %#ok<NASGU>
c=onCleanup(@()airdropx_jsbsim_wind_oracle_mex("close")); %#ok<NASGU>
x0=sched.models{1}.ctrl.xref; u0=sched.models{1}.ctrl.uref;
d0=airdropx_jsbsim_wind_oracle_mex("start_continuous",x0,u0,0,1.0,0.0);
x=x0; maxNorm=0; last=d0;
for k=1:10
    [x,last]=airdropx_jsbsim_wind_oracle_mex("step_continuous",u0,0,1.0,0.0,0.1);
    e=airdropx_phys_mpc_state_error(x,x0); maxNorm=max(maxNorm,max(abs(e)./sched.stateScale));
end
mass0=double(last.mass_kg);
[x1,d1]=airdropx_jsbsim_wind_oracle_mex("step_continuous",sched.models{2}.ctrl.uref,1,1.0,5.0,0.1); %#ok<ASGLU>
massDrop=mass0-double(d1.mass_kg);
% v1.3.5 regression: a cfg transition followed by only one 1/120-s base
% substep must already expose the final mass/cg/Iyy for the new payload cfg.
dt=1/120; [x2,d2]=airdropx_jsbsim_wind_oracle_mex("step_continuous",sched.models{3}.ctrl.uref,2,1.0,5.0,dt); %#ok<ASGLU>
dd2=sched.models{3}.vertex.trim.diag;
iyyErr=abs(double(d2.Iyy_kgm2)-double(dd2.Iyy_kgm2));
massErr=abs(double(d2.mass_kg)-double(dd2.mass_kg));
cgErr=abs(double(d2.cg_x_m)-double(dd2.cg_x_m));
refreshOk=isfield(d2,"mass_refresh_converged") && double(d2.mass_refresh_converged)==1;
r=struct(); r.version=string(airdropx_jsbsim_wind_oracle_mex("version")); r.zero_wind_1s_max_norm=maxNorm;
r.wind_echo_mps=double(d1.wind_long_mps); r.crosswind_echo_mps=double(d1.wind_cross_mps); r.mass_drop_kg=massDrop;
r.short_transition_mass_error_kg=massErr; r.short_transition_cg_error_m=cgErr; r.short_transition_Iyy_error_kgm2=iyyErr; r.mass_refresh_converged=refreshOk;
r.pass=maxNorm<0.25 && abs(r.wind_echo_mps-5)<1e-9 && abs(r.crosswind_echo_mps)<1e-9 && massDrop>250 && massDrop<350 && all(isfinite(x1)) && all(isfinite(x2)) && massErr<=1e-2 && cgErr<=1e-5 && iyyErr<=1e-2 && refreshOk;
if ~r.pass
    error("AirdropX:WindAirdrop:WindOracleSelftestFailed","Wind oracle self-test failed: norm=%.4g wind=%.4g cross=%.4g drop=%.4g kg shortIyyErr=%.4g refresh=%d",maxNorm,r.wind_echo_mps,r.crosswind_echo_mps,massDrop,iyyErr,refreshOk);
end
end
