function cmd = airdropx_physics_mpc_set_reference(targetAltitudeM,targetAirspeedMps)
%AIRDROPX_PHYSICS_MPC_SET_REFERENCE Change H/V command while simulation is running.
% The v32 runtime S-function reads these base-workspace values every 0.1 s.
% Calling this function also disables any pre-programmed dynamic profile so
% the live command immediately becomes authoritative.
h=double(targetAltitudeM);v=double(targetAirspeedMps);
if ~isscalar(h)||~isfinite(h)||~isscalar(v)||~isfinite(v),error('Altitude and airspeed must be finite scalars.');end
if h<20||h>200,warning('AirdropX:PhysicsMPC:AltitudeOutsideCertifiedRange','Hcmd=%.2f m is outside the planned 20..200 m certification range.',h);end
if v<45||v>55,warning('AirdropX:PhysicsMPC:SpeedOutsideCertifiedRange','Vcmd=%.2f m/s is outside the 45..55 m/s model-node envelope.',v);end
assignin('base','airdropx_v32_dynamic_reference_profile',zeros(0,3));
assignin('base','airdropx_target_altitude_m',h);
assignin('base','airdropx_pd_v_ref_mps',v);
cmd=struct('target_altitude_m',h,'target_airspeed_mps',v,'timestamp',datetime('now'));
fprintf('[PHYS-MPC] live reference -> H=%.3f m, Va=%.3f m/s\n',h,v);
end
