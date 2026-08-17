function airdropx_urmpc_set_reference(targetAltitudeM,targetAirspeedMps)
%AIRDROPX_URMPC_SET_REFERENCE Change H/V commands during a running UR-MPC simulation.
validateattributes(targetAltitudeM,{'numeric'},{'scalar','real','finite'});validateattributes(targetAirspeedMps,{'numeric'},{'scalar','real','finite'});
assignin('base','airdropx_target_altitude_m',double(targetAltitudeM));assignin('base','airdropx_pd_v_ref_mps',double(targetAirspeedMps));
fprintf('[UR-MPC v2.0] reference updated: H=%.3f m, Va=%.3f m/s\n',double(targetAltitudeM),double(targetAirspeedMps));
end
