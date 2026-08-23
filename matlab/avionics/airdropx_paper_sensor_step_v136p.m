function [s,o]=airdropx_paper_sensor_step_v136p(s,truth)
%AIRDROPX_PAPER_SENSOR_STEP_V136P Unbiased simulated sensors + causal estimate.
% `truth` is plant-side stimulus only. Output `o` is the only onboard data path.
req=["pos_n_m","h_m","Va_mps","theta_rad","q_radps","N1","N2","Vg_long_mps","Vz_up_mps"];
for i=1:numel(req)
    if ~isfield(truth,req(i)), error("AirdropX:PaperSensor:MissingTruthStimulus","Missing truth.%s",req(i)); end
end
if ~isstruct(s) || ~isfield(s,"version") || ~contains(string(s.version),"v1.3.6-Paper")
    error("AirdropX:PaperSensor:BadState","Initialize with airdropx_paper_sensor_init_v136p.");
end
Ts=s.Ts; p=s.opts; s.sample_count=s.sample_count+1;
% Only zero-mean white measurement noise is modeled in the paper baseline.
m=struct();
m.gnss_pos_n_m=double(truth.pos_n_m)+p.SigmaGnssPos_m*randn;
m.gnss_vg_long_mps=double(truth.Vg_long_mps)+p.SigmaGnssGroundspeed_mps*randn;
m.gnss_vz_up_mps=double(truth.Vz_up_mps)+p.SigmaGnssVerticalSpeed_mps*randn;
m.baro_h_m=double(truth.h_m)+p.SigmaBaroAltitude_m*randn;
m.pitot_Va_mps=max(0.1,double(truth.Va_mps)+p.SigmaAirspeed_mps*randn);
m.ahrs_theta_rad=double(truth.theta_rad)+p.SigmaPitch_rad*randn;
m.gyro_q_radps=double(truth.q_radps)+p.SigmaPitchRate_radps*randn;
m.engine_N1=double(truth.N1)+p.SigmaN1*randn;
m.engine_N2=double(truth.N2)+p.SigmaN2*randn;
if ~s.initialized
    s.est.pos_n_m=m.gnss_pos_n_m; s.est.Vg_long_mps=m.gnss_vg_long_mps; s.est.Vz_up_mps=m.gnss_vz_up_mps;
    s.est.h_m=m.baro_h_m; s.est.Va_mps=m.pitot_Va_mps; s.est.theta_rad=m.ahrs_theta_rad;
    s.est.q_radps=m.gyro_q_radps; s.est.N1=m.engine_N1; s.est.N2=m.engine_N2;
    s.est.gamma_rad=atan2(s.est.Vz_up_mps,max(abs(s.est.Vg_long_mps),1e-3)); s.initialized=true;
else
    aPos=1-exp(-Ts/p.PositionFilterTau_s); aVel=1-exp(-Ts/p.VelocityFilterTau_s);
    aH=1-exp(-Ts/p.AltitudeFilterTau_s); aVa=1-exp(-Ts/p.AirspeedFilterTau_s);
    aAtt=1-exp(-Ts/p.AttitudeFilterTau_s); aEng=1-exp(-Ts/p.EngineFilterTau_s);
    s.est.Vg_long_mps=s.est.Vg_long_mps+aVel*(m.gnss_vg_long_mps-s.est.Vg_long_mps);
    s.est.Vz_up_mps=s.est.Vz_up_mps+aVel*(m.gnss_vz_up_mps-s.est.Vz_up_mps);
    posPred=s.est.pos_n_m+s.est.Vg_long_mps*Ts; s.est.pos_n_m=posPred+aPos*(m.gnss_pos_n_m-posPred);
    hPred=s.est.h_m+s.est.Vz_up_mps*Ts; s.est.h_m=hPred+aH*(m.baro_h_m-hPred);
    s.est.Va_mps=s.est.Va_mps+aVa*(m.pitot_Va_mps-s.est.Va_mps);
    s.est.theta_rad=localAngleBlend(s.est.theta_rad,m.ahrs_theta_rad,aAtt);
    s.est.q_radps=s.est.q_radps+aAtt*(m.gyro_q_radps-s.est.q_radps);
    s.est.N1=s.est.N1+aEng*(m.engine_N1-s.est.N1); s.est.N2=s.est.N2+aEng*(m.engine_N2-s.est.N2);
    gammaMeas=atan2(s.est.Vz_up_mps,max(abs(s.est.Vg_long_mps),1e-3));
    s.est.gamma_rad=localAngleBlend(s.est.gamma_rad,gammaMeas,aVel);
end
xEst=[s.est.h_m;s.est.Va_mps;s.est.gamma_rad;s.est.theta_rad;s.est.q_radps;s.est.N1;s.est.N2];
o=struct(); o.measurement=m; o.x_est=xEst; o.pos_n_m=s.est.pos_n_m; o.Vg_long_mps=s.est.Vg_long_mps; o.Vz_up_mps=s.est.Vz_up_mps;
o.Va_mps=s.est.Va_mps; o.gamma_rad=s.est.gamma_rad; o.theta_rad=s.est.theta_rad; o.q_radps=s.est.q_radps; o.N1=s.est.N1; o.N2=s.est.N2;
end
function y=localAngleBlend(a,b,k)
d=atan2(sin(b-a),cos(b-a)); y=a+k*d;
end
