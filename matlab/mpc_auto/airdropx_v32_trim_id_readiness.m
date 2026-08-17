function result = airdropx_v32_trim_id_readiness(varargin)
%AIRDROPX_V32_TRIM_ID_READINESS Long-horizon trim certification using the exact ID preparation path.
%
% This diagnostic/certification run deliberately uses zero excitation and the
% same cfg0->cfgN payload preparation schedule used by ID generation.  A trim
% is considered ID-ready only when its final baseline window has small speed,
% vertical-speed, pitch-rate and drift residuals.  Absolute altitude offset is
% NOT a gate: the open-loop equilibrium may be re-anchored vertically.

opts = local_options(varargin{:});
root = char(string(opts.ProjectRoot));
if strlength(string(root)) == 0
    thisDir = fileparts(mfilename('fullpath'));
    root = fileparts(fileparts(thisDir));
end
addpath(fullfile(root,'matlab'));
addpath(fullfile(root,'matlab','mpc'));
addpath(fullfile(root,'matlab','mpc_auto'));

bank = opts.TrimBank;
if isempty(bank), error('AirdropX:V32:IdReadinessMissingTrimBank','TrimBank is required.'); end
cfg = max(0,min(4,round(double(opts.ConfigId))));
trim = bank(cfg+1);

releasePitch = double(trim.pitch_deg);
releaseGamma = local_field(trim,'initial_flight_path_deg',0.0);
if cfg > 0
    releasePitch = double(bank(1).pitch_deg);
    releaseGamma = local_field(bank(1),'initial_flight_path_deg',0.0);
end

cfgReach = 0.0;
if cfg > 0
    cfgReach = double(opts.PrepDropStartS) + double(opts.PrepDropIntervalS)*max(0,cfg-1);
end
settleS = max(local_field(trim,'id_settle_s',0.0), ...
    double(opts.SettleBaseS) + double(opts.SettlePerConfigS)*cfg);
stopS = cfgReach + settleS + double(opts.BaselineDurationS) + double(opts.PostBaselineMarginS);

outRoot = char(string(opts.OutputRoot));
if strlength(string(outRoot)) == 0
    outRoot = fullfile(root,'matlab','results','mpc_auto_v32_clean','id_readiness',sprintf('cfg%d',cfg));
end
if ~isfolder(outRoot), mkdir(outRoot); end
runId = sprintf('id_ready_cfg%d_%s',cfg,char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')));

run = airdropx_auto_run_id_experiment( ...
    'ProjectRoot',root,'Model',opts.Model,'OutputRoot',outRoot,'RunId',runId, ...
    'ConfigId',cfg,'Trim',trim,'StopTimeS',stopS,'RecordStartS',0,'ExportStartS',0, ...
    'ExcitationStartS',stopS+10,'Seed',double(opts.Seed)+cfg, ...
    'PrepDropStartS',opts.PrepDropStartS,'PrepDropIntervalS',opts.PrepDropIntervalS, ...
    'InitialAirspeedMps',double(opts.TargetAirspeedMps), ...
    'ReferenceMassKg',double(opts.ReferenceMassKg),'CargoMassKg',double(opts.CargoMassKg), ...
    'InitialAltitudeM',double(opts.TargetAltitudeM),'InitialPitchDeg',releasePitch, ...
    'InitialFlightPathDeg',releaseGamma,'ElevatorAmplitude',0.0,'ThrottleAmplitude',0.0, ...
    'PreparationTrimBank',bank,'UsePreparationTrimSchedule',true, ...
    'KeepFixedConfigurationOnly',true,'DirectIdMode',true);

T = run.timeseries;
metrics = local_metrics(T,trim,opts);
metrics.config_id = cfg;
metrics.settle_s = settleS;
metrics.stop_time_s = stopS;
metrics.pass = local_pass(metrics,opts);
metrics.fail_reason = local_reason(metrics,opts);

summary = struct2table(metrics,'AsArray',true);
writetable(summary,fullfile(outRoot,'id_readiness_summary.csv'));
try
    writetable(T,fullfile(outRoot,'id_readiness_timeseries.csv'));
catch
end
fid = fopen(fullfile(outRoot,'id_readiness_report.txt'),'w');
if fid >= 0
    fprintf(fid,'cfg=%d\npass=%d\nreason=%s\n',cfg,logical(metrics.pass),char(string(metrics.fail_reason)));
    fprintf(fid,'Va=%.6f target=%.6f Verr=%.6f VaSlope=%.6f\n',metrics.va_mps,double(opts.TargetAirspeedMps),metrics.va_error_mps,metrics.va_slope_mps2);
    fprintf(fid,'pitch=%.6f trimPitch=%.6f pitchErr=%.6f\n',metrics.pitch_deg,double(trim.pitch_deg),metrics.pitch_error_deg);
    fprintf(fid,'vz=%.6f q=%.6f hSlope=%.6f hDrift=%.6f\n',metrics.vz_mps,metrics.q_dps,metrics.h_slope_mps,metrics.h_drift_m);
    fprintf(fid,'NOTE: absolute altitude offset is diagnostic only and is not an ID-readiness gate.\n');
    fclose(fid);
end

result = struct('pass',logical(metrics.pass),'metrics',metrics,'run',run, ...
    'summary_csv',string(fullfile(outRoot,'id_readiness_summary.csv')), ...
    'output_root',string(outRoot));
end

function m = local_metrics(T,trim,opts)
m = struct('pass',false,'va_mps',NaN,'va_error_mps',NaN,'va_slope_mps2',NaN, ...
    'pitch_deg',NaN,'pitch_error_deg',NaN,'vz_mps',NaN,'q_dps',NaN, ...
    'h_slope_mps',NaN,'h_drift_m',NaN,'altitude_m',NaN,'window_s',0,'fail_reason',"");
if isempty(T) || height(T) < 10, return; end
t = double(T.time_s(:)); h = double(T.altitude_m(:)); va = double(T.airspeed_mps(:));
p = double(T.pitch_deg(:)); vz = double(T.vz_up_mps(:)); q = double(T.q_dps(:));
valid = isfinite(t)&isfinite(h)&isfinite(va)&isfinite(p)&isfinite(vz)&isfinite(q);
if nnz(valid) < 10, return; end
tEnd = max(t(valid),[],'omitnan');
mask = valid & t >= tEnd-double(opts.BaselineDurationS);
if nnz(mask) < double(opts.MinSamples), mask = valid; end
m.window_s = max(t(mask))-min(t(mask));
m.va_mps = median(va(mask),'omitnan');
m.va_error_mps = m.va_mps-double(opts.TargetAirspeedMps);
m.va_slope_mps2 = local_slope(t(mask),va(mask));
m.pitch_deg = median(p(mask),'omitnan');
m.pitch_error_deg = m.pitch_deg-double(trim.pitch_deg);
m.vz_mps = median(vz(mask),'omitnan');
m.q_dps = median(q(mask),'omitnan');
m.h_slope_mps = local_slope(t(mask),h(mask));
m.altitude_m = median(h(mask),'omitnan');
n = nnz(mask); edge = max(2,round(0.20*n)); hv = h(mask);
m.h_drift_m = mean(hv(end-edge+1:end),'omitnan')-mean(hv(1:edge),'omitnan');
end

function tf = local_pass(m,o)
tf = all(isfinite([m.va_error_mps,m.va_slope_mps2,m.pitch_error_deg,m.vz_mps,m.q_dps,m.h_slope_mps,m.h_drift_m])) && ...
    abs(m.va_error_mps) <= double(o.MaxAirspeedErrorMps) && ...
    abs(m.va_slope_mps2) <= double(o.MaxAirspeedSlopeMps2) && ...
    abs(m.pitch_error_deg) <= double(o.MaxPitchErrorDeg) && ...
    abs(m.vz_mps) <= double(o.MaxAbsVzMps) && ...
    abs(m.q_dps) <= double(o.MaxAbsQDps) && ...
    abs(m.h_slope_mps) <= double(o.MaxHeightSlopeMps) && ...
    abs(m.h_drift_m) <= double(o.MaxHeightDriftM);
end

function s = local_reason(m,o)
parts = strings(0,1);
if ~isfinite(m.va_error_mps) || abs(m.va_error_mps)>double(o.MaxAirspeedErrorMps), parts(end+1)="Va_error"; end %#ok<AGROW>
if ~isfinite(m.va_slope_mps2) || abs(m.va_slope_mps2)>double(o.MaxAirspeedSlopeMps2), parts(end+1)="Va_slope"; end %#ok<AGROW>
if ~isfinite(m.pitch_error_deg) || abs(m.pitch_error_deg)>double(o.MaxPitchErrorDeg), parts(end+1)="pitch_error"; end %#ok<AGROW>
if ~isfinite(m.vz_mps) || abs(m.vz_mps)>double(o.MaxAbsVzMps), parts(end+1)="vz"; end %#ok<AGROW>
if ~isfinite(m.q_dps) || abs(m.q_dps)>double(o.MaxAbsQDps), parts(end+1)="q"; end %#ok<AGROW>
if ~isfinite(m.h_slope_mps) || abs(m.h_slope_mps)>double(o.MaxHeightSlopeMps), parts(end+1)="h_slope"; end %#ok<AGROW>
if ~isfinite(m.h_drift_m) || abs(m.h_drift_m)>double(o.MaxHeightDriftM), parts(end+1)="h_drift"; end %#ok<AGROW>
if isempty(parts), s="PASS"; else, s=strjoin(parts,"|"); end
end

function v = local_field(s,name,defaultValue)
try
    v = double(s.(name));
    if isempty(v)||~isfinite(v),v=double(defaultValue);end
catch
    v=double(defaultValue);
end
end
function v = local_slope(t,y)
t=double(t(:));y=double(y(:));ok=isfinite(t)&isfinite(y);
if nnz(ok)<3,v=NaN;return;end
t0=t(find(ok,1,'first'));p=polyfit(t(ok)-t0,y(ok),1);v=p(1);
end
function o = local_options(varargin)
o.ProjectRoot="";o.Model="airdropx_mpc_id";o.OutputRoot="";o.TrimBank=[];o.ConfigId=0;
o.TargetAltitudeM=200;o.TargetAirspeedMps=50;o.ReferenceMassKg=3423;o.CargoMassKg=300;
o.PrepDropStartS=1.0;o.PrepDropIntervalS=2.0;o.SettleBaseS=55;o.SettlePerConfigS=8;
o.BaselineDurationS=12;o.PostBaselineMarginS=1;o.MinSamples=60;o.Seed=3251;
o.MaxAirspeedErrorMps=0.75;o.MaxAirspeedSlopeMps2=0.08;o.MaxPitchErrorDeg=1.0;
o.MaxAbsVzMps=0.15;o.MaxAbsQDps=0.15;o.MaxHeightSlopeMps=0.15;o.MaxHeightDriftM=1.5;
if mod(numel(varargin),2)~=0,error('Options must be name-value pairs.');end
for k=1:2:numel(varargin)
    n=string(varargin{k});if ~isfield(o,n),error('Unknown option: %s',n);end;o.(n)=varargin{k+1};
end
end
