function [trimOut,info] = airdropx_physics_trim_solve(varargin)
%AIRDROPX_PHYSICS_TRIM_SOLVE Deterministic 5-residual/2-control trim solve.
%
% Physics-MPC v1.6:
%   - pitch is NOT an actuator and is never a Newton decision variable;
%   - every evaluation in one solve uses one explicit fixed IC pitch;
%   - elevator_cmd remains the legacy MEX EXTERNAL delta coordinate;
%   - actual physical elevator is logged in every evaluation;
%   - caller may request the best finite candidate on non-catastrophic failure
%     so a state-consistency restart can remove a pure pitch transient.
%
% Residual:
%   [Va-Vref, vz, q, h_dot, Va_dot]
% No BayesOpt, random/global learning, model-order search or online tuning.

opts=local_options(varargin{:});
root=local_root(opts.ProjectRoot);
addpath(fullfile(root,'matlab'));addpath(fullfile(root,'matlab','mpc'));addpath(fullfile(root,'matlab','mpc_auto'));addpath(fullfile(root,'matlab','sfunc_jsbsim'));
outRoot=char(string(opts.OutputRoot));
if isempty(outRoot),outRoot=fullfile(root,'matlab','results','mpc_physics_v1','deterministic_retrim');end
if ~isfolder(outRoot),mkdir(outRoot);end
bank=opts.TrimBank;trim=opts.InitialTrim;cfg=round(double(opts.ConfigId));v=double(opts.SpeedMps);
if isempty(bank)||numel(bank)<5||isempty(trim),error('AirdropX:PhysicsMPC:RetrimInputs','TrimBank and InitialTrim are required.');end
cfg=max(0,min(4,cfg));
if isfinite(double(opts.FixedInitialPitchDeg))
    fixedPitchSeed=double(opts.FixedInitialPitchDeg);
else
    fixedPitchSeed=local_field(bank(cfg+1),'pitch_deg',local_field(bank(1),'pitch_deg',4.0));
end

traceRows=cell(0,16);best=trim;bestScore=Inf;bestStats=struct();
info=struct('success',false,'best_score',Inf,'best_stats',struct(),'best_trim',trim, ...
    'fixed_initial_pitch_deg',fixedPitchSeed,'message',"");
for it=0:round(double(opts.MaxIterations))
    [s0,score0]=local_eval(root,fullfile(outRoot,sprintf('iter_%02d_base',it)),bank,trim,cfg,v,fixedPitchSeed,opts,100+20*it);
    traceRows=[traceRows;local_row(it,'base',fixedPitchSeed,trim,s0,score0)]; %#ok<AGROW>
    if score0<bestScore,bestScore=score0;best=trim;bestStats=s0;end
    fprintf(['[PHYS-TRIM5x2] V=%.1f cfg%d iter%d seedPitch=%.4f extE=%.6f physE=%.6f th=%.6f ', ...
        'VaErr=%+.4f vz=%+.4f q=%+.4f hSlope=%+.4f VaSlope=%+.4f pitchStd=%.4f score=%.3f pass=%d\n'], ...
        v,cfg,fixedPitchSeed,double(trim.elevator_cmd),s0.elevator_physical,double(trim.throttle_cmd), ...
        s0.va_error_mps,s0.vz_mps,s0.q_dps,s0.h_slope_mps,s0.va_slope_mps2,s0.pitch_std_deg,score0,s0.pass);
    if s0.pass
        local_write_trace(traceRows,outRoot);
        trimOut=local_observed_trim(trim,s0);
        info=local_info(true,score0,s0,trimOut,fixedPitchSeed,"PASS");
        return;
    end
    if logical(opts.ReturnBestOnFailure) && local_dynamic_pass(s0,opts)
        % Controls already satisfy the physical force/moment/velocity residuals.
        % Do not distort them to reduce pitchStd caused by an inconsistent IC.
        local_write_trace(traceRows,outRoot);
        trimOut=local_observed_trim(trim,s0);
        info=local_info(false,score0,s0,trimOut,fixedPitchSeed,"DYNAMIC_PASS_PITCH_STATE_MISMATCH");
        return;
    end
    if it>=round(double(opts.MaxIterations)),break;end

    te=trim;te.elevator_cmd=local_clip(double(trim.elevator_cmd)+double(opts.ElevatorProbe),double(opts.ElevatorBounds));
    tt=trim;tt.throttle_cmd=local_clip(double(trim.throttle_cmd)+double(opts.ThrottleProbe),double(opts.ThrottleBounds));
    de=double(te.elevator_cmd)-double(trim.elevator_cmd);dt=double(tt.throttle_cmd)-double(trim.throttle_cmd);
    if abs(de)<1e-8||abs(dt)<1e-8
        local_write_trace(traceRows,outRoot);
        error('AirdropX:PhysicsMPC:RetrimAtBounds','Finite-difference probe is blocked by actuator bounds.');
    end
    [se,sce]=local_eval(root,fullfile(outRoot,sprintf('iter_%02d_probe_e',it)),bank,te,cfg,v,fixedPitchSeed,opts,101+20*it);
    [st,sct]=local_eval(root,fullfile(outRoot,sprintf('iter_%02d_probe_t',it)),bank,tt,cfg,v,fixedPitchSeed,opts,102+20*it);
    traceRows=[traceRows;local_row(it,'probe_e',fixedPitchSeed,te,se,sce);local_row(it,'probe_t',fixedPitchSeed,tt,st,sct)]; %#ok<AGROW>

    r0=local_residual(s0,opts);
    J=[(local_residual(se,opts)-r0)/de,(local_residual(st,opts)-r0)/dt];
    if any(~isfinite(J(:)))||rank(J)<2
        local_write_trace(traceRows,outRoot);
        error('AirdropX:PhysicsMPC:RetrimSingularJacobian','5x2 trim Jacobian is rank %d at V=%.1f cfg%d.',rank(J),v,cfg);
    end

    varScale=[double(opts.MaxElevatorStep);double(opts.MaxThrottleStep)];
    Js=J*diag(varScale);
    dw=-((Js.'*Js+double(opts.NewtonDamping)*eye(2))\(Js.'*r0));
    step=varScale.*dw;
    step(1)=min(max(step(1),-double(opts.MaxElevatorStep)),double(opts.MaxElevatorStep));
    step(2)=min(max(step(2),-double(opts.MaxThrottleStep)),double(opts.MaxThrottleStep));

    factors=[1.0 0.5 0.25];chosen=[];chosenStats=[];chosenScore=Inf;
    for lf=1:numel(factors)
        a=factors(lf);cand=trim;
        cand.elevator_cmd=local_clip(double(trim.elevator_cmd)+a*step(1),double(opts.ElevatorBounds));
        cand.throttle_cmd=local_clip(double(trim.throttle_cmd)+a*step(2),double(opts.ThrottleBounds));
        [sc,scc]=local_eval(root,fullfile(outRoot,sprintf('iter_%02d_step_%g',it,a)),bank,cand,cfg,v,fixedPitchSeed,opts,110+20*it+lf);
        traceRows=[traceRows;local_row(it,sprintf('step_%g',a),fixedPitchSeed,cand,sc,scc)]; %#ok<AGROW>
        if scc<chosenScore,chosen=cand;chosenStats=sc;chosenScore=scc;end
        if sc.pass
            local_write_trace(traceRows,outRoot);
            trimOut=local_observed_trim(cand,sc);
            info=local_info(true,scc,sc,trimOut,fixedPitchSeed,"PASS");
            return;
        end
        if logical(opts.ReturnBestOnFailure) && local_dynamic_pass(sc,opts)
            local_write_trace(traceRows,outRoot);
            trimOut=local_observed_trim(cand,sc);
            info=local_info(false,scc,sc,trimOut,fixedPitchSeed,"DYNAMIC_PASS_PITCH_STATE_MISMATCH");
            return;
        end
    end
    if chosenScore<bestScore,bestScore=chosenScore;best=chosen;bestStats=chosenStats;end
    if chosenScore>=score0*(1-double(opts.MinRelativeImprovement))
        break;
    end
    trim=chosen;
end

local_write_trace(traceRows,outRoot);
info=local_info(false,bestScore,bestStats,best,fixedPitchSeed,"BEST_CANDIDATE_ONLY");
if logical(opts.ReturnBestOnFailure) && isstruct(bestStats) && isfield(bestStats,'va_error_mps') && isfinite(bestScore)
    trimOut=local_observed_trim(best,bestStats);
    info.best_trim=trimOut;
    return;
end
error('AirdropX:PhysicsMPC:DeterministicRetrimFailed', ...
    ['V=%.1f cfg%d deterministic 5-residual/2-control trim failed. Best: extE=%.6f physE=%.6f th=%.6f ', ...
     'VaErr=%+.4f vz=%+.4f q=%+.4f hSlope=%+.4f VaSlope=%+.4f pitchStd=%.4f score=%.3f. See %s'], ...
    v,cfg,double(best.elevator_cmd),bestStats.elevator_physical,double(best.throttle_cmd), ...
    bestStats.va_error_mps,bestStats.vz_mps,bestStats.q_dps,bestStats.h_slope_mps,bestStats.va_slope_mps2,bestStats.pitch_std_deg,bestScore,fullfile(outRoot,'deterministic_trim_trace.csv'));
end

function info=local_info(success,score,stats,trim,pitch,msg)
info=struct('success',logical(success),'best_score',double(score),'best_stats',stats,'best_trim',trim, ...
    'fixed_initial_pitch_deg',double(pitch),'message',string(msg));
end

function trim=local_observed_trim(trim,s)
% Observed pitch is an operating-point state only. The caller explicitly
% chooses the next IC pitch and compensates hidden elevator if it changes.
trim.pitch_deg=s.pitch_deg;
trim.airspeed_mps=s.va_mps;
trim.vz_up_mps=s.vz_mps;
trim.q_dps=s.q_dps;
trim.throttle_cmd=s.throttle_physical;
trim.physical_elevator_cmd=s.elevator_physical;
end

function r=local_residual(s,o)
r=[s.va_error_mps/max(0.05,double(o.MaxVaErrorMps)); ...
   s.vz_mps/max(0.03,double(o.MaxAbsVzMps)); ...
   s.q_dps/max(0.03,double(o.MaxAbsQDps)); ...
   s.h_slope_mps/max(0.03,double(o.MaxHeightSlopeMps)); ...
   s.va_slope_mps2/max(0.01,double(o.MaxVaSlopeMps2))];
end

function [s,score]=local_eval(root,outRoot,bank,trim,cfg,v,fixedPitchSeed,o,seed)
if ~isfolder(outRoot),mkdir(outRoot);end
r=airdropx_auto_run_id_experiment('ProjectRoot',root,'OutputRoot',outRoot,'RunId',sprintf('phys_trim52_v%.1f_cfg%d_s%d',v,cfg,seed), ...
    'ConfigId',cfg,'InitialDropCount',cfg,'PrepareByDrops',false,'DirectCfgViaAircraftXml',true,'Trim',trim,'StopTimeS',double(o.StopTimeS),'RecordStartS',0,'ExportStartS',0,'ExcitationStartS',Inf, ...
    'PrepDropStartS',double(o.StopTimeS)+100,'PrepDropIntervalS',1.0,'KeepFixedConfigurationOnly',true,'DirectIdMode',true, ...
    'PreparationTrimBank',bank,'UsePreparationTrimSchedule',false,'OperatingPointWindowS',3.0,'Seed',seed, ...
    'InitialAirspeedMps',v,'InitialAltitudeM',double(o.ReferenceAltitudeM),'InitialPitchDeg',double(fixedPitchSeed),'InitialFlightPathDeg',0, ...
    'TargetAltitudeM',double(o.ReferenceAltitudeM),'TargetAirspeedMps',v,'ReferenceMassKg',double(o.ReferenceMassKg),'CargoMassKg',double(o.CargoMassKg), ...
    'IsolateGeneratedIc',true,'ElevatorAmplitude',0,'ThrottleAmplitude',0);
T=r.timeseries;t=double(T.time_s(:));m=isfinite(t)&t>=max(min(t),max(t)-double(o.TailWindowS));
va=double(T.airspeed_mps(:));p=double(T.pitch_deg(:));vz=double(T.vz_up_mps(:));q=double(T.q_dps(:));h=double(T.altitude_m(:));
m=m&isfinite(va)&isfinite(p)&isfinite(vz)&isfinite(q)&isfinite(h);if nnz(m)<20,error('AirdropX:PhysicsMPC:RetrimShortRun','Too few tail samples.');end
s=struct();s.va_mps=median(va(m),'omitnan');s.pitch_deg=median(p(m),'omitnan');s.va_error_mps=s.va_mps-v;s.vz_mps=median(vz(m),'omitnan');s.q_dps=median(q(m),'omitnan');
s.h_slope_mps=local_slope(t(m),h(m));s.va_slope_mps2=local_slope(t(m),va(m));s.pitch_std_deg=std(p(m),0,'omitnan');
s.elevator_physical=local_median_field(T,m,{'elevator_physical_actual','elevator_cmd_norm','elevator_cmd_actual','elevator_delta'},NaN);
s.throttle_physical=local_median_field(T,m,{'throttle_physical_actual','throttle_norm','throttle_cmd_actual','throttle_cmd'},NaN);
s.pass=abs(s.va_error_mps)<=double(o.MaxVaErrorMps)&&abs(s.vz_mps)<=double(o.MaxAbsVzMps)&&abs(s.q_dps)<=double(o.MaxAbsQDps)&& ...
    abs(s.h_slope_mps)<=double(o.MaxHeightSlopeMps)&&abs(s.va_slope_mps2)<=double(o.MaxVaSlopeMps2)&&s.pitch_std_deg<=double(o.MaxPitchStdDeg);
rn=local_residual(s,o);score=sum(rn.^2);
end

function tf=local_dynamic_pass(s,o)
tf=abs(s.va_error_mps)<=double(o.MaxVaErrorMps)&&abs(s.vz_mps)<=double(o.MaxAbsVzMps)&& ...
   abs(s.q_dps)<=double(o.MaxAbsQDps)&&abs(s.h_slope_mps)<=double(o.MaxHeightSlopeMps)&& ...
   abs(s.va_slope_mps2)<=double(o.MaxVaSlopeMps2);
end
function r=local_row(it,kind,pitchSeed,trim,s,score)
r={it,string(kind),double(pitchSeed),double(trim.elevator_cmd),s.elevator_physical,double(trim.throttle_cmd),s.throttle_physical, ...
   s.pitch_deg,s.va_error_mps,s.vz_mps,s.q_dps,s.h_slope_mps,s.va_slope_mps2,s.pitch_std_deg,score,logical(s.pass)};
end
function local_write_trace(rows,outRoot)
if isempty(rows),return;end
T=cell2table(rows,'VariableNames',{'iteration','evaluation','fixed_initial_pitch_deg','elevator_external_delta','elevator_physical', ...
    'throttle_cmd','throttle_physical','observed_pitch_deg','va_error_mps','vz_mps','q_dps','h_slope_mps','va_slope_mps2','pitch_std_deg','score','pass'});
writetable(T,fullfile(outRoot,'deterministic_trim_trace.csv'));
end
function x=local_clip(x,b),x=min(max(x,min(b)),max(b));end
function s=local_slope(t,y),pp=polyfit(double(t(:))-double(t(1)),double(y(:)),1);s=pp(1);end
function x=local_median_field(T,m,names,fallback)
x=fallback;
for k=1:numel(names)
    if ismember(names{k},T.Properties.VariableNames)
        z=double(T.(names{k})(:));z=z(m&isfinite(z));
        if ~isempty(z),x=median(z,'omitnan');return;end
    end
end
end
function v=local_field(s,n,d),v=d;try,if isstruct(s)&&isfield(s,n)&&~isempty(s.(n))&&isfinite(double(s.(n))),v=double(s.(n));end,catch,end,end
function root=local_root(x),if strlength(string(x))>0,root=char(string(x));else,a=fileparts(mfilename('fullpath'));root=fileparts(fileparts(a));end,end
function opts=local_options(varargin)
opts.ProjectRoot="";opts.OutputRoot="";opts.TrimBank=[];opts.InitialTrim=[];opts.ConfigId=0;opts.SpeedMps=50;opts.ReferenceAltitudeM=200;opts.ReferenceMassKg=3423;opts.CargoMassKg=300;
opts.StopTimeS=28;opts.TailWindowS=10;opts.MaxIterations=8;opts.ElevatorProbe=0.008;opts.ThrottleProbe=0.015;opts.MaxElevatorStep=0.05;opts.MaxThrottleStep=0.08;opts.ElevatorBounds=[-0.75 0.45];opts.ThrottleBounds=[0.25 0.95];opts.NewtonDamping=1e-3;opts.MinRelativeImprovement=1e-3;
opts.MaxVaErrorMps=0.50;opts.MaxAbsVzMps=0.15;opts.MaxAbsQDps=0.15;opts.MaxHeightSlopeMps=0.15;opts.MaxVaSlopeMps2=0.05;opts.MaxPitchStdDeg=0.50;
opts.FixedInitialPitchDeg=NaN;opts.ReturnBestOnFailure=false;
if mod(numel(varargin),2)~=0,error('Options must be name-value pairs.');end
for i=1:2:numel(varargin),n=string(varargin{i});if ~isfield(opts,n),error('Unknown option: %s',n);end,opts.(n)=varargin{i+1};end
end
