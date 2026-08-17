function trim = airdropx_phys_trim_discrete(Href,Vref,p,z0,opts)
%AIRDROPX_PHYS_TRIM_DISCRETE Physics trim with independent discrete verification.
%
% Unknowns:
%   z3 = [theta_rad; elevator_abs_norm; throttle_norm]
%
% Engine N1/N2 are eliminated by JSBSim propulsion steady-state physics for
% every candidate throttle. JSBSim FGTrim supplies a physics-informed seed.
%
% IMPORTANT: the optimizer solves the instantaneous equations of motion at the
% reconstructed state:
%       udot = 0
%       wdot = 0
%       qdot = 0
% It does NOT optimize only the Ts endpoint. This prevents a transient inside
% the sample from returning q/Va/gamma to their initial endpoint values while
% theta drifts. After the instantaneous equilibrium is found, the exact JSBSim
% Ts map is independently required to satisfy the full seven-state fixed-point
% gate (plus height drift).
arguments
    Href (1,1) double {mustBeFinite}
    Vref (1,1) double {mustBeFinite,mustBePositive}
    p (1,1) struct
    z0 (5,1) double {mustBeFinite}

    % Instantaneous physical-EOM trim tolerances / objective scaling.
    opts.UDotTol_mps2 (1,1) double {mustBePositive} = 0.02
    opts.WDotTol_mps2 (1,1) double {mustBePositive} = 0.02
    opts.QDotTol_radps2 (1,1) double {mustBePositive} = 2e-4

    % Independent exact-discrete fixed-point tolerances.
    opts.VaRateTol_mps2 (1,1) double {mustBePositive} = 0.02
    opts.GammaRateTol_radps (1,1) double {mustBePositive} = 2e-4
    opts.ThetaRateTol_radps (1,1) double {mustBePositive} = 2e-4
    opts.QRateTol_radps2 (1,1) double {mustBePositive} = 2e-4
    opts.EngineRateTol_pctps (1,1) double {mustBePositive} = 0.05
    opts.MaxHeightStep_m (1,1) double {mustBePositive} = 0.01

    opts.ThetaBoundsDeg (1,2) double = [-5 20]
    opts.ElevatorBounds (1,2) double = [-0.95 0.95]
    opts.ThrottleBounds (1,2) double = [0.02 0.98]
    opts.MaxFunctionEvaluations (1,1) double {mustBeInteger,mustBePositive} = 400
    opts.MaxIterations (1,1) double {mustBeInteger,mustBePositive} = 120
    opts.Display (1,1) string = "off"
end

mustHaveFields(p,{'cfgId','Ts'});
if ~isfield(p,"fuelScale") || isempty(p.fuelScale), p.fuelScale=1.0; end
validateattributes(p.cfgId,{'double'},{'scalar','real','finite','integer','>=',0,'<=',4},mfilename,'p.cfgId');
validateattributes(p.Ts,{'double'},{'scalar','real','finite','positive'},mfilename,'p.Ts');
validateattributes(p.fuelScale,{'double'},{'scalar','real','finite','>=',0,'<=',1.2},mfilename,'p.fuelScale');
assert(opts.ThetaBoundsDeg(1)<opts.ThetaBoundsDeg(2),"Bad theta bounds.");
assert(opts.ElevatorBounds(1)<opts.ElevatorBounds(2),"Bad elevator bounds.");
assert(opts.ThrottleBounds(1)<opts.ThrottleBounds(2),"Bad throttle bounds.");

lb=[deg2rad(opts.ThetaBoundsDeg(1));opts.ElevatorBounds(1);opts.ThrottleBounds(1)];
ub=[deg2rad(opts.ThetaBoundsDeg(2));opts.ElevatorBounds(2);opts.ThrottleBounds(2)];
seed3=z0(1:3);
seedSource="existing";
builtin=struct("success",false,"error","");

% Stage 1: native JSBSim longitudinal trim as a physics-informed seed only.
try
    b=airdropx_jsbsim_oracle_mex("builtin_trim",Href,Vref,p.cfgId,p.fuelScale,z0);
    builtin=b;
    if isfield(b,"success") && logical(b.success) && all(isfinite(b.z(1:3)))
        seed3=b.z(1:3);
        seedSource="JSBSim_FGTrim";
    end
catch ME
    builtin=struct("success",false,"error",string(ME.identifier)+": "+string(ME.message));
end
seed3=min(max(seed3,lb+1e-8),ub-1e-8);

% Stage 2: bounded physical-equilibrium polish.
accelScale=[opts.UDotTol_mps2;opts.WDotTol_mps2;opts.QDotTol_radps2];
objective=@(z3)localPhysicalObjective(z3,Href,Vref,p,accelScale);
solverOpts=optimoptions("lsqnonlin", ...
    "Display",opts.Display, ...
    "FunctionTolerance",1e-12, ...
    "StepTolerance",1e-11, ...
    "OptimalityTolerance",1e-10, ...
    "FiniteDifferenceType","central", ...
    "MaxFunctionEvaluations",opts.MaxFunctionEvaluations, ...
    "MaxIterations",opts.MaxIterations);

[z3,resnorm,~,exitflag,output]=lsqnonlin(objective,seed3,lb,ub,solverOpts);
raw=localEval(z3,Href,Vref,p);
x0=raw.x0(:);
x1=raw.xnext(:);

% Instantaneous EOM residual: this is the actual trim definition.
physicalAccel=[raw.diag0.udot_mps2;raw.diag0.wdot_mps2;raw.diag0.qdot_radps2];
physicalAccelTol=accelScale;
scaledPhysicalAccel=physicalAccel./physicalAccelTol;

% Independent exact-discrete verification over Ts.
targetResidual=[x1(2)-x0(2); ...
                x1(3)-x0(3); ...
                x1(4)-x0(4); ...
                x1(5)-x0(5); ...
                x1(6)-x0(6); ...
                x1(7)-x0(7)];
initialTargetError=[x0(2)-Vref; x0(3); x0(5)];
rateResidual=targetResidual./p.Ts;
rateTol=[opts.VaRateTol_mps2; ...
         opts.GammaRateTol_radps; ...
         opts.ThetaRateTol_radps; ...
         opts.QRateTol_radps2; ...
         opts.EngineRateTol_pctps; ...
         opts.EngineRateTol_pctps];
scaled=rateResidual./rateTol;
hStep=x1(1)-x0(1);

% Keep legacy z=[theta,elevator,throttle,N1,N2] for continuation/build-bank callers.
z=[z3;x0(6);x0(7)];
nearBound=min(z3-lb,ub-z3);
boundScale=max(ub-lb,eps);
boundFraction=nearBound./boundScale;

trim=struct();
trim.method="JSBSim_FGTrim_seed__steady_engine__instantaneous_EOM_polish__discrete_verify";
trim.seed_source=seedSource;
trim.builtin_trim=builtin;
trim.seed3=seed3;
trim.z=z;
trim.z3=z3;
trim.x=x0;
trim.u=z3(2:3);
trim.xnext=x1;
trim.diag=raw.diag0;
trim.diag_next=raw.diag1;

trim.physical_accel_residual=physicalAccel;
trim.physical_accel_tolerance=physicalAccelTol;
trim.scaled_physical_accel=scaledPhysicalAccel;

trim.sample_residual=targetResidual;
trim.residual=targetResidual; % backward-compatible field name
trim.initial_target_error=initialTargetError;
trim.rate_residual=rateResidual;
trim.rate_tolerance=rateTol;
trim.scaled_residual=scaled;
trim.h_step_m=hStep;
trim.exitflag=exitflag;
trim.output=output;
trim.resnorm=resnorm;
trim.bounds=struct("lb",lb,"ub",ub,"fraction_to_nearest_bound",boundFraction);

% Optimizer status is diagnostic. Acceptance is physics + independent map checks.
trim.pass_solver=exitflag>0;
trim.pass_initial=all(abs(initialTargetError)<=[1e-5;1e-8;1e-8]);
trim.pass_acceleration=all(isfinite(scaledPhysicalAccel)) && norm(scaledPhysicalAccel,inf)<=1;
trim.pass_fixed_point=all(isfinite(scaled)) && norm(scaled,inf)<=1;
trim.pass_height=isfinite(hStep) && abs(hStep)<=opts.MaxHeightStep_m;
trim.pass_physics=trim.pass_initial && trim.pass_acceleration && trim.pass_fixed_point && trim.pass_height;
trim.pass=trim.pass_physics;
end

function s=localPhysicalObjective(z3,H,V,p,scale)
r=localEval(z3,H,V,p);
a=[r.diag0.udot_mps2; ...
   r.diag0.wdot_mps2; ...
   r.diag0.qdot_radps2];
s=a./scale;
if any(~isfinite(s)), s(:)=1e12; end
end

function r=localEval(z3,H,V,p)
r=airdropx_jsbsim_oracle_mex("steady_eval",H,V,z3(1),z3(2),z3(3),p.cfgId,p.fuelScale,p.Ts);
end

function mustHaveFields(s,names)
for k=1:numel(names)
    if ~isfield(s,names{k}) || isempty(s.(names{k}))
        error("AirdropX:PhysMPC:MissingField","Required field p.%s is missing.",names{k});
    end
end
end
