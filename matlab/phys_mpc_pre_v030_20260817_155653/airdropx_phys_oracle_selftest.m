function report=airdropx_phys_oracle_selftest(x,u,p,opts)
%AIRDROPX_PHYS_ORACLE_SELFTEST Determinism, sanity, and Markov-closure gate.
% The semigroup check is important for the engine-augmented state. If the
% direct 2*Ts trajectory differs materially from two reconstructed Ts steps,
% [h Va gamma theta q N1 N2] is not a sufficiently closed state for MPC.
arguments
    x (7,1) double {mustBeFinite}
    u (2,1) double {mustBeFinite}
    p (1,1) struct
    opts.Repeats (1,1) double {mustBeInteger,mustBePositive} = 3
    opts.MaxRepeatAbsState (1,1) double {mustBeNonnegative} = 1e-11
    opts.MaxPathAbsState (1,1) double {mustBeNonnegative} = 1e-10
    opts.CheckSemigroup (1,1) logical = true
    opts.SemigroupAbsTolerance (7,1) double {mustBePositive} = ...
        [1e-3;1e-4;1e-6;1e-6;1e-6;1e-3;1e-3]
    opts.MaxSemigroupScaled (1,1) double {mustBePositive} = 1.0
end
assert(opts.Repeats>=3,"Repeats must be >= 3.");
p=localValidateP(p);

X=zeros(7,opts.Repeats); D=cell(1,opts.Repeats);
for k=1:opts.Repeats
    [X(:,k),D{k}]=airdropx_phys_step(x,u,p);
end
spread=max(X,[],2)-min(X,[],2);
maxSpread=max(abs(spread));
d=D{1};
fuelFrozen=isfield(d,"fuel_frozen") && logical(d.fuel_frozen);
physicalOk=all(isfinite([d.mass_kg d.cg_x_m d.Iyy_kgm2 d.qbar_Pa])) && ...
    d.mass_kg>100 && d.Iyy_kgm2>10 && d.qbar_Pa>0 && fuelFrozen;

deterministic=maxSpread<=opts.MaxRepeatAbsState;

% Path-independence gate: deliberately visit a different operating/configuration
% point, then reconstruct the original point. A persistent Oracle is useful only
% if a previous call cannot leak hidden FCS/mass/engine state into the next call.
pAlt=p;
pAlt.cfgId=mod(round(p.cfgId)+1,5);
if abs(p.fuelScale-0.8)>1e-9, pAlt.fuelScale=0.8; else, pAlt.fuelScale=1.0; end
xAlt=x;
xAlt(2)=max(1,xAlt(2)+0.37);
xAlt(3)=xAlt(3)+3e-3;
xAlt(4)=xAlt(4)-2e-3;
xAlt(5)=xAlt(5)+1e-3;
xAlt(6)=max(0,xAlt(6)+0.2);
xAlt(7)=max(0,xAlt(7)-0.2);
uAlt=[max(-0.95,min(0.95,u(1)+0.01)); max(0.02,min(0.98,u(2)-0.015))];
airdropx_phys_step(xAlt,uAlt,pAlt);
xAfterPath=airdropx_phys_step(x,u,p);
pathError=xAfterPath-X(:,1);
pathIndependent=all(isfinite(pathError)) && norm(pathError,inf)<=opts.MaxPathAbsState;

semigroup=struct("checked",false,"pass",true,"error",zeros(7,1), ...
    "scaled_error",zeros(7,1),"max_scaled",0,"x2_direct",nan(7,1),"x2_chained",nan(7,1));
if opts.CheckSemigroup
    % Direct 2Ts preserves every hidden JSBSim internal state. Chained Ts+Ts
    % deliberately reconstructs JSBSim from only our declared seven states.
    % Their agreement is therefore a practical closure test of the MPC state.
    p2=p; p2.Ts=2*p.Ts;
    x1=X(:,1);
    x2direct=airdropx_phys_step(x,u,p2);
    x2chained=airdropx_phys_step(x1,u,p);
    e=x2chained-x2direct;
    es=e./opts.SemigroupAbsTolerance;
    semigroup=struct("checked",true,"pass",all(isfinite(es)) && norm(es,inf)<=opts.MaxSemigroupScaled, ...
        "error",e,"scaled_error",es,"max_scaled",norm(es,inf), ...
        "x2_direct",x2direct,"x2_chained",x2chained, ...
        "tolerance",opts.SemigroupAbsTolerance);
end

report=struct("states",X,"spread",spread,"max_abs_state_spread",maxSpread, ...
    "physical_ok",physicalOk,"fuel_frozen",fuelFrozen,"deterministic",deterministic, ...
    "path_independent",pathIndependent,"path_error",pathError,"max_abs_path_error",norm(pathError,inf), ...
    "semigroup",semigroup, ...
    "pass",physicalOk && deterministic && pathIndependent && semigroup.pass,"diag",d);
if ~report.pass
    error("AirdropX:PhysMPC:OracleSelfTestFailed", ...
        "Oracle self-test failed: deterministic=%d maxSpread=%.3g pathIndependent=%d pathError=%.3g physicalOk=%d fuelFrozen=%d semigroupPass=%d semigroupMaxScaled=%.3g", ...
        deterministic,maxSpread,pathIndependent,norm(pathError,inf),physicalOk,fuelFrozen,semigroup.pass,semigroup.max_scaled);
end
end

function p=localValidateP(p)
for c={'cfgId','Ts'}
    n=c{1};
    if ~isfield(p,n) || isempty(p.(n))
        error("AirdropX:PhysMPC:MissingField","Required field p.%s is missing.",n);
    end
end
if ~isfield(p,"fuelScale") || isempty(p.fuelScale), p.fuelScale=1.0; end
validateattributes(p.cfgId,{'double'},{'scalar','real','finite','integer','>=',0,'<=',4},mfilename,'p.cfgId');
validateattributes(p.fuelScale,{'double'},{'scalar','real','finite','>=',0,'<=',1.2},mfilename,'p.fuelScale');
validateattributes(p.Ts,{'double'},{'scalar','real','finite','positive'},mfilename,'p.Ts');
end
