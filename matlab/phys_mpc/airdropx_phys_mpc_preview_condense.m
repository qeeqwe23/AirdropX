function ctrl=airdropx_phys_mpc_preview_condense(models,cfgSeq,opts)
%AIRDROPX_PHYS_MPC_PREVIEW_CONDENSE Condense a scheduled/preview finite-horizon MPC.
%
% At control stage j the local model is
%   x+ = r_j + A_j (x-r_j) + B_j (u-v_j)
% and the next state error is expressed about the next scheduled trim r_{j+1}:
%   e_{j+1} = A_j e_j + B_j du_j + (r_j-r_{j+1}).
%
% Therefore known payload releases appear as deterministic affine terms inside
% ONE QP. Q/R, state scales, input scales and the optimization kernel remain
% common; only physics-derived A/B/trim/P are scheduled by the known cfg plan.
arguments
    models (5,1) cell
    cfgSeq (1,:) double
    opts.EnableQSoft (1,1) logical = false
    opts.QSoftLimitRadps (1,1) double = NaN
    opts.QSoftPenaltyMultiplier (1,1) double {mustBePositive} = 1e4
end
N=numel(cfgSeq)-1;
if N<1 || any(~isfinite(cfgSeq)) || any(cfgSeq<0) || any(cfgSeq>4) || any(cfgSeq~=round(cfgSeq))
    error("AirdropX:PhysMPC:BadPreviewSequence","cfgSeq must contain N+1 integer cfg values in 0..4.");
end
base=models{1}.ctrl;
n=base.n; m=base.m; Q=base.Q; R=base.R;
if n~=7 || m~=2, error("AirdropX:PhysMPC:BadDimensions","Preview MPC expects 7 states and 2 inputs."); end
if N~=base.N
    error("AirdropX:PhysMPC:PreviewHorizonMismatch","cfgSeq horizon N=%d does not match common controller N=%d.",N,base.N);
end
for c=1:5
    cc=models{c}.ctrl;
    if cc.N~=N || norm(cc.Q-Q,inf)>1e-12 || norm(cc.R-R,inf)>1e-12 || ...
            norm(cc.stateScale-base.stateScale,inf)>1e-12 || norm(cc.inputScale-base.inputScale,inf)>1e-12
        error("AirdropX:PhysMPC:ControllerNotUnified","Preview requires identical Q/R/scales/horizon across cfg0..4.");
    end
end

Astage=zeros(n,n,N); Bstage=zeros(n,m,N);
Rstate=zeros(n,N+1); Rinput=zeros(m,N);
for j=1:N+1
    Rstate(:,j)=models{cfgSeq(j)+1}.ctrl.xref;
    if j<=N
        Astage(:,:,j)=models{cfgSeq(j)+1}.ctrl.A;
        Bstage(:,:,j)=models{cfgSeq(j)+1}.ctrl.B;
        Rinput(:,j)=models{cfgSeq(j)+1}.ctrl.uref;
    end
end
Pterm=models{cfgSeq(end)+1}.ctrl.P;

% Recursively accumulate the cost without forming a giant block Qbar/Gamma.
nu=m*N;
H=zeros(nu,nu); Fdx=zeros(nu,n); fAffine=zeros(nu,1);
M=eye(n); G=zeros(n,nu); d=zeros(n,1);
Mq=zeros(N,n); Gq=zeros(N,nu); cq=zeros(N,1);
for j=1:N
    A=Astage(:,:,j); B=Bstage(:,:,j);
    cols=(j-1)*m+(1:m);
    c=Rstate(:,j)-Rstate(:,j+1);
    M=A*M;
    G=A*G; G(:,cols)=G(:,cols)+B;
    d=A*d+c;
    if j<N, W=Q; else, W=Pterm; end
    H=H+2*(G'*W*G);
    Fdx=Fdx+2*(G'*W*M);
    fAffine=fAffine+2*(G'*W*d);
    H(cols,cols)=H(cols,cols)+2*R;
    Mq(j,:)=M(5,:);
    Gq(j,:)=G(5,:);
    cq(j)=Rstate(5,j+1)+d(5);
end
H=(H+H')/2;

umin=base.umin; umax=base.umax;
lbU=zeros(nu,1); ubU=zeros(nu,1);
for j=1:N
    cols=(j-1)*m+(1:m);
    lbU(cols)=umin-Rinput(:,j);
    ubU(cols)=umax-Rinput(:,j);
end

qLimit=opts.QSoftLimitRadps;
if isnan(qLimit), qLimit=base.stateScale(5); end
if ~(isfinite(qLimit) && qLimit>0)
    error("AirdropX:PhysMPC:BadQSoftLimit","Q soft limit must be positive and finite.");
end
if opts.EnableQSoft
    ns=N; nz=nu+ns;
    qPenalty=opts.QSoftPenaltyMultiplier*Q(5,5);
    Haug=zeros(nz,nz); Haug(1:nu,1:nu)=H; Haug(nu+(1:ns),nu+(1:ns))=2*qPenalty*eye(ns);
    Faug=zeros(nz,n); Faug(1:nu,:)=Fdx;
    fAug=zeros(nz,1); fAug(1:nu)=fAffine;
    Aineq=zeros(2*N,nz); b0=zeros(2*N,1); Bx=zeros(2*N,n);
    for j=1:N
        sj=nu+j;
        r1=2*j-1; r2=2*j;
        Aineq(r1,1:nu)= Gq(j,:); Aineq(r1,sj)=-1;
        Aineq(r2,1:nu)=-Gq(j,:); Aineq(r2,sj)=-1;
        b0(r1)=qLimit-cq(j); Bx(r1,:)=-Mq(j,:);
        b0(r2)=qLimit+cq(j); Bx(r2,:)= Mq(j,:);
    end
    lb=[lbU;zeros(ns,1)]; ub=[ubU;inf(ns,1)];
else
    ns=0; nz=nu; qPenalty=NaN;
    Haug=H; Faug=Fdx; fAug=fAffine;
    Aineq=zeros(0,nz); b0=zeros(0,1); Bx=zeros(0,n);
    lb=lbU; ub=ubU;
end
[~,cholFlag]=chol(Haug);
if cholFlag~=0
    error("AirdropX:PhysMPC:NonConvexPreviewQP","Preview condensed Hessian is not positive definite.");
end
qpopts=optimoptions("quadprog","Algorithm","active-set","Display","off", ...
    "ConstraintTolerance",1e-8,"OptimalityTolerance",1e-8,"StepTolerance",1e-9, ...
    "MaxIterations",max(1000,20*nz));

ctrl=struct();
ctrl.version="Physics-MPC v0.6.0 time-varying preview QP";
ctrl.cfgSeq=cfgSeq; ctrl.N=N; ctrl.n=n; ctrl.m=m; ctrl.nu=nu; ctrl.ns=ns; ctrl.nz=nz;
ctrl.Astage=Astage; ctrl.Bstage=Bstage; ctrl.Rstate=Rstate; ctrl.Rinput=Rinput;
ctrl.Q=Q; ctrl.R=R; ctrl.P=Pterm; ctrl.H=Haug; ctrl.Fdx=Faug; ctrl.fAffine=fAug;
ctrl.Aineq=Aineq; ctrl.bineq0=b0; ctrl.bineqDx=Bx; ctrl.lb=lb; ctrl.ub=ub;
ctrl.umin=umin; ctrl.umax=umax; ctrl.stateScale=base.stateScale; ctrl.inputScale=base.inputScale;
ctrl.enableQSoft=opts.EnableQSoft; ctrl.qSoftLimitRadps=qLimit; ctrl.qSoftPenalty=qPenalty;
ctrl.options=qpopts; ctrl.firstCfg=cfgSeq(1); ctrl.terminalCfg=cfgSeq(end);
ctrl.hasTransition=any(diff(cfgSeq)~=0);
end
