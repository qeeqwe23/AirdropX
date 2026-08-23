function ctrl=airdropx_phys_mpc_runtime_condense_v101(bank,Hseq,Vseq,HdotSeq,VdotSeq,cfgSeq)
%AIRDROPX_PHYS_MPC_RUNTIME_CONDENSE_V101 Condense dynamic-feasible H/V/cfg reference into one QP.
arguments
    bank (1,1) struct
    Hseq (1,:) double
    Vseq (1,:) double
    HdotSeq (1,:) double
    VdotSeq (1,:) double
    cfgSeq (1,:) double
end
N=numel(Hseq)-1;
if N~=100 || numel(Vseq)~=N+1 || numel(HdotSeq)~=N+1 || numel(VdotSeq)~=N+1 || numel(cfgSeq)~=N+1
    error("AirdropX:PhysMPC:RuntimeSequenceSize","Runtime horizon must contain 101 H/V/Hdot/Vdot/cfg samples.");
end
if any(~isfinite(Hseq))||any(~isfinite(Vseq))||any(~isfinite(HdotSeq))||any(~isfinite(VdotSeq))||any(~isfinite(cfgSeq))||any(cfgSeq<0)||any(cfgSeq>4)||any(cfgSeq~=round(cfgSeq))
    error("AirdropX:PhysMPC:RuntimeBadSequence","Bad runtime sequence.");
end
n=7; m=2; Q=bank.Q; R=bank.R; Astage=zeros(n,n,N); Bstage=zeros(n,m,N); Xbar=zeros(n,N+1); Ubar=zeros(m,N); certCount=zeros(N+1,1);
for j=1:N
    s=airdropx_phys_runtime_interpolate_stage_v101(bank,Hseq(j),Vseq(j),cfgSeq(j),NeedTerminal=false);
    Astage(:,:,j)=s.A; Bstage(:,:,j)=s.B; Xbar(:,j)=s.xbar; Ubar(:,j)=s.ubar; certCount(j)=s.source_certified_count;
end
st=airdropx_phys_runtime_interpolate_stage_v101(bank,Hseq(end),Vseq(end),cfgSeq(end),NeedTerminal=true); Xbar(:,end)=st.xbar; certCount(end)=st.source_certified_count; Pterm=st.P;

% Dynamic-feasible state reference.  The level-trim alpha is preserved while
% gamma follows hdot = Va*sin(gamma).  q_ref is the time derivative of theta_ref
% computed separately on each constant-cfg segment, so a discrete cfg trim jump
% cannot create an artificial pitch-rate impulse.
ratio=HdotSeq./Vseq;
if any(abs(ratio)>=0.98), error("AirdropX:PhysMPC:RuntimeReferenceKinematics","|Hdot/Va| too large for a finite flight-path reference."); end
gammaRef=asin(ratio);
alphaTrim=localWrap(Xbar(4,:)-Xbar(3,:));
thetaRef=localWrap(alphaTrim+gammaRef);
qRef=localSegmentDerivative(thetaRef,cfgSeq,bank.Ts);
Rstate=Xbar; Rstate(1,:)=Hseq; Rstate(2,:)=Vseq; Rstate(3,:)=gammaRef; Rstate(4,:)=thetaRef; Rstate(5,:)=qRef;
Rinput=Ubar;

% The local plant is linearized about level trim xbar/ubar, not about Rstate.
% Therefore the moving-reference error system has an explicit affine defect:
% e+ = A e + B du + [xbar + A(r-xbar) + B(uref-ubar) - r+].
cstage=zeros(n,N);
for j=1:N
    cstage(:,j)=Xbar(:,j)+Astage(:,:,j)*(Rstate(:,j)-Xbar(:,j))+Bstage(:,:,j)*(Rinput(:,j)-Ubar(:,j))-Rstate(:,j+1);
end

nu=m*N; H=zeros(nu); Fdx=zeros(nu,n); fAffine=zeros(nu,1); M=eye(n); G=zeros(n,nu); d=zeros(n,1);
for j=1:N
    A=Astage(:,:,j); B=Bstage(:,:,j); cols=(j-1)*m+(1:m); M=A*M; G=A*G; G(:,cols)=G(:,cols)+B; d=A*d+cstage(:,j);
    if j<N, W=Q; else, W=Pterm; end
    H=H+2*(G'*W*G); Fdx=Fdx+2*(G'*W*M); fAffine=fAffine+2*(G'*W*d); H(cols,cols)=H(cols,cols)+2*R;
end
H=(H+H')/2; lb=zeros(nu,1); ub=zeros(nu,1);
for j=1:N, cols=(j-1)*m+(1:m); lb(cols)=bank.umin-Rinput(:,j); ub(cols)=bank.umax-Rinput(:,j); end
[~,flag]=chol(H); if flag~=0, error("AirdropX:PhysMPC:RuntimeNonConvexQP","Runtime Hessian is not positive definite."); end
qpopts=optimoptions("quadprog","Algorithm","active-set","Display","off","ConstraintTolerance",1e-8,"OptimalityTolerance",1e-8,"StepTolerance",1e-9,"MaxIterations",4000);
kinResidual=Vseq.*sin(gammaRef)-HdotSeq;
ctrl=struct("version","Physics-MPC v1.0.1 dynamic-feasible runtime QP","reference_mode","dynamic_feasible", ...
    "cfgSeq",cfgSeq,"Hseq",Hseq,"Vseq",Vseq,"HdotSeq",HdotSeq,"VdotSeq",VdotSeq,"N",N,"n",n,"m",m,"nu",nu,"ns",0,"nz",nu, ...
    "Astage",Astage,"Bstage",Bstage,"Xbar",Xbar,"Ubar",Ubar,"Rstate",Rstate,"Rinput",Rinput,"cstage",cstage, ...
    "gammaRef",gammaRef,"thetaRef",thetaRef,"qRef",qRef,"kinematic_residual_max",max(abs(kinResidual)), ...
    "Q",Q,"R",R,"P",Pterm,"H",H,"Fdx",Fdx,"fAffine",fAffine,"Aineq",zeros(0,nu),"bineq0",zeros(0,1),"bineqDx",zeros(0,n), ...
    "lb",lb,"ub",ub,"umin",bank.umin,"umax",bank.umax,"stateScale",bank.stateScale,"inputScale",bank.inputScale,"enableQSoft",false,"options",qpopts, ...
    "firstCfg",cfgSeq(1),"terminalCfg",cfgSeq(end),"source_certified_count_min",min(certCount));
end

function y=localWrap(x)
y=atan2(sin(x),cos(x));
end

function d=localSegmentDerivative(x,cfg,Ts)
x=unwrap(double(x(:).')); cfg=double(cfg(:).'); d=zeros(size(x)); n=numel(x); a=1;
while a<=n
    b=a; while b<n && cfg(b+1)==cfg(a), b=b+1; end
    len=b-a+1;
    if len==2
        vv=(x(b)-x(a))/Ts; d(a)=vv; d(b)=vv;
    elseif len>=3
        seg=x(a:b); d(a:b)=gradient(seg,Ts);
    end
    a=b+1;
end
end
