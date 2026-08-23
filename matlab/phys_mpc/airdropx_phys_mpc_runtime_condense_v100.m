function ctrl=airdropx_phys_mpc_runtime_condense_v100(bank,Hseq,Vseq,cfgSeq)
%AIRDROPX_PHYS_MPC_RUNTIME_CONDENSE_V100 Condense one continuously scheduled H/V/cfg horizon.
arguments
    bank (1,1) struct
    Hseq (1,:) double
    Vseq (1,:) double
    cfgSeq (1,:) double
end
N=numel(Hseq)-1; if N~=100 || numel(Vseq)~=N+1 || numel(cfgSeq)~=N+1, error("AirdropX:PhysMPC:RuntimeSequenceSize","Runtime horizon must contain 101 H/V/cfg samples."); end
if any(~isfinite(Hseq))||any(~isfinite(Vseq))||any(~isfinite(cfgSeq))||any(cfgSeq<0)||any(cfgSeq>4)||any(cfgSeq~=round(cfgSeq)), error("AirdropX:PhysMPC:RuntimeBadSequence","Bad runtime sequence."); end
n=7; m=2; Q=bank.Q; R=bank.R; Astage=zeros(n,n,N); Bstage=zeros(n,m,N); Rstate=zeros(n,N+1); Rinput=zeros(m,N); certCount=zeros(N+1,1);
for j=1:N
    s=airdropx_phys_runtime_interpolate_stage_v100(bank,Hseq(j),Vseq(j),cfgSeq(j),NeedTerminal=false); Astage(:,:,j)=s.A; Bstage(:,:,j)=s.B; Rstate(:,j)=s.xref; Rinput(:,j)=s.uref; certCount(j)=s.source_certified_count;
end
st=airdropx_phys_runtime_interpolate_stage_v100(bank,Hseq(end),Vseq(end),cfgSeq(end),NeedTerminal=true); Rstate(:,end)=st.xref; certCount(end)=st.source_certified_count; Pterm=st.P;
nu=m*N; H=zeros(nu); Fdx=zeros(nu,n); fAffine=zeros(nu,1); M=eye(n); G=zeros(n,nu); d=zeros(n,1);
for j=1:N
    A=Astage(:,:,j); B=Bstage(:,:,j); cols=(j-1)*m+(1:m); c=Rstate(:,j)-Rstate(:,j+1); M=A*M; G=A*G; G(:,cols)=G(:,cols)+B; d=A*d+c;
    if j<N, W=Q; else, W=Pterm; end
    H=H+2*(G'*W*G); Fdx=Fdx+2*(G'*W*M); fAffine=fAffine+2*(G'*W*d); H(cols,cols)=H(cols,cols)+2*R;
end
H=(H+H')/2; lb=zeros(nu,1); ub=zeros(nu,1);
for j=1:N, cols=(j-1)*m+(1:m); lb(cols)=bank.umin-Rinput(:,j); ub(cols)=bank.umax-Rinput(:,j); end
[~,flag]=chol(H); if flag~=0, error("AirdropX:PhysMPC:RuntimeNonConvexQP","Runtime Hessian is not positive definite."); end
qpopts=optimoptions("quadprog","Algorithm","active-set","Display","off","ConstraintTolerance",1e-8,"OptimalityTolerance",1e-8,"StepTolerance",1e-9,"MaxIterations",4000);
ctrl=struct("version","Physics-MPC v1.0.0 runtime time-varying QP","cfgSeq",cfgSeq,"Hseq",Hseq,"Vseq",Vseq,"N",N,"n",n,"m",m,"nu",nu,"ns",0,"nz",nu, ...
    "Astage",Astage,"Bstage",Bstage,"Rstate",Rstate,"Rinput",Rinput,"Q",Q,"R",R,"P",Pterm,"H",H,"Fdx",Fdx,"fAffine",fAffine, ...
    "Aineq",zeros(0,nu),"bineq0",zeros(0,1),"bineqDx",zeros(0,n),"lb",lb,"ub",ub,"umin",bank.umin,"umax",bank.umax,"stateScale",bank.stateScale,"inputScale",bank.inputScale, ...
    "enableQSoft",false,"options",qpopts,"firstCfg",cfgSeq(1),"terminalCfg",cfgSeq(end),"source_certified_count_min",min(certCount));
end
