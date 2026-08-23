function bank=airdropx_phys_runtime_prepare_bank_v101(masterBankPath)
%AIRDROPX_PHYS_RUNTIME_PREPARE_BANK_V101 Prepare indexed access to 475 usable HxVxcfg anchors.
arguments
    masterBankPath (1,1) string
end
if ~isfile(masterBankPath), error("AirdropX:PhysMPC:BankMissing","Master bank missing: %s",masterBankPath); end
S=load(masterBankPath,"vertices","rows","info");
if ~isfield(S,'vertices') || ~iscell(S.vertices) || numel(S.vertices)~=475 || ~isfield(S,'rows') || ~istable(S.rows) || height(S.rows)~=475
    error("AirdropX:PhysMPC:BadRuntimeBank","Expected the v0.8.2 475-row usable anchor bank.");
end
r=S.rows; need={'H_m','V_mps','cfg','fuel_scale','pass'};
for i=1:numel(need), if ~ismember(need{i},r.Properties.VariableNames), error("AirdropX:PhysMPC:BadRuntimeBank","rows missing %s",need{i}); end, end
if ~ismember('usable',r.Properties.VariableNames), r.usable=logical(r.pass); end
if ~all(r.usable), error("AirdropX:PhysMPC:RuntimeAnchorUnusable","Runtime interval requires all 475 anchors usable."); end
hGrid=(20:10:200)'; vGrid=[45 50 55 60 65]'; idx=zeros(numel(hGrid),numel(vGrid),5);
for ih=1:numel(hGrid), for iv=1:numel(vGrid), for c=0:4
    m=find(abs(r.H_m-hGrid(ih))<=1e-9 & abs(r.V_mps-vGrid(iv))<=1e-9 & r.cfg==c & abs(r.fuel_scale-1)<=1e-12);
    if numel(m)~=1, error("AirdropX:PhysMPC:RuntimeAnchorMissing","Expected one H=%g V=%g cfg=%d anchor.",hGrid(ih),vGrid(iv),c); end
    if isempty(S.vertices{m}), error("AirdropX:PhysMPC:RuntimeAnchorPayloadMissing","Anchor payload missing."); end
    idx(ih,iv,c+1)=m;
end, end, end
v0=S.vertices{idx(1,1,1)}; Q=double(v0.Q); R=double(v0.R); stateScale=double(v0.qrMeta.StateScale(:)); inputScale=double(v0.qrMeta.InputScale(:)); Ts=double(v0.p.Ts);
maxQ=0; maxR=0; maxSS=0; maxUS=0;
for k=1:numel(S.vertices)
    v=S.vertices{k}; maxQ=max(maxQ,norm(double(v.Q)-Q,inf)); maxR=max(maxR,norm(double(v.R)-R,inf));
    maxSS=max(maxSS,norm(double(v.qrMeta.StateScale(:))-stateScale,inf)); maxUS=max(maxUS,norm(double(v.qrMeta.InputScale(:))-inputScale,inf));
end
if max([maxQ maxR maxSS maxUS])>1e-12, error("AirdropX:PhysMPC:RuntimeControllerNotUnified","475 anchors do not share one Q/R/scales set."); end
pByCfg=cell(5,1); expectedMass=zeros(5,1); expectedCg=zeros(5,1); expectedIyy=zeros(5,1);
for c=0:4
    vv=S.vertices{idx(1,1,c+1)}; p=vv.p; p.cfgId=c; p.fuelScale=1; p.Ts=Ts; pByCfg{c+1}=p;
    d=vv.trim.diag; expectedMass(c+1)=double(d.mass_kg); expectedCg(c+1)=double(d.cg_x_m); expectedIyy(c+1)=double(d.Iyy_kgm2);
end
bank=struct("version","Physics-MPC v1.0.1 runtime indexed bank","source",masterBankPath,"vertices",{S.vertices},"rows",r,"hGrid",hGrid,"vGrid",vGrid,"index",idx, ...
    "Q",Q,"R",R,"stateScale",stateScale,"inputScale",inputScale,"Ts",Ts,"N",100,"umin",[-1;0],"umax",[1;1],"pByCfg",{pByCfg}, ...
    "expectedMass",expectedMass,"expectedCg",expectedCg,"expectedIyy",expectedIyy,"audit",struct("pass",true,"max_Q_diff",maxQ,"max_R_diff",maxR,"max_state_scale_diff",maxSS,"max_input_scale_diff",maxUS));
end
