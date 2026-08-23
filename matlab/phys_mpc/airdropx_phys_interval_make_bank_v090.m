function report=airdropx_phys_interval_make_bank_v090(masterBankPath,H,V,outputBankPath)
%AIRDROPX_PHYS_INTERVAL_MAKE_BANK_V090 Build a five-cfg local bank at arbitrary continuous H,V.
arguments
    masterBankPath (1,1) string
    H (1,1) double {mustBeFinite}
    V (1,1) double {mustBeFinite,mustBePositive}
    outputBankPath (1,1) string
end
if ~isfile(masterBankPath), error("AirdropX:PhysMPC:BankMissing","Master 475-point bank missing: %s",masterBankPath); end
S=load(masterBankPath,"vertices","rows","info","bankAudit");
if ~isfield(S,'vertices') || numel(S.vertices)~=475 || ~isfield(S,'rows') || height(S.rows)~=475
    error("AirdropX:PhysMPC:BadIntervalMasterBank","Expected the v0.8.2 475-row master bank.");
end
if ismember('usable',S.rows.Properties.VariableNames) && ~all(S.rows.usable)
    error("AirdropX:PhysMPC:IntervalMasterUnusable","Continuous interpolation requires all 475 anchor payloads usable.");
end
vertices=cell(5,1); metaByCfg=cell(5,1);
rows=table(zeros(5,1),zeros(5,1),zeros(5,1),ones(5,1),false(5,1),true(5,1),nan(5,1),nan(5,1),nan(5,1),nan(5,1),100*ones(5,1),strings(5,1),strings(5,1),strings(5,1), ...
    'VariableNames',{'H_m','V_mps','cfg','fuel_scale','pass','usable','trim_inf','dA_rel','dB_rel','rho_cl','Np','message','error_id','error_message'});
for cfg=0:4
    [v,m]=airdropx_phys_interval_interpolate_vertex_v090(S,H,V,cfg,Horizon=100); vertices{cfg+1}=v; metaByCfg{cfg+1}=m;
    rows.H_m(cfg+1)=H; rows.V_mps(cfg+1)=V; rows.cfg(cfg+1)=cfg; rows.rho_cl(cfg+1)=double(v.terminal.rho);
    if m.exact_grid
        sourceRow=S.rows(m.corner_row_indices(1),:); rows.pass(cfg+1)=logical(sourceRow.pass); if ismember('usable',sourceRow.Properties.VariableNames), rows.usable(cfg+1)=logical(sourceRow.usable); end
        if ismember('trim_inf',sourceRow.Properties.VariableNames), rows.trim_inf(cfg+1)=double(sourceRow.trim_inf); end
        if ismember('dA_rel',sourceRow.Properties.VariableNames), rows.dA_rel(cfg+1)=double(sourceRow.dA_rel); end
        if ismember('dB_rel',sourceRow.Properties.VariableNames), rows.dB_rel(cfg+1)=double(sourceRow.dB_rel); end
        rows.message(cfg+1)="exact v0.8.2 anchor node";
    else
        rows.pass(cfg+1)=false; rows.usable(cfg+1)=true; rows.message(cfg+1)="v0.9.0 continuous bilinear HxV interpolated model (not separately certified)";
    end
end
Q0=double(vertices{1}.Q); R0=double(vertices{1}.R); ss0=double(vertices{1}.qrMeta.StateScale(:)); us0=double(vertices{1}.qrMeta.InputScale(:));
maxQ=0; maxR=0; maxSS=0; maxUS=0; rho=zeros(5,1);
for k=1:5
    maxQ=max(maxQ,norm(double(vertices{k}.Q)-Q0,inf)); maxR=max(maxR,norm(double(vertices{k}.R)-R0,inf));
    maxSS=max(maxSS,norm(double(vertices{k}.qrMeta.StateScale(:))-ss0,inf)); maxUS=max(maxUS,norm(double(vertices{k}.qrMeta.InputScale(:))-us0,inf)); rho(k)=double(vertices{k}.terminal.rho);
end
common=max([maxQ maxR maxSS maxUS])<=1e-12;
if ~common, error("AirdropX:PhysMPC:IntervalControllerNotUnified","Interpolated cfg models lost unified Q/R/scales."); end
if isfield(S,'info'), info=S.info; else, info=struct(); end
bankAudit=struct("pass",false,"diagnostic_runnable",true,"continuous_interval",true,"common_controller_pass",common,"vertex_count",5,"certified_count",sum(rows.pass),"usable_count",sum(rows.usable), ...
    "max_Q_diff",maxQ,"max_R_diff",maxR,"max_state_scale_diff",maxSS,"max_input_scale_diff",maxUS,"same_horizon",true,"mpc_horizon_fixed",true,"Np",100,"Nc",100,"rho_min",min(rho),"rho_max",max(rho));
report=struct("version","Physics-MPC v0.9.0 continuous local HxV bank","H",H,"V",V,"exact_grid",all(cellfun(@(x)x.exact_grid,metaByCfg)), ...
    "source_master_bank",masterBankPath,"meta_by_cfg",{metaByCfg},"bank_audit",bankAudit,"rows",rows,"completed_at",datetime("now"));
outDir=fileparts(outputBankPath); if ~isfolder(outDir), mkdir(outDir); end
save(outputBankPath,"vertices","rows","info","bankAudit","report","-v7.3");
end
