function report=airdropx_phys_merge_envelope_bank_v082(projectRoot,opts)
%AIRDROPX_PHYS_MERGE_ENVELOPE_BANK_V082 Merge all 475 attempted vertices for diagnostic full-flow use.
% Certification status is preserved exactly. A pass=false but usable=true
% vertex may be consumed only by explicit diagnostic missions.
arguments
    projectRoot (1,1) string
    opts.SliceRoot (1,1) string
    opts.OutputBankPath (1,1) string
    opts.BaseBankPath (1,1) string = ""
end
if opts.BaseBankPath=="", opts.BaseBankPath=fullfile(projectRoot,"matlab","results","physics_mpc_v033","physics_bank.mat"); end
speeds=[45 50 55 60 65]; heights=20:10:200; cfgs=0:4; expected=475;
vertices=cell(expected,1); rows=localEmptyRows(); idx=0; info=struct();
for V=speeds
    if V==50
        S=load(opts.BaseBankPath,"vertices","rows","info");
        if height(S.rows)~=95 || ~all(S.rows.pass), error("AirdropX:PhysMPC:BadV50Bank","Certified V50 bank must contain 95/95 PASS vertices."); end
        if isempty(fieldnames(info)), info=S.info; end
        srcRows=localNormalizeRows(S.rows);
    else
        dirV=fullfile(opts.SliceRoot,sprintf("V%03d",round(V))); p=fullfile(dirV,"physics_speed_slice.mat"); marker=fullfile(dirV,"slice_complete.ok");
        if ~isfile(p) || ~isfile(marker), error("AirdropX:PhysMPC:SpeedSliceMissing","Missing completed diagnostic V=%.0f slice: %s",V,p); end
        S=load(p,"vertices","rows","info");
        if height(S.rows)~=95, error("AirdropX:PhysMPC:BadSpeedSlice","V=%.0f diagnostic slice did not attempt all 95 vertices.",V); end
        if isempty(fieldnames(info)), info=S.info; end
        srcRows=localNormalizeRows(S.rows);
    end
    for H=heights
        for cfg=cfgs
            idx=idx+1; mask=abs(srcRows.H_m-H)<=1e-9 & abs(srcRows.V_mps-V)<=1e-9 & srcRows.cfg==cfg & abs(srcRows.fuel_scale-1)<=1e-12;
            k=find(mask); if numel(k)~=1, error("AirdropX:PhysMPC:EnvelopeGridNotUnique","Expected one H=%.0f V=%.0f cfg=%d vertex, found %d.",H,V,cfg,numel(k)); end
            vertices{idx}=S.vertices{k}; rows=[rows;srcRows(k,:)]; %#ok<AGROW>
        end
    end
end
if idx~=expected || height(rows)~=expected, error("AirdropX:PhysMPC:BadEnvelopeBank","Diagnostic merged bank must contain 475 attempted rows."); end
for V=speeds, for H=heights, for cfg=cfgs
    mask=abs(rows.H_m-H)<=1e-9 & abs(rows.V_mps-V)<=1e-9 & rows.cfg==cfg & abs(rows.fuel_scale-1)<=1e-12;
    if nnz(mask)~=1, error("AirdropX:PhysMPC:EnvelopeGridNotUnique","Expected one H=%.0f V=%.0f cfg=%d row, found %d.",H,V,cfg,nnz(mask)); end
end, end, end

% Common-controller audit over every computationally usable vertex.
% v0.8.2 deliberately separates the ACTUAL MPC horizon from each vertex's
% auto-horizon diagnostic. The runtime controller is one fixed Np=Nc=100
% kernel everywhere. vertex.terminal.Np/Nc are retained only as diagnostics.
mpcNp=100; mpcNc=100;
base=[]; maxQ=0; maxR=0; maxSS=0; maxUS=0; rhoMin=inf; rhoMax=-inf; dAMax=0; dBMax=0; usablePayloadCount=0;
reqNpMin=inf; reqNpMax=-inf; reqNcMin=inf; reqNcMax=-inf;
for k=1:expected
    if ~rows.usable(k), continue; end
    v=vertices{k};
    if isempty(v), rows.usable(k)=false; continue; end
    q=double(v.Q); rr=double(v.R); ss=double(v.qrMeta.StateScale(:)); us=double(v.qrMeta.InputScale(:));
    reqNp=double(v.terminal.Np); reqNc=double(v.terminal.Nc);
    if isempty(base), base=struct("Q",q,"R",rr,"ss",ss,"us",us); else
        maxQ=max(maxQ,norm(q-base.Q,inf)); maxR=max(maxR,norm(rr-base.R,inf)); maxSS=max(maxSS,norm(ss-base.ss,inf)); maxUS=max(maxUS,norm(us-base.us,inf));
    end
    usablePayloadCount=usablePayloadCount+1; rhoMin=min(rhoMin,double(v.terminal.rho)); rhoMax=max(rhoMax,double(v.terminal.rho));
    dAMax=max(dAMax,double(v.lin.richardson_relerr_A)); dBMax=max(dBMax,double(v.lin.richardson_relerr_B));
    reqNpMin=min(reqNpMin,reqNp); reqNpMax=max(reqNpMax,reqNp); reqNcMin=min(reqNcMin,reqNc); reqNcMax=max(reqNcMax,reqNc);
end
if isempty(base), error("AirdropX:PhysMPC:NoUsableVertices","Diagnostic bank contains no usable vertex payloads."); end
terminalHorizonSame=(reqNpMin==reqNpMax) && (reqNcMin==reqNcMax);
commonUnified=maxQ<=1e-12 && maxR<=1e-12 && maxSS<=1e-12 && maxUS<=1e-12;
certificationPass=all(rows.pass) && commonUnified;
diagnosticRunnable=all(rows.usable) && usablePayloadCount==expected && commonUnified;
bankAudit=struct("pass",certificationPass,"certification_pass",certificationPass,"diagnostic_runnable",diagnosticRunnable, ...
    "common_controller_pass",commonUnified,"vertex_count",expected,"certified_count",sum(rows.pass),"usable_count",sum(rows.usable), ...
    "uncertified_usable_count",sum(~rows.pass & rows.usable),"hard_unusable_count",sum(~rows.usable), ...
    "max_Q_diff",maxQ,"max_R_diff",maxR,"max_state_scale_diff",maxSS,"max_input_scale_diff",maxUS, ...
    "same_horizon",true,"mpc_horizon_fixed",true,"Np",mpcNp,"Nc",mpcNc, ...
    "terminal_horizon_same",terminalHorizonSame,"Np_required_min",reqNpMin,"Np_required_max",reqNpMax,"Nc_required_min",reqNcMin,"Nc_required_max",reqNcMax, ...
    "rho_min",rhoMin,"rho_max",rhoMax,"dA_rel_max",dAMax,"dB_rel_max",dBMax);

Vcol=speeds(:); VertexCount=zeros(5,1); CertifiedCount=zeros(5,1); UsableCount=zeros(5,1); dAMaxV=nan(5,1); dBMaxV=nan(5,1); RhoMin=nan(5,1); RhoMax=nan(5,1);
for j=1:5
    m=abs(rows.V_mps-speeds(j))<=1e-9; VertexCount(j)=nnz(m); CertifiedCount(j)=sum(rows.pass(m)); UsableCount(j)=sum(rows.usable(m));
    mu=m & rows.usable & isfinite(rows.rho_cl); if any(mu), RhoMin(j)=min(rows.rho_cl(mu)); RhoMax(j)=max(rows.rho_cl(mu)); dAMaxV(j)=max(rows.dA_rel(mu)); dBMaxV(j)=max(rows.dB_rel(mu)); end
end
PerSpeed=table(Vcol,VertexCount,CertifiedCount,UsableCount,RhoMin,RhoMax,dAMaxV,dBMaxV, ...
    'VariableNames',{'V_mps','vertex_count','certified_count','usable_count','rho_min','rho_max','dA_rel_max','dB_rel_max'});
outDir=fileparts(opts.OutputBankPath); if ~isfolder(outDir), mkdir(outDir); end; optsSaved=opts; %#ok<NASGU>
save(opts.OutputBankPath,"vertices","rows","info","bankAudit","PerSpeed","optsSaved","-v7.3");
writetable(rows,fullfile(outDir,"physics_vertex_summary.csv")); writetable(PerSpeed,fullfile(outDir,"physics_per_speed.csv"));
Uncert=rows(~rows.pass,:); writetable(Uncert,fullfile(outDir,"physics_uncertified_vertices.csv"));
report=struct("version","Physics-MPC v0.8.2 fixed-horizon diagnostic full HxV envelope bank","pass",certificationPass,"diagnostic_runnable",diagnosticRunnable, ...
    "bank_path",opts.OutputBankPath,"bank_audit",bankAudit,"per_speed",PerSpeed,"rows",rows);
save(fullfile(outDir,"envelope_bank_report.mat"),"report"); localWrite(report,fullfile(outDir,"envelope_bank_summary.txt"));
end

function r=localNormalizeRows(r)
if ~ismember('usable',r.Properties.VariableNames), r.usable=logical(r.pass); end
if ~ismember('message',r.Properties.VariableNames), r.message=strings(height(r),1); else, r.message=string(r.message); end
if ~ismember('error_id',r.Properties.VariableNames), r.error_id=strings(height(r),1); else, r.error_id=string(r.error_id); end
if ~ismember('error_message',r.Properties.VariableNames), r.error_message=strings(height(r),1); else, r.error_message=string(r.error_message); end
keep={'H_m','V_mps','cfg','fuel_scale','pass','usable','trim_inf','dA_rel','dB_rel','rho_cl','Np','message','error_id','error_message'};
r=r(:,keep);
end
function T=localEmptyRows()
T=table(zeros(0,1),zeros(0,1),zeros(0,1),zeros(0,1),false(0,1),false(0,1),zeros(0,1),zeros(0,1),zeros(0,1),zeros(0,1),zeros(0,1),strings(0,1),strings(0,1),strings(0,1), ...
    'VariableNames',{'H_m','V_mps','cfg','fuel_scale','pass','usable','trim_inf','dA_rel','dB_rel','rho_cl','Np','message','error_id','error_message'});
end
function localWrite(r,path)
fid=fopen(path,"w"); if fid<0, return; end; c=onCleanup(@()fclose(fid)); %#ok<NASGU>
a=r.bank_audit;
fprintf(fid,"Physics-MPC v0.8.2 fixed-horizon diagnostic full HxV envelope bank\n");
fprintf(fid,"formal_certification_pass=%d\ndiagnostic_runnable=%d\n",a.certification_pass,a.diagnostic_runnable);
fprintf(fid,"grid_H_m=20:10:200\ngrid_V_mps=45 50 55 60 65\ncfg=0:4\nvertices_attempted=%d/475\ncertified=%d/475\nusable=%d/475\nuncertified_usable=%d\nhard_unusable=%d\n", ...
    a.vertex_count,a.certified_count,a.usable_count,a.uncertified_usable_count,a.hard_unusable_count);
fprintf(fid,"common_controller_pass=%d\ncommon_Q_max_diff=%.9g\ncommon_R_max_diff=%.9g\ncommon_state_scale_max_diff=%.9g\ncommon_input_scale_max_diff=%.9g\nmpc_horizon_fixed=%d\nhorizon_same=%d\nNp=%d\nNc=%d\nterminal_horizon_same=%d\nNp_required_range=[%.9g %.9g]\nNc_required_range=[%.9g %.9g]\n", ...
    a.common_controller_pass,a.max_Q_diff,a.max_R_diff,a.max_state_scale_diff,a.max_input_scale_diff,a.mpc_horizon_fixed,a.same_horizon,a.Np,a.Nc,a.terminal_horizon_same,a.Np_required_min,a.Np_required_max,a.Nc_required_min,a.Nc_required_max);
fprintf(fid,"rho_cl_range=[%.9g %.9g]\ndA_rel_max=%.9g\ndB_rel_max=%.9g\n",a.rho_min,a.rho_max,a.dA_rel_max,a.dB_rel_max);
end
