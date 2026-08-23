function report=airdropx_phys_merge_envelope_bank_v080(projectRoot,opts)
%AIRDROPX_PHYS_MERGE_ENVELOPE_BANK_V080 Merge four full speed slices + certified V50 into one 475-vertex bank.
arguments
    projectRoot (1,1) string
    opts.SliceRoot (1,1) string
    opts.OutputBankPath (1,1) string
    opts.BaseBankPath (1,1) string = ""
end
if opts.BaseBankPath=="", opts.BaseBankPath=fullfile(projectRoot,"matlab","results","physics_mpc_v033","physics_bank.mat"); end
speeds=[45 50 55 60 65]; heights=20:10:200; cfgs=0:4;
expected=numel(speeds)*numel(heights)*numel(cfgs);
vertices=cell(expected,1); rows=table(); idx=0; info=struct();
for V=speeds
    if V==50
        S50=load(opts.BaseBankPath,"vertices","rows","info"); info=S50.info;
        if height(S50.rows)~=95 || ~all(S50.rows.pass), error("AirdropX:PhysMPC:BadV50Bank","Certified V50 bank must contain 95/95 PASS vertices."); end
        for H=heights
            for cfg=cfgs
                idx=idx+1; mask=abs(S50.rows.H_m-H)<=1e-9 & abs(S50.rows.V_mps-50)<=1e-9 & S50.rows.cfg==cfg & abs(S50.rows.fuel_scale-1)<=1e-12;
                k=find(mask); if numel(k)~=1 || ~logical(S50.rows.pass(k)), error("AirdropX:PhysMPC:MissingV50Vertex","Missing certified V50 H=%.0f cfg=%d.",H,cfg); end
                vertices{idx}=S50.vertices{k}; rows=[rows;S50.rows(k,:)]; %#ok<AGROW>
            end
        end
    else
        dirV=fullfile(opts.SliceRoot,sprintf("V%03d",round(V))); p=fullfile(dirV,"physics_speed_slice.mat"); marker=fullfile(dirV,"slice_complete.ok");
        if ~isfile(p) || ~isfile(marker), error("AirdropX:PhysMPC:SpeedSliceMissing","Missing completed V=%.0f full slice: %s",V,p); end
        S=load(p,"vertices","rows","info"); if height(S.rows)~=95 || ~all(S.rows.pass), error("AirdropX:PhysMPC:BadSpeedSlice","V=%.0f full slice is incomplete or contains FAIL.",V); end
        if isempty(fieldnames(info)), info=S.info; end
        for H=heights
            for cfg=cfgs
                idx=idx+1; mask=abs(S.rows.H_m-H)<=1e-9 & abs(S.rows.V_mps-V)<=1e-9 & S.rows.cfg==cfg & abs(S.rows.fuel_scale-1)<=1e-12;
                k=find(mask); if numel(k)~=1, error("AirdropX:PhysMPC:BadSpeedSlice","V=%.0f H=%.0f cfg=%d is not unique.",V,H,cfg); end
                vertices{idx}=S.vertices{k}; rows=[rows;S.rows(k,:)]; %#ok<AGROW>
            end
        end
    end
end
if idx~=expected || height(rows)~=expected || ~all(rows.pass), error("AirdropX:PhysMPC:BadEnvelopeBank","Merged envelope bank must contain 475/475 PASS vertices."); end
% Exact grid uniqueness audit.
for V=speeds, for H=heights, for cfg=cfgs
    mask=abs(rows.H_m-H)<=1e-9 & abs(rows.V_mps-V)<=1e-9 & rows.cfg==cfg & abs(rows.fuel_scale-1)<=1e-12;
    if nnz(mask)~=1, error("AirdropX:PhysMPC:EnvelopeGridNotUnique","Expected one H=%.0f V=%.0f cfg=%d vertex, found %d.",H,V,cfg,nnz(mask)); end
end, end, end
% Common-controller audit across all 475 vertices.
base=[]; maxQ=0; maxR=0; maxSS=0; maxUS=0; sameHorizon=true; rhoMin=inf; rhoMax=-inf; dAMax=0; dBMax=0;
for k=1:expected
    v=vertices{k}; q=double(v.Q); rr=double(v.R); ss=double(v.qrMeta.StateScale(:)); us=double(v.qrMeta.InputScale(:)); np=double(v.terminal.Np); nc=double(v.terminal.Nc);
    if isempty(base), base=struct("Q",q,"R",rr,"ss",ss,"us",us,"Np",np,"Nc",nc); else
        maxQ=max(maxQ,norm(q-base.Q,inf)); maxR=max(maxR,norm(rr-base.R,inf)); maxSS=max(maxSS,norm(ss-base.ss,inf)); maxUS=max(maxUS,norm(us-base.us,inf)); sameHorizon=sameHorizon && np==base.Np && nc==base.Nc;
    end
    rhoMin=min(rhoMin,double(v.terminal.rho)); rhoMax=max(rhoMax,double(v.terminal.rho)); dAMax=max(dAMax,double(v.lin.richardson_relerr_A)); dBMax=max(dBMax,double(v.lin.richardson_relerr_B));
end
bankAudit=struct("pass",maxQ<=1e-12 && maxR<=1e-12 && maxSS<=1e-12 && maxUS<=1e-12 && sameHorizon, ...
    "vertex_count",expected,"max_Q_diff",maxQ,"max_R_diff",maxR,"max_state_scale_diff",maxSS,"max_input_scale_diff",maxUS,"same_horizon",sameHorizon,"Np",base.Np,"Nc",base.Nc,"rho_min",rhoMin,"rho_max",rhoMax,"dA_rel_max",dAMax,"dB_rel_max",dBMax);
if ~bankAudit.pass, error("AirdropX:PhysMPC:EnvelopeBankNotUnified","Full HxV bank failed common-controller audit."); end
% Per-speed physics summary.
Vcol=speeds(:); VertexCount=zeros(numel(speeds),1); RhoMin=zeros(numel(speeds),1); RhoMax=zeros(numel(speeds),1); dAMaxV=zeros(numel(speeds),1); dBMaxV=zeros(numel(speeds),1);
for j=1:numel(speeds)
    m=abs(rows.V_mps-speeds(j))<=1e-9; VertexCount(j)=nnz(m); RhoMin(j)=min(rows.rho_cl(m)); RhoMax(j)=max(rows.rho_cl(m)); dAMaxV(j)=max(rows.dA_rel(m)); dBMaxV(j)=max(rows.dB_rel(m));
end
PerSpeed=table(Vcol,VertexCount,RhoMin,RhoMax,dAMaxV,dBMaxV,'VariableNames',{'V_mps','vertex_count','rho_min','rho_max','dA_rel_max','dB_rel_max'});
optsSaved=opts; %#ok<NASGU>
outDir=fileparts(opts.OutputBankPath); if ~isfolder(outDir), mkdir(outDir); end
save(opts.OutputBankPath,"vertices","rows","info","bankAudit","PerSpeed","optsSaved","-v7.3");
writetable(rows,fullfile(outDir,"physics_vertex_summary.csv")); writetable(PerSpeed,fullfile(outDir,"physics_per_speed.csv"));
report=struct("version","Physics-MPC v0.8.0 full HxV envelope bank","pass",true,"bank_path",opts.OutputBankPath,"bank_audit",bankAudit,"per_speed",PerSpeed,"rows",rows);
save(fullfile(outDir,"envelope_bank_report.mat"),"report"); localWrite(report,fullfile(outDir,"envelope_bank_summary.txt"));
end
function localWrite(r,path)
fid=fopen(path,"w"); if fid<0, return; end; c=onCleanup(@()fclose(fid)); %#ok<NASGU>
a=r.bank_audit;
fprintf(fid,"Physics-MPC v0.8.0 full HxV envelope bank\npass=%d\n",r.pass);
fprintf(fid,"grid_H_m=20:10:200\ngrid_V_mps=45 50 55 60 65\ncfg=0:4\nvertices=%d/475\n",a.vertex_count);
fprintf(fid,"common_Q_max_diff=%.9g\ncommon_R_max_diff=%.9g\ncommon_state_scale_max_diff=%.9g\ncommon_input_scale_max_diff=%.9g\nhorizon_same=%d\nNp=%d\nNc=%d\n",a.max_Q_diff,a.max_R_diff,a.max_state_scale_diff,a.max_input_scale_diff,a.same_horizon,a.Np,a.Nc);
fprintf(fid,"rho_cl_range=[%.9g %.9g]\ndA_rel_max=%.9g\ndB_rel_max=%.9g\n",a.rho_min,a.rho_max,a.dA_rel_max,a.dB_rel_max);
end
