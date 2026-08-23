function report=airdropx_phys_merge_speed_pilot_bank_v070(projectRoot,opts)
%AIRDROPX_PHYS_MERGE_SPEED_PILOT_BANK_V070 Merge four new speed slices + certified V50 vertices into one 75-vertex bank.
arguments
    projectRoot (1,1) string
    opts.SliceRoot (1,1) string
    opts.OutputBankPath (1,1) string
    opts.BaseBankPath (1,1) string = ""
end
if opts.BaseBankPath=="", opts.BaseBankPath=fullfile(projectRoot,"matlab","results","physics_mpc_v033","physics_bank.mat"); end
speeds=[40 45 50 55 60]; heights=[20 110 200]; cfgs=0:4;
vertices=cell(75,1); rows=table(); idx=0; info=struct();
for V=speeds
    if V==50
        S50=load(opts.BaseBankPath,"vertices","rows","info"); info=S50.info;
        for H=heights
            for cfg=cfgs
                idx=idx+1; mask=abs(S50.rows.H_m-H)<=1e-9 & abs(S50.rows.V_mps-50)<=1e-9 & S50.rows.cfg==cfg & abs(S50.rows.fuel_scale-1)<=1e-12;
                k=find(mask); if numel(k)~=1 || ~logical(S50.rows.pass(k)), error("AirdropX:PhysMPC:MissingV50Vertex","Missing certified V50 H=%.0f cfg=%d.",H,cfg); end
                vertices{idx}=S50.vertices{k}; rows=[rows;S50.rows(k,:)]; %#ok<AGROW>
            end
        end
    else
        dirV=fullfile(opts.SliceRoot,sprintf("V%03d",round(V))); p=fullfile(dirV,"physics_speed_slice.mat"); marker=fullfile(dirV,"slice_complete.ok");
        if ~isfile(p) || ~isfile(marker), error("AirdropX:PhysMPC:SpeedSliceMissing","Missing completed V=%.0f slice: %s",V,p); end
        S=load(p,"vertices","rows","info"); if height(S.rows)~=15 || ~all(S.rows.pass), error("AirdropX:PhysMPC:BadSpeedSlice","V=%.0f slice is incomplete or contains FAIL.",V); end
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
if idx~=75 || height(rows)~=75 || ~all(rows.pass), error("AirdropX:PhysMPC:BadPilotBank","Merged pilot bank must contain 75/75 PASS vertices."); end
% Common-controller audit across all 75 vertices.
base=[]; maxQ=0; maxR=0; maxSS=0; maxUS=0; sameHorizon=true; rhoMin=inf; rhoMax=-inf; dAMax=0; dBMax=0;
for k=1:75
    v=vertices{k}; q=double(v.Q); rr=double(v.R); ss=double(v.qrMeta.StateScale(:)); us=double(v.qrMeta.InputScale(:)); np=double(v.terminal.Np); nc=double(v.terminal.Nc);
    if isempty(base), base=struct("Q",q,"R",rr,"ss",ss,"us",us,"Np",np,"Nc",nc); else
        maxQ=max(maxQ,norm(q-base.Q,inf)); maxR=max(maxR,norm(rr-base.R,inf)); maxSS=max(maxSS,norm(ss-base.ss,inf)); maxUS=max(maxUS,norm(us-base.us,inf)); sameHorizon=sameHorizon && np==base.Np && nc==base.Nc;
    end
    rhoMin=min(rhoMin,double(v.terminal.rho)); rhoMax=max(rhoMax,double(v.terminal.rho)); dAMax=max(dAMax,double(v.lin.richardson_relerr_A)); dBMax=max(dBMax,double(v.lin.richardson_relerr_B));
end
bankAudit=struct("pass",maxQ<=1e-12 && maxR<=1e-12 && maxSS<=1e-12 && maxUS<=1e-12 && sameHorizon, ...
    "vertex_count",75,"max_Q_diff",maxQ,"max_R_diff",maxR,"max_state_scale_diff",maxSS,"max_input_scale_diff",maxUS,"same_horizon",sameHorizon,"Np",base.Np,"Nc",base.Nc,"rho_min",rhoMin,"rho_max",rhoMax,"dA_rel_max",dAMax,"dB_rel_max",dBMax);
if ~bankAudit.pass, error("AirdropX:PhysMPC:PilotBankNotUnified","Speed pilot bank failed common-controller audit."); end
optsSaved=opts; %#ok<NASGU>
outDir=fileparts(opts.OutputBankPath); if ~isfolder(outDir), mkdir(outDir); end
save(opts.OutputBankPath,"vertices","rows","info","bankAudit","optsSaved","-v7.3");
writetable(rows,fullfile(outDir,"physics_vertex_summary.csv"));
report=struct("version","Physics-MPC v0.7.0 speed pilot bank","pass",true,"bank_path",opts.OutputBankPath,"bank_audit",bankAudit,"rows",rows);
save(fullfile(outDir,"speed_pilot_bank_report.mat"),"report");
localWrite(report,fullfile(outDir,"speed_pilot_bank_summary.txt"));
end
function localWrite(r,path)
fid=fopen(path,"w"); if fid<0, return; end; c=onCleanup(@()fclose(fid)); %#ok<NASGU>
a=r.bank_audit;
fprintf(fid,"Physics-MPC v0.7.0 speed pilot bank\npass=%d\n",r.pass);
fprintf(fid,"grid_H_m=20 110 200\ngrid_V_mps=40 45 50 55 60\ncfg=0:4\nvertices=%d/75\n",a.vertex_count);
fprintf(fid,"common_Q_max_diff=%.9g\ncommon_R_max_diff=%.9g\ncommon_state_scale_max_diff=%.9g\ncommon_input_scale_max_diff=%.9g\nhorizon_same=%d\nNp=%d\nNc=%d\n",a.max_Q_diff,a.max_R_diff,a.max_state_scale_diff,a.max_input_scale_diff,a.same_horizon,a.Np,a.Nc);
fprintf(fid,"rho_cl_range=[%.9g %.9g]\ndA_rel_max=%.9g\ndB_rel_max=%.9g\n",a.rho_min,a.rho_max,a.dA_rel_max,a.dB_rel_max);
end
