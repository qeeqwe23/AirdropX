function report=airdropx_phys_interval_dense_audit_v090(projectRoot,opts)
%AIRDROPX_PHYS_INTERVAL_DENSE_AUDIT_V090 Dense computational audit over the full continuous interval.
arguments
    projectRoot (1,1) string
    opts.MasterBankPath (1,1) string = ""
    opts.OutputRoot (1,1) string = ""
end
if opts.MasterBankPath=="", opts.MasterBankPath=fullfile(projectRoot,"matlab","results","physics_mpc_v082_fixed_horizon_envelope_bank","physics_full_envelope_bank_diagnostic.mat"); end
if opts.OutputRoot=="", opts.OutputRoot=fullfile(projectRoot,"matlab","results","physics_mpc_v090_continuous_interval_validation","dense_model_audit"); end
if ~isfolder(opts.OutputRoot), mkdir(opts.OutputRoot); end
S=load(opts.MasterBankPath,"vertices","rows","info","bankAudit");
Hs=(20:1:200)'; Vs=(45:1:65)'; n=numel(Hs)*numel(Vs);
Hcol=zeros(n,1); Vcol=zeros(n,1); Pass=false(n,1); RhoMax=nan(n,1); RhoMin=nan(n,1); MinTrimInputMargin=nan(n,1); UncertifiedSourceCornersMax=zeros(n,1); ErrorId=strings(n,1); ErrorMessage=strings(n,1);
k=0; tic0=tic;
for iv=1:numel(Vs)
    for ih=1:numel(Hs)
        k=k+1; H=Hs(ih); V=Vs(iv); Hcol(k)=H; Vcol(k)=V;
        try
            rhos=zeros(5,1); margins=zeros(5,1); unc=zeros(5,1);
            for cfg=0:4
                [v,m]=airdropx_phys_interval_interpolate_vertex_v090(S,H,V,cfg,Horizon=100);
                rhos(cfg+1)=double(v.terminal.rho); u=double(v.trim.u(:)); margins(cfg+1)=min([u(1)+1,1-u(1),u(2),1-u(2)]); unc(cfg+1)=sum(~m.corner_certified & m.weights>1e-14);
                if any(~isfinite(double(v.lin.Ad)),'all') || any(~isfinite(double(v.lin.Bd)),'all') || margins(cfg+1)<=0, error("AirdropX:PhysMPC:DenseAuditPayload","Nonfinite model or nonpositive trim-input margin."); end
            end
            RhoMax(k)=max(rhos); RhoMin(k)=min(rhos); MinTrimInputMargin(k)=min(margins); UncertifiedSourceCornersMax(k)=max(unc); Pass(k)=all(rhos<1) && MinTrimInputMargin(k)>0;
        catch ME
            Pass(k)=false; ErrorId(k)=string(ME.identifier); ErrorMessage(k)=string(ME.message);
        end
    end
    fprintf("[V090 DENSE] V=%g complete: %d/%d points, elapsed %.1fs\n",Vs(iv),k,n,toc(tic0));
end
T=table(Hcol,Vcol,Pass,RhoMin,RhoMax,MinTrimInputMargin,UncertifiedSourceCornersMax,ErrorId,ErrorMessage, ...
    'VariableNames',{'H_m','V_mps','pass','rho_min','rho_max','min_trim_input_margin','uncertified_source_corners_max','error_id','error_message'});
writetable(T,fullfile(opts.OutputRoot,"interval_dense_audit.csv"));
report=struct("version","Physics-MPC v0.9.0 dense interval model audit","pass",all(Pass),"points",n,"points_pass",sum(Pass), ...
    "grid_H","20:1:200","grid_V","45:1:65","rho_min",min(RhoMin,[],'omitnan'),"rho_max",max(RhoMax,[],'omitnan'), ...
    "min_trim_input_margin",min(MinTrimInputMargin,[],'omitnan'),"elapsed_s",toc(tic0),"completed_at",datetime("now"));
save(fullfile(opts.OutputRoot,"interval_dense_audit.mat"),"report","T");
localWrite(report,fullfile(opts.OutputRoot,"interval_dense_audit_summary.txt"));
fid=fopen(fullfile(opts.OutputRoot,"dense_audit_complete.ok"),"w"); if fid>=0, fprintf(fid,"pass=%d\npoints=%d\n",report.pass,n); fclose(fid); end
end
function localWrite(r,path)
fid=fopen(path,"w"); if fid<0, return; end; c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,"Physics-MPC v0.9.0 dense continuous-interval computational audit\n");
fprintf(fid,"pass=%d\npoints_pass=%d/%d\nH=20:1:200\nV=45:1:65\nrho_range=[%.9g %.9g]\nmin_trim_input_margin=%.9g\nelapsed_s=%.3f\n",r.pass,r.points_pass,r.points,r.rho_min,r.rho_max,r.min_trim_input_margin,r.elapsed_s);
end
