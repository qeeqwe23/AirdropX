function report=airdropx_phys_build_speed_slice_v080(projectRoot,opts)
%AIRDROPX_PHYS_BUILD_SPEED_SLICE_V080 Build one full-height non-V50 speed slice.
% Formal envelope slice: H=20:10:200 m, cfg0..4, fuel=1, Ts=0.1 s.
% V50 is not rebuilt; it is reused from the validated v0.3.3 95-point bank.
arguments
    projectRoot (1,1) string
    opts.OutputRoot (1,1) string
    opts.Speed_mps (1,1) double {mustBeFinite,mustBePositive}
    opts.BaseBankPath (1,1) string = ""
    opts.Ts (1,1) double {mustBePositive} = 0.1
end
allowed=[45 55 60 65];
if ~any(abs(opts.Speed_mps-allowed)<=1e-12)
    error("AirdropX:PhysMPC:EnvelopeSpeedGrid","Full-envelope new speed slice must be one of [45 55 60 65] m/s; V50 is reused from the certified bank.");
end
if opts.BaseBankPath=="", opts.BaseBankPath=fullfile(projectRoot,"matlab","results","physics_mpc_v033","physics_bank.mat"); end
if ~isfolder(opts.OutputRoot), mkdir(opts.OutputRoot); end
status=fullfile(opts.OutputRoot,"slice_status.txt"); marker=fullfile(opts.OutputRoot,"slice_complete.ok");
if isfile(marker), delete(marker); end
localStatus(status,"STARTED",sprintf("V=%.1f H=20:10:200 cfg=0:4",opts.Speed_mps));
try
    preflight=airdropx_phys_preflight(projectRoot); %#ok<NASGU>
    mathSelftest=airdropx_phys_math_selftest(); %#ok<NASGU>
    localStatus(status,"ORACLE_INIT_START","initializing v0.3.3 physics Oracle");
    info=airdropx_phys_oracle_init(projectRoot);
    if ~isfield(info,"version") || ~contains(string(info.version),"v0.3.3")
        error("AirdropX:PhysMPC:OldOracle","Full HxV envelope requires Physics Oracle v0.3.3.");
    end
    localStatus(status,"ORACLE_INIT_DONE",string(info.version));
    [v50audit,~,~]=airdropx_phys_mpc_get_vertex(opts.BaseBankPath,200,50,0,1.0);
    auditX=[200; opts.Speed_mps; 0; v50audit.trim.x(4); 0; v50audit.trim.x(6); v50audit.trim.x(7)];
    auditU=v50audit.trim.u; auditP=struct("cfgId",0,"fuelScale",1.0,"Ts",opts.Ts);
    configurationAudit=airdropx_phys_oracle_config_audit(auditX,auditU,auditP,info);
    localStatus(status,"CONFIG_AUDIT_DONE",sprintf("pass=%d massErr=%.3g fuelErr=%.3g",configurationAudit.pass,max(abs(configurationAudit.cfg_mass_error_kg)),configurationAudit.fuel_probe.error_kg));
    heights=20:10:200; cfgs=0:4; n=numel(heights)*numel(cfgs);
    vertices=cell(n,1); rows=table(); idx=0;
    for H=heights
        for cfg=cfgs
            idx=idx+1;
            [v50,~,~]=airdropx_phys_mpc_get_vertex(opts.BaseBankPath,H,50,cfg,1.0);
            seed=v50.trim.z;
            localStatus(status,"VERTEX_START",sprintf("%d/%d H=%.0f V=%.0f cfg=%d",idx,n,H,opts.Speed_mps,cfg));
            vertex=airdropx_phys_build_vertex(H,opts.Speed_mps,cfg,1.0,seed,Ts=opts.Ts);
            vertices{idx}=vertex;
            trimNorm=max(norm(vertex.trim.scaled_physical_accel,inf),norm(vertex.trim.scaled_residual,inf));
            row=table(H,opts.Speed_mps,cfg,1.0,true,trimNorm,vertex.lin.richardson_relerr_A,vertex.lin.richardson_relerr_B,vertex.terminal.rho,vertex.terminal.Np,string(""), ...
                'VariableNames',{'H_m','V_mps','cfg','fuel_scale','pass','trim_inf','dA_rel','dB_rel','rho_cl','Np','message'});
            rows=[rows;row]; %#ok<AGROW>
            vdir=fullfile(opts.OutputRoot,sprintf("H%03.0f_V%05.1f_cfg%d_fuel1.00",H,opts.Speed_mps,cfg));
            if ~isfolder(vdir), mkdir(vdir); end
            save(fullfile(vdir,"physics_vertex.mat"),"vertex");
            writetable(rows,fullfile(opts.OutputRoot,"physics_vertex_summary.csv"));
            localStatus(status,"VERTEX_DONE",sprintf("H=%.0f V=%.0f cfg=%d rho=%.9g dA=%.3g dB=%.3g",H,opts.Speed_mps,cfg,vertex.terminal.rho,vertex.lin.richardson_relerr_A,vertex.lin.richardson_relerr_B));
        end
    end
    if height(rows)~=95 || ~all(rows.pass), error("AirdropX:PhysMPC:BadFullSlice","Full speed slice must contain 95/95 PASS vertices."); end
    optsSaved=opts; %#ok<NASGU>
    save(fullfile(opts.OutputRoot,"physics_speed_slice.mat"),"vertices","rows","info","mathSelftest","configurationAudit","optsSaved","-v7.3");
    report=struct("version","Physics-MPC v0.8.0 full-envelope speed-slice builder","pass",all(rows.pass),"speed_mps",opts.Speed_mps,"rows",rows,"vertex_count",height(rows));
    save(fullfile(opts.OutputRoot,"speed_slice_report.mat"),"report");
    fid=fopen(marker,"w"); if fid<0, error("AirdropX:PhysMPC:MarkerWriteFailed","Cannot write speed slice completion marker."); end
    fprintf(fid,"speed_mps=%.9g\npass=%d\nvertices=%d\ncompleted=%s\n",opts.Speed_mps,report.pass,height(rows),char(datetime("now","Format","yyyy-MM-dd HH:mm:ss.SSS"))); fclose(fid);
    localStatus(status,"COMPLETE_MARKER_WRITTEN","slice outputs durable; Oracle close deferred to MATLAB process exit");
    pause(0.05);
catch ME
    localStatus(status,"ERROR",sprintf("%s | %s",ME.identifier,ME.message));
    rethrow(ME);
end
end
function localStatus(path,phase,detail)
fid=fopen(path,"a"); if fid<0, return; end
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,"%s | %s | %s\n",char(datetime("now","Format","yyyy-MM-dd HH:mm:ss.SSS")),phase,detail);
end
