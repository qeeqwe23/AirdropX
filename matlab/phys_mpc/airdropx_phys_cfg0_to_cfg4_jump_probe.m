function report=airdropx_phys_cfg0_to_cfg4_jump_probe(projectRoot,opts)
%AIRDROPX_PHYS_CFG0_TO_CFG4_JUMP_PROBE Isolate the one-sample simultaneous-release physics path.
%
% Reconstruct the certified cfg0 trim state, switch the common MPC kernel
% directly to cfg4, solve one QP command, and evaluate the nonlinear JSBSim
% oracle at cfg4 twice.  This tests the actual code path used by a simultaneous
% four-payload release without running a 40 s mission and without sharing an
% Oracle instance with any previous scenario.
arguments
    projectRoot (1,1) string
    opts.OutputRoot (1,1) string
    opts.BankPath (1,1) string = ""
    opts.H (1,1) double {mustBeFinite} = 200
    opts.V (1,1) double {mustBeFinite,mustBePositive} = 50
    opts.FuelScale (1,1) double {mustBeFinite} = 1.0
end
if opts.BankPath=="", opts.BankPath=fullfile(projectRoot,"matlab","results","physics_mpc_v033","physics_bank.mat"); end
if ~isfolder(opts.OutputRoot), mkdir(opts.OutputRoot); end
statusPath=fullfile(opts.OutputRoot,"probe_status.txt");
markerPath=fullfile(opts.OutputRoot,"probe_complete.ok");
if isfile(markerPath), delete(markerPath); end
localStatus(statusPath,"STARTED","fresh MATLAB process; direct cfg0->cfg4 probe");
report=struct("pass",false,"completed_at",datetime("now"));
try
    schedule=airdropx_phys_mpc_build_cfg_schedule(opts.BankPath,opts.H,opts.V,opts.FuelScale);
    models=schedule.models;
    ctrl0=models{1}.ctrl;
    ctrl4=models{5}.ctrl;
    x0=ctrl0.xref;
    warm=zeros(ctrl4.m*ctrl4.N,1);
    sol=airdropx_phys_mpc_solve(ctrl4,x0,warm);
    if ~sol.feasible, error("AirdropX:PhysMPC:ProbeQPInfeasible","cfg4 QP infeasible from certified cfg0 trim state."); end
    u=sol.u;
    p4=models{5}.vertex.p;
    localStatus(statusPath,"ORACLE_INIT_START","initializing v0.3.3 Oracle");
    info=airdropx_phys_oracle_init(projectRoot);
    localStatus(statusPath,"ORACLE_INIT_DONE",sprintf("version=%s",string(info.version)));
    if ~contains(string(info.version),"v0.3.3"), error("AirdropX:PhysMPC:WrongOracle","Expected v0.3.3 Oracle."); end

    localStatus(statusPath,"CFG4_EVAL_1_START",sprintf("u=[%.9g %.9g]",u(1),u(2)));
    tic; [x1,d1]=airdropx_phys_step(x0,u,p4); t1=toc;
    localStatus(statusPath,"CFG4_EVAL_1_DONE",sprintf("elapsed=%.6gs mass=%.9g",t1,double(d1.mass_kg)));
    localStatus(statusPath,"CFG4_EVAL_2_START","repeatability call at identical x/u/cfg4");
    tic; [x2,d2]=airdropx_phys_step(x0,u,p4); t2=toc;
    localStatus(statusPath,"CFG4_EVAL_2_DONE",sprintf("elapsed=%.6gs",t2));

    expected=models{5}.vertex.trim.diag;
    stateRepeat=max(abs(double(x2(:))-double(x1(:))));
    massErr=double(d1.mass_kg)-double(expected.mass_kg);
    cgErr=double(d1.cg_x_m)-double(expected.cg_x_m);
    iyyErr=double(d1.Iyy_kgm2)-double(expected.Iyy_kgm2);
    alg1=localDiagLogical(d1,"algebraic_settle_converged");
    alg2=localDiagLogical(d2,"algebraic_settle_converged");
    finite=all(isfinite(x1)) && all(isfinite(x2)) && all(isfinite(u));
    pass=sol.feasible && finite && alg1 && alg2 && stateRepeat<=1e-12 && ...
        abs(massErr)<=1e-3 && abs(cgErr)<=1e-9 && abs(iyyErr)<=1e-6;
    report=struct("pass",pass,"completed_at",datetime("now"),"oracle_version",string(info.version), ...
        "x_cfg0",x0,"u_cfg4",u,"xnext_first",x1,"xnext_second",x2,"diag_first",d1,"diag_second",d2, ...
        "eval_time_first_s",t1,"eval_time_second_s",t2,"state_repeat_max_abs",stateRepeat, ...
        "mass_error_kg",massErr,"cg_error_m",cgErr,"Iyy_error_kgm2",iyyErr,"qp_exitflag",sol.exitflag);
    save(fullfile(opts.OutputRoot,"simultaneous_cfg4_probe.mat"),"report","-v7.3");
    localWriteSummary(report,fullfile(opts.OutputRoot,"probe_summary.txt"));
    fid=fopen(markerPath,"w");
    if fid<0, error("AirdropX:PhysMPC:MarkerWriteFailed","Could not write probe completion marker."); end
    fprintf(fid,"pass=%d\ncompleted=%s\n",logical(report.pass),char(datetime("now","Format","yyyy-MM-dd HH:mm:ss.SSS")));
    fclose(fid);
    localStatus(statusPath,"COMPLETE_MARKER_WRITTEN",sprintf("pass=%d",logical(report.pass)));
    pause(0.05);
catch ME
    report.pass=false; report.completed_at=datetime("now");
    report.error=struct("identifier",string(ME.identifier),"message",string(ME.message),"stack",ME.stack);
    save(fullfile(opts.OutputRoot,"simultaneous_cfg4_probe_failure.mat"),"report","-v7.3");
    localStatus(statusPath,"ERROR",sprintf("%s | %s",ME.identifier,ME.message));
    rethrow(ME);
end
end

function tf=localDiagLogical(d,name)
tf=isfield(d,name) && logical(d.(name));
end
function localStatus(path,phase,detail)
fid=fopen(path,"a"); if fid<0, return; end
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,"%s | %s | %s\n",char(datetime("now","Format","yyyy-MM-dd HH:mm:ss.SSS")),phase,detail);
end
function localWriteSummary(r,path)
fid=fopen(path,"w"); if fid<0, return; end
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,"Physics-MPC v0.5.2 cfg0->cfg4 direct jump probe\n");
fprintf(fid,"pass=%d\n",r.pass);
fprintf(fid,"oracle=%s\n",r.oracle_version);
fprintf(fid,"eval_time_first_s=%.9g\n",r.eval_time_first_s);
fprintf(fid,"eval_time_second_s=%.9g\n",r.eval_time_second_s);
fprintf(fid,"state_repeat_max_abs=%.9g\n",r.state_repeat_max_abs);
fprintf(fid,"mass_error_kg=%.9g\n",r.mass_error_kg);
fprintf(fid,"cg_error_m=%.9g\n",r.cg_error_m);
fprintf(fid,"Iyy_error_kgm2=%.9g\n",r.Iyy_error_kgm2);
fprintf(fid,"qp_exitflag=%d\n",r.qp_exitflag);
end
