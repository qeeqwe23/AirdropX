function report=airdropx_phys_smoke(projectRoot,opts)
%AIRDROPX_PHYS_SMOKE Full preflight gate before any bank build.
arguments
    projectRoot (1,1) string
    opts.H (1,1) double = 200
    opts.V (1,1) double = 50
    opts.CfgId (1,1) double = 0
    opts.FuelScale (1,1) double = 1.0
    opts.Ts (1,1) double = 0.1
    opts.OutputRoot (1,1) string = ""
end
if opts.OutputRoot==""
    opts.OutputRoot=fullfile(projectRoot,"matlab","results","physics_mpc_v033");
end
if ~isfolder(opts.OutputRoot), mkdir(opts.OutputRoot); end
fprintf("\n=== Physics-MPC v0.3.3 Smoke ===\n");
fprintf("H=%.3f m  V=%.3f m/s  cfg=%d  fuel=%.3f  Ts=%.4f s\n", ...
    opts.H,opts.V,opts.CfgId,opts.FuelScale,opts.Ts);

report=struct();
try
    report.preflight=airdropx_phys_preflight(projectRoot);
    report.math_selftest=airdropx_phys_math_selftest();
    fprintf("Math self-test: rho=%.9g Np=%d (PASS)\n",report.math_selftest.rho,report.math_selftest.Np);
    info=airdropx_phys_oracle_init(projectRoot);
    cleanup=onCleanup(@()airdropx_jsbsim_oracle_mex("close")); %#ok<NASGU>
    report.info=info;
    if ~isfield(info,"version") || ~contains(string(info.version),"v0.3.3")
        error("AirdropX:PhysMPC:OldOracle", ...
            "Physics Oracle is not v0.3.3. Rebuild it once without -SkipBuild before running Smoke.");
    end
    fprintf("Oracle: %s\n",string(info.version));

    p=struct("cfgId",opts.CfgId,"fuelScale",opts.FuelScale,"Ts",opts.Ts);
    z0=airdropx_phys_seed_from_existing("N1",info.base_n1,"N2",info.base_n2);
    fprintf("Seed z0=[theta %.6f, elev %.6f, thr %.6f, N1 %.4f, N2 %.4f]\n",z0);

    trim=airdropx_phys_trim_discrete(opts.H,opts.V,p,z0);
    report.trim=trim;
    localPrintTrim(trim);
    if ~trim.pass
        report.pass=false;
        localSave(report,opts.OutputRoot,"physics_smoke_failure.mat");
        rr=trim.rate_residual(:);
        aa=trim.physical_accel_residual(:);
        error("AirdropX:PhysMPC:SmokeTrimFailed", ...
            ["Trim failed: exitflag=%d maxScaledAccel=%.6g maxScaledRate=%.6g hStep=%.6g m " + ...
             "accel=[%.6g %.6g %.6g] alphadot=%.6g scaledAlphaDot=%.6g algIter=%g algErr=%.3g rates=[%.6g %.6g %.6g %.6g %.6g %.6g]."], ...
            trim.exitflag,norm(trim.scaled_physical_accel,inf),norm(trim.scaled_residual,inf),trim.h_step_m, ...
            aa(1),aa(2),aa(3),trim.alphadot_residual,trim.scaled_alphadot, ...
            trim.diag.algebraic_settle_iterations,trim.diag.algebraic_settle_error, ...
            rr(1),rr(2),rr(3),rr(4),rr(5),rr(6));
    end

    selftest=airdropx_phys_oracle_selftest(trim.x,trim.u,p,ThrowOnFail=false);
    report.selftest=selftest;
    if ~selftest.pass
        report.pass=false;
        localSave(report,opts.OutputRoot,"physics_smoke_failure.mat");
        error("AirdropX:PhysMPC:OracleSelfTestFailed", ...
            ["Oracle self-test failed: deterministic=%d maxSpread=%.3g " + ...
             "pathIndependent=%d pathErrorInf=%.3g pathError=[%.3g %.3g %.3g %.3g %.3g %.3g %.3g] " + ...
             "physicalOk=%d fuelFrozen=%d controlOk=%d algebraicOk=%d algIter=%.0f algErr=%.3g semigroupPass=%d semigroupMaxScaled=%.3g"], ...
            selftest.deterministic,selftest.max_abs_state_spread,selftest.path_independent, ...
            selftest.max_abs_path_error, ...
            selftest.path_error(1),selftest.path_error(2),selftest.path_error(3),selftest.path_error(4), ...
            selftest.path_error(5),selftest.path_error(6),selftest.path_error(7), ...
            selftest.physical_ok,selftest.fuel_frozen,selftest.control_ok,selftest.algebraic_ok, ...
            selftest.algebraic_iterations,selftest.algebraic_error,selftest.semigroup.pass,selftest.semigroup.max_scaled);
    end
    fprintf("Oracle algebraic closure: iter=%.0f err=%.3g alphadot=%.6g rad/s (PASS)\n", ...
        selftest.algebraic_iterations,selftest.algebraic_error,selftest.diag.alphadot_radps);
    fprintf("Oracle repeatability: max state spread = %.3g (PASS)\n",selftest.max_abs_state_spread);
    fprintf("Oracle path independence: max state error = %.3g (PASS)\n",selftest.max_abs_path_error);
    fprintf("Oracle Markov closure: 2Ts vs Ts+Ts maxScaled = %.6g (PASS)\n", ...
        selftest.semigroup.max_scaled);

    cfgAudit=airdropx_phys_oracle_config_audit(trim.x,trim.u,p,info);
    report.configuration_audit=cfgAudit;
    fprintf("Configuration audit: cfg mass max error = %.6g kg, fuel probe error = %.6g kg (PASS)\n", ...
        max(abs(cfgAudit.cfg_mass_error_kg)),cfgAudit.fuel_probe.error_kg);

    lin=airdropx_phys_linearize_discrete(trim.x,trim.u,p);
    report.lin=lin;
    fprintf("Jacobian: dA_rel=%.6g  dB_rel=%.6g  rho_open=%.9g\n", ...
        lin.richardson_relerr_A,lin.richardson_relerr_B,lin.spectral_radius_open);
    if ~lin.converged
        report.pass=false;
        localSave(report,opts.OutputRoot,"physics_smoke_failure.mat");
        error("AirdropX:PhysMPC:SmokeDerivativeFailed", ...
            "Richardson check failed: dA=%.6g dB=%.6g.", ...
            lin.richardson_relerr_A,lin.richardson_relerr_B);
    end

    [Q,R,qrMeta]=airdropx_phys_bryson_qr;
    terminal=airdropx_phys_autohorizon(lin.Ad,lin.Bd,Q,R);
    report.Q=Q; report.R=R; report.qrMeta=qrMeta; report.terminal=terminal;
    fprintf("Terminal: rho_cl=%.9g  Np=Nc=%d\n",terminal.rho,terminal.Np);

    report.pass=true;
    report.completed_at=datetime("now");
    localSave(report,opts.OutputRoot,"physics_smoke_pass.mat");
    fprintf("=== Physics-MPC v0.3.3 Smoke PASS ===\n\n");
catch ME
    if ~isfield(report,"pass"), report.pass=false; end
    report.error=struct("identifier",string(ME.identifier),"message",string(ME.message),"stack",ME.stack);
    localSave(report,opts.OutputRoot,"physics_smoke_failure.mat");
    fprintf(2,"\nSMOKE FAIL [%s]\n%s\n",ME.identifier,ME.message);
    if ~isempty(ME.stack)
        fprintf(2,"at %s line %d\n",ME.stack(1).name,ME.stack(1).line);
    end
    rethrow(ME);
end
end

function localPrintTrim(t)
fprintf("Trim method: %s\n",t.method);
fprintf("Seed source: %s\n",t.seed_source);
if isstruct(t.builtin_trim) && isfield(t.builtin_trim,"success")
    fprintf("FGTrim seed success=%d",logical(t.builtin_trim.success));
    if isfield(t.builtin_trim,"error") && strlength(string(t.builtin_trim.error))>0
        fprintf(" error=%s",string(t.builtin_trim.error));
    end
    fprintf("\n");
end
fprintf("exitflag=%d  iterations=%d  funcCount=%d  resnorm=%.6g\n", ...
    t.exitflag,localField(t.output,"iterations",-1),localField(t.output,"funcCount",-1),t.resnorm);
fprintf("z=[theta %.8f elev %.8f throttle %.8f N1 %.6f N2 %.6f]\n",t.z);
fprintf("x0=[%.6f %.6f %.9f %.9f %.9f %.6f %.6f]\n",t.x);
fprintf("x1=[%.6f %.6f %.9f %.9f %.9f %.6f %.6f]\n",t.xnext);
fprintf("instantaneousAccel=[udot %.6g m/s2, wdot %.6g m/s2, qdot %.6g rad/s2]\n",t.physical_accel_residual);
fprintf("scaledAccel=[%.6g %.6g %.6g], max=%.6g\n", ...
    t.scaled_physical_accel,norm(t.scaled_physical_accel,inf));
fprintf("alphadot=%.6g rad/s scaled=%.6g  algebraicSettle=[iter %.0f err %.3g pass %d]\n", ...
    t.alphadot_residual,t.scaled_alphadot,t.diag.algebraic_settle_iterations, ...
    t.diag.algebraic_settle_error,t.algebraic_converged);
fprintf("rates=[dVa %.6g m/s2, dgamma %.6g rad/s, dtheta %.6g rad/s, dq %.6g rad/s2, dN1 %.6g/s, dN2 %.6g/s]\n",t.rate_residual);
fprintf("scaledRates=[%.6g %.6g %.6g %.6g %.6g %.6g], max=%.6g\n", ...
    t.scaled_residual,norm(t.scaled_residual,inf));
fprintf("hStep=%.9g m  mass=%.6f kg  cgX=%.6f m  Iyy=%.6f kg*m^2\n", ...
    t.h_step_m,t.diag.mass_kg,t.diag.cg_x_m,t.diag.Iyy_kgm2);
fprintf("initialTargetError=[%.3g %.3g %.3g]\n",t.initial_target_error);
fprintf("pass_solver=%d (diagnostic) pass_initial=%d pass_acceleration=%d pass_alphadot=%d pass_fixed_point=%d pass_height=%d pass_physics=%d pass=%d\n", ...
    t.pass_solver,t.pass_initial,t.pass_acceleration,t.pass_alphadot,t.pass_fixed_point,t.pass_height,t.pass_physics,t.pass);
end

function v=localField(s,name,default)
if isstruct(s) && isfield(s,name), v=s.(name); else, v=default; end
end

function localSave(report,root,name)
try
    save(fullfile(root,name),"report");
catch
end
end
