function report=airdropx_phys_finalize_runtime_v101(projectRoot,opts)
%AIRDROPX_PHYS_FINALIZE_RUNTIME_V101 Aggregate six dynamic-feasible runtime-command missions.
arguments
    projectRoot (1,1) string
    opts.OutputRoot (1,1) string
    opts.ManifestPath (1,1) string
    opts.IntervalEvidenceRoot (1,1) string = ""
    opts.BaselineV100Root (1,1) string = ""
end
M=readtable(opts.ManifestPath,TextType='string'); n=height(M); Found=false(n,1); Pass=false(n,1); PeakNorm=nan(n,1); PeakQ=nan(n,1); PeakGamma=nan(n,1); PeakH=nan(n,1); PeakVa=nan(n,1); TotalP95=nan(n,1); PredMax=nan(n,1); GammaRefPeak=nan(n,1); KinResidual=nan(n,1);
GateNames={'four_drops','mass_configuration','cargo_mass','qp_all_feasible','hard_input_bounds','finite','reference_kinematics','peak_primary','final_normalized','tail5s','realtime_solver','realtime_total','no_restart'}; G=false(n,numel(GateNames));
for i=1:n
    d=fullfile(opts.OutputRoot,M.Scenario(i)); p=fullfile(d,'runtime_command_mission.mat'); mk=fullfile(d,'scenario_complete.ok');
    if isfile(p)&&isfile(mk)
        S=load(p,'report'); r=S.report; Found(i)=true; Pass(i)=logical(r.pass); m=r.metrics; PeakNorm(i)=m.peak_primary_normalized; PeakQ(i)=m.peak_q_err_dps; PeakGamma(i)=m.peak_gamma_err_deg; PeakH(i)=m.peak_h_err_m; PeakVa(i)=m.peak_Va_err_mps; TotalP95(i)=m.total_compute_p95_ms; PredMax(i)=m.prediction_error_norm_max; GammaRefPeak(i)=m.gamma_ref_peak_abs_deg; KinResidual(i)=m.reference_kinematic_residual_max_mps;
        for j=1:numel(GateNames), G(i,j)=logical(r.gate.(GateNames{j})); end
    end
end
T=M; T.found=Found; T.pass=Pass; T.peak_norm=PeakNorm; T.peak_q_err_dps=PeakQ; T.peak_gamma_err_deg=PeakGamma; T.peak_h_m=PeakH; T.peak_Va_mps=PeakVa; T.gamma_ref_peak_deg=GammaRefPeak; T.total_compute_p95_ms=TotalP95; T.prediction_error_max=PredMax; T.kinematic_residual_max_mps=KinResidual; writetable(T,fullfile(opts.OutputRoot,'runtime_command_validation.csv'));
Gate=table(M.Scenario,Found,Pass,'VariableNames',{'Scenario','found','mission_pass'}); for j=1:numel(GateNames), Gate.(GateNames{j})=G(:,j); end; writetable(Gate,fullfile(opts.OutputRoot,'runtime_command_gate_matrix.csv'));
intervalPresent=false; intervalPass=false; if opts.IntervalEvidenceRoot~="", mp=fullfile(opts.IntervalEvidenceRoot,'interval_validation.mat'); sp=fullfile(opts.IntervalEvidenceRoot,'interval_validation_summary.txt'); if isfile(mp), Z=load(mp,'report'); intervalPresent=true; intervalPass=logical(Z.report.pass); elseif isfile(sp), intervalPresent=true; intervalPass=contains(fileread(sp),'continuous_interval_validation_pass=1'); end, end
allPass=all(Found)&all(Pass)&all(G,'all')&(~intervalPresent||intervalPass); [wn,in]=max(PeakNorm); [wq,iq]=max(PeakQ); [wg,ig]=max(PeakGamma); [wh,ih]=max(PeakH); [wv,iv]=max(PeakVa); [wc,ic]=max(TotalP95); [wp,ip]=max(PredMax);
worst=struct('peak_norm',wn,'peak_norm_scenario',M.Scenario(in),'q_err_dps',wq,'q_scenario',M.Scenario(iq),'gamma_err_deg',wg,'gamma_scenario',M.Scenario(ig),'h_m',wh,'h_scenario',M.Scenario(ih),'Va_mps',wv,'Va_scenario',M.Scenario(iv),'total_compute_p95_ms',wc,'compute_scenario',M.Scenario(ic),'prediction_max',wp,'prediction_scenario',M.Scenario(ip));
comparison=table(); if opts.BaselineV100Root~=""
    bp=fullfile(opts.BaselineV100Root,'runtime_command_validation.csv'); if isfile(bp)
        B=readtable(bp,TextType='string'); [tf,loc]=ismember(M.Scenario,B.Scenario); if all(tf)
            comparison=table(M.Scenario,logical(B.pass(loc)),Pass,double(B.peak_norm(loc)),PeakNorm,PeakNorm-double(B.peak_norm(loc)),double(B.peak_q_dps(loc)),PeakQ, ...
                'VariableNames',{'Scenario','v100_pass','v101_pass','v100_peak_norm','v101_peak_norm','delta_peak_norm','v100_peak_q_dps','v101_peak_q_err_dps'}); writetable(comparison,fullfile(opts.OutputRoot,'runtime_command_comparison_v100.csv'));
        end
    end
end
report=struct('version','Physics-MPC v1.0.1 dynamic-feasible runtime command validation','pass',allPass,'runtime_command_validation_pass',allPass,'missions_found',sum(Found),'missions_pass',sum(Pass&Found),'missions_expected',n,'interval_reference_present',intervalPresent,'interval_reference_pass',intervalPass,'worst',worst,'rows',T,'gate_matrix',Gate,'comparison_v100',comparison,'completed_at',datetime('now'));
save(fullfile(opts.OutputRoot,'runtime_command_validation.mat'),'report','T','Gate','comparison'); localWrite(report,fullfile(opts.OutputRoot,'runtime_command_validation_summary.txt')); localPlot(T,fullfile(opts.OutputRoot,'runtime_command_validation.png'));
end

function localWrite(r,path)
fid=fopen(path,'w'); if fid<0, return; end; c=onCleanup(@()fclose(fid)); %#ok<NASGU>; fprintf(fid,'Physics-MPC v1.0.1 dynamic-feasible runtime H/V command validation\n'); fprintf(fid,'H_interval_m=[20,200]\nV_interval_mps=[45,65]\nNp=100\nNc=100\nq_soft=0\nreference_mode=dynamic_feasible\ndrop_schedule_s=60 60.2 60.4 60.6\n'); fprintf(fid,'runtime_command_validation_pass=%d\nmissions_found=%d/%d\nmissions_pass=%d/%d\ninterval_reference_present=%d\ninterval_reference_pass=%d\n',r.pass,r.missions_found,r.missions_expected,r.missions_pass,r.missions_expected,r.interval_reference_present,r.interval_reference_pass); w=r.worst; fprintf(fid,'worst_peak_norm=%.9g scenario=%s\nworst_q_err_dps=%.9g scenario=%s\nworst_gamma_err_deg=%.9g scenario=%s\nworst_h_m=%.9g scenario=%s\nworst_Va_mps=%.9g scenario=%s\nworst_total_compute_p95_ms=%.9g scenario=%s\nworst_prediction_error_max=%.9g scenario=%s\n',w.peak_norm,w.peak_norm_scenario,w.q_err_dps,w.q_scenario,w.gamma_err_deg,w.gamma_scenario,w.h_m,w.h_scenario,w.Va_mps,w.Va_scenario,w.total_compute_p95_ms,w.compute_scenario,w.prediction_max,w.prediction_scenario); fprintf(fid,'NOTE: six deterministic continuously moving command trajectories; not an analytic proof for every arbitrary H_cmd(t), Va_cmd(t).\n');
end

function localPlot(T,path)
try
    f=figure('Visible','off','Position',[100 100 1500 900]); tl=tiledlayout(f,2,2,'TileSpacing','compact','Padding','compact'); title(tl,'Physics-MPC v1.0.1 dynamic-feasible runtime validation'); x=categorical(T.Scenario); nexttile; bar(x,T.peak_norm); yline(1,'--'); ylabel('peak normalized'); grid on; nexttile; bar(x,T.peak_gamma_err_deg); ylabel('peak |gamma error| deg'); grid on; nexttile; bar(x,T.peak_q_err_dps); ylabel('peak |q error| deg/s'); grid on; nexttile; bar(x,T.total_compute_p95_ms); yline(100,'--'); ylabel('total compute p95 ms'); grid on; exportgraphics(f,path,'Resolution',160); close(f);
catch ME, warning("AirdropX:PhysMPC:RuntimeFinalizePlot","Finalize plot failed: %s",ME.message); end
end
