function report=airdropx_phys_finalize_runtime_v100(projectRoot,opts)
%AIRDROPX_PHYS_FINALIZE_RUNTIME_V100 Aggregate six runtime-command missions.
arguments
    projectRoot (1,1) string
    opts.OutputRoot (1,1) string
    opts.ManifestPath (1,1) string
    opts.IntervalEvidenceRoot (1,1) string = ""
end
M=readtable(opts.ManifestPath,TextType='string'); n=height(M); Found=false(n,1); Pass=false(n,1); PeakNorm=nan(n,1); PeakQ=nan(n,1); PeakH=nan(n,1); PeakVa=nan(n,1); BuildP95=nan(n,1); QpP95=nan(n,1); TotalP95=nan(n,1); PredMax=nan(n,1); Hmin=nan(n,1); Hmax=nan(n,1); Vmin=nan(n,1); Vmax=nan(n,1);
GateNames={'four_drops','mass_configuration','cargo_mass','qp_all_feasible','hard_input_bounds','finite','peak_primary','final_normalized','tail5s','realtime_solver','realtime_total','no_restart'}; G=false(n,numel(GateNames));
for i=1:n
    d=fullfile(opts.OutputRoot,M.Scenario(i)); p=fullfile(d,'runtime_command_mission.mat'); mk=fullfile(d,'scenario_complete.ok');
    if isfile(p)&&isfile(mk), S=load(p,'report'); r=S.report; Found(i)=true; Pass(i)=logical(r.pass); m=r.metrics; PeakNorm(i)=m.peak_primary_normalized; PeakQ(i)=m.peak_q_err_dps; PeakH(i)=m.peak_h_err_m; PeakVa(i)=m.peak_Va_err_mps; BuildP95(i)=m.model_build_p95_ms; QpP95(i)=m.qp_time_p95_ms; TotalP95(i)=m.total_compute_p95_ms; PredMax(i)=m.prediction_error_norm_max; Hmin(i)=m.H_cmd_min; Hmax(i)=m.H_cmd_max; Vmin(i)=m.V_cmd_min; Vmax(i)=m.V_cmd_max; for j=1:numel(GateNames), G(i,j)=logical(r.gate.(GateNames{j})); end, end
end
T=M; T.found=Found; T.pass=Pass; T.peak_norm=PeakNorm; T.peak_q_dps=PeakQ; T.peak_h_m=PeakH; T.peak_Va_mps=PeakVa; T.model_build_p95_ms=BuildP95; T.qp_p95_ms=QpP95; T.total_compute_p95_ms=TotalP95; T.prediction_error_max=PredMax; T.H_cmd_min=Hmin; T.H_cmd_max=Hmax; T.V_cmd_min=Vmin; T.V_cmd_max=Vmax; writetable(T,fullfile(opts.OutputRoot,'runtime_command_validation.csv'));
Gate=table(M.Scenario,Found,Pass,'VariableNames',{'Scenario','found','mission_pass'}); for j=1:numel(GateNames), Gate.(GateNames{j})=G(:,j); end; writetable(Gate,fullfile(opts.OutputRoot,'runtime_command_gate_matrix.csv'));
intervalPresent=false; intervalPass=false; if opts.IntervalEvidenceRoot~="", sp=fullfile(opts.IntervalEvidenceRoot,'interval_validation_summary.txt'); mp=fullfile(opts.IntervalEvidenceRoot,'interval_validation.mat'); if isfile(mp), Z=load(mp,'report'); intervalPresent=true; intervalPass=logical(Z.report.pass); elseif isfile(sp), intervalPresent=true; txt=fileread(sp); intervalPass=contains(txt,'continuous_interval_validation_pass=1'); end, end
allPass=all(Found)&all(Pass)&all(G,'all')&(~intervalPresent||intervalPass); [wn,in]=max(PeakNorm); [wq,iq]=max(PeakQ); [wh,ih]=max(PeakH); [wv,iv]=max(PeakVa); [wc,ic]=max(TotalP95); [wp,ip]=max(PredMax);
worst=struct('peak_norm',wn,'peak_norm_scenario',M.Scenario(in),'q_dps',wq,'q_scenario',M.Scenario(iq),'h_m',wh,'h_scenario',M.Scenario(ih),'Va_mps',wv,'Va_scenario',M.Scenario(iv),'total_compute_p95_ms',wc,'compute_scenario',M.Scenario(ic),'prediction_max',wp,'prediction_scenario',M.Scenario(ip));
report=struct('version','Physics-MPC v1.0.0 runtime command validation','pass',allPass,'runtime_command_validation_pass',allPass,'missions_found',sum(Found),'missions_pass',sum(Pass&Found),'missions_expected',n,'interval_reference_present',intervalPresent,'interval_reference_pass',intervalPass,'worst',worst,'rows',T,'gate_matrix',Gate,'completed_at',datetime('now'));
save(fullfile(opts.OutputRoot,'runtime_command_validation.mat'),'report','T','Gate'); localWrite(report,fullfile(opts.OutputRoot,'runtime_command_validation_summary.txt')); localPlot(T,fullfile(opts.OutputRoot,'runtime_command_validation.png'));
end
function localWrite(r,path)
fid=fopen(path,'w'); if fid<0, return; end; c=onCleanup(@()fclose(fid)); %#ok<NASGU>; fprintf(fid,'Physics-MPC v1.0.0 runtime continuous H/V command validation\n'); fprintf(fid,'H_interval_m=[20,200]\nV_interval_mps=[45,65]\nNp=100\nNc=100\nq_soft=0\ndrop_schedule_s=60 60.2 60.4 60.6\n'); fprintf(fid,'runtime_command_validation_pass=%d\nmissions_found=%d/%d\nmissions_pass=%d/%d\ninterval_reference_present=%d\ninterval_reference_pass=%d\n',r.pass,r.missions_found,r.missions_expected,r.missions_pass,r.missions_expected,r.interval_reference_present,r.interval_reference_pass); w=r.worst; fprintf(fid,'worst_peak_norm=%.9g scenario=%s\nworst_q_dps=%.9g scenario=%s\nworst_h_m=%.9g scenario=%s\nworst_Va_mps=%.9g scenario=%s\nworst_total_compute_p95_ms=%.9g scenario=%s\nworst_prediction_error_max=%.9g scenario=%s\n',w.peak_norm,w.peak_norm_scenario,w.q_dps,w.q_scenario,w.h_m,w.h_scenario,w.Va_mps,w.Va_scenario,w.total_compute_p95_ms,w.compute_scenario,w.prediction_max,w.prediction_scenario); fprintf(fid,'NOTE: validates six deterministic continuously moving command trajectories; not an analytic proof for every arbitrary H_cmd(t), Va_cmd(t).\n');
end
function localPlot(T,path)
try
    f=figure('Visible','off','Position',[100 100 1500 850]); tl=tiledlayout(f,2,2,'TileSpacing','compact','Padding','compact'); title(tl,'Physics-MPC v1.0 runtime H/V command validation'); x=categorical(T.Scenario); nexttile; bar(x,T.peak_norm); yline(1,'--'); ylabel('peak normalized'); grid on; nexttile; bar(x,T.peak_q_dps); ylabel('peak |q| deg/s'); grid on; nexttile; bar(x,T.total_compute_p95_ms); yline(100,'--'); ylabel('total compute p95 ms'); grid on; nexttile; bar(x,T.prediction_error_max); ylabel('prediction max'); grid on; exportgraphics(f,path,'Resolution',160); close(f);
catch ME, warning("AirdropX:PhysMPC:RuntimeFinalizePlot","Finalize plot failed: %s",ME.message); end
end
