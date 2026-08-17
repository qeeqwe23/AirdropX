function paths=airdropx_phys_mpc_plot_four_drop(T,eventMetrics,outputRoot,dropTimes)
%AIRDROPX_PHYS_MPC_PLOT_FOUR_DROP Mission plots for four payload releases.
arguments
    T table
    eventMetrics table
    outputRoot (1,1) string
    dropTimes (1,4) double
end
if ~isfolder(outputRoot), mkdir(outputRoot); end
paths=strings(3,1);

f=figure('Visible','off','Color','w','Position',[80 80 1500 1000]);
tl=tiledlayout(f,3,2,'TileSpacing','compact','Padding','compact');
nexttile; plot(T.t_s,T.h_err_m,'LineWidth',1.4); ylabel('h error (m)'); grid on; localDrops(dropTimes);
nexttile; plot(T.t_s,T.Va_err_mps,'LineWidth',1.4); ylabel('Va error (m/s)'); grid on; localDrops(dropTimes);
nexttile; plot(T.t_s,rad2deg(T.gamma_err_rad),'LineWidth',1.4); ylabel('gamma error (deg)'); grid on; localDrops(dropTimes);
nexttile; plot(T.t_s,rad2deg(T.theta_err_rad),'LineWidth',1.4); ylabel('theta error (deg)'); grid on; localDrops(dropTimes);
nexttile; plot(T.t_s,rad2deg(T.q_err_radps),'LineWidth',1.4); ylabel('q error (deg/s)'); xlabel('t (s)'); grid on; localDrops(dropTimes);
nexttile; stairs(T.t_s,T.cfg,'LineWidth',1.4); ylabel('cfg / drop count'); xlabel('t (s)'); ylim([-0.2 4.2]); yticks(0:4); grid on; localDrops(dropTimes);
title(tl,'Physics-MPC v0.5.2 configurable payload-release tracking');
paths(1)=fullfile(outputRoot,'four_drop_tracking.png'); exportgraphics(f,paths(1),'Resolution',160); close(f);

f=figure('Visible','off','Color','w','Position',[80 80 1500 900]);
tl=tiledlayout(f,3,1,'TileSpacing','compact','Padding','compact');
nexttile; plot(T.t_s,T.elevator_cmd,'LineWidth',1.4); hold on; plot(T.t_s,T.elevator_trim,'--','LineWidth',1.0); ylabel('elevator norm'); grid on; localDrops(dropTimes); legend('cmd','scheduled trim','Location','best');
nexttile; plot(T.t_s,T.throttle_cmd,'LineWidth',1.4); hold on; plot(T.t_s,T.throttle_trim,'--','LineWidth',1.0); ylabel('throttle'); grid on; localDrops(dropTimes); legend('cmd','scheduled trim','Location','best');
nexttile; yyaxis left; plot(T.t_s,T.mass_kg,'LineWidth',1.5); ylabel('mass (kg)'); yyaxis right; plot(T.t_s,T.cg_x_m,'LineWidth',1.2); ylabel('CG x (m)'); xlabel('t (s)'); grid on; localDrops(dropTimes);
title(tl,'Commands and real JSBSim mass/CG transitions');
paths(2)=fullfile(outputRoot,'four_drop_controls_mass.png'); exportgraphics(f,paths(2),'Resolution',160); close(f);

f=figure('Visible','off','Color','w','Position',[80 80 1500 900]);
tl=tiledlayout(f,3,1,'TileSpacing','compact','Padding','compact');
nexttile; semilogy(T.t_s,max(T.pred_err_norm_inf,1e-12),'LineWidth',1.3); ylabel('1-step pred. err / scale'); grid on; localDrops(dropTimes);
nexttile; plot(T.t_s,1e3*T.qp_time_s,'LineWidth',1.2); ylabel('QP solve (ms)'); grid on; localDrops(dropTimes);
nexttile; bar(eventMetrics.event_group,eventMetrics.peak_primary_normalized); xlabel('release event group'); ylabel('non-overlap post-event peak normalized error'); grid on;
title(tl,'Prediction, realtime QP, and post-drop peaks');
paths(3)=fullfile(outputRoot,'four_drop_prediction_qp.png'); exportgraphics(f,paths(3),'Resolution',160); close(f);
end

function localDrops(dropTimes)
u=unique(dropTimes,'stable');
for j=1:numel(u)
    idx=find(abs(dropTimes-u(j))<1e-12);
    n=numel(idx);
    if n==1, label=sprintf('drop%d',idx(1)); else, label=sprintf('drop x%d',n); end
    xline(u(j),'--',label,'LabelVerticalAlignment','bottom');
end
end
