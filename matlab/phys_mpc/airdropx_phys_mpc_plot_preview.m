function files=airdropx_phys_mpc_plot_preview(T,outDir,dropTimes,scenarioName)
%AIRDROPX_PHYS_MPC_PLOT_PREVIEW Diagnostic plots for v0.6 preview mission.
arguments
    T table
    outDir (1,1) string
    dropTimes (1,4) double
    scenarioName (1,1) string
end
files=strings(0,1);
t=T.t_s;
fig=figure('Visible','off','Color','w','Position',[100 100 1500 1050]);
tl=tiledlayout(fig,5,1,'TileSpacing','compact','Padding','compact');
vals={T.h_err_m,T.Va_err_mps,rad2deg(T.gamma_err_rad),rad2deg(T.theta_err_rad),rad2deg(T.q_err_radps)};
ylabs={'h err (m)','Va err (m/s)','gamma err (deg)','theta err (deg)','q err (deg/s)'};
for i=1:5
    ax=nexttile(tl); plot(ax,t,vals{i},'LineWidth',1.2); grid(ax,'on'); ylabel(ax,ylabs{i});
    for d=dropTimes, xline(ax,d,'--'); end
end
xlabel(nexttile(tl,5),'time (s)'); title(tl,sprintf('Physics-MPC v0.6.0 %s tracking',scenarioName),'Interpreter','none');
p=fullfile(outDir,'preview_tracking.png'); exportgraphics(fig,p,'Resolution',150); close(fig); files(end+1)=p;

fig=figure('Visible','off','Color','w','Position',[100 100 1500 800]);
tl=tiledlayout(fig,3,1,'TileSpacing','compact','Padding','compact');
ax=nexttile(tl); plot(ax,t,T.elevator_cmd,'LineWidth',1.2); hold(ax,'on'); plot(ax,t,T.elevator_trim,'--'); grid(ax,'on'); ylabel(ax,'elevator');
for d=dropTimes, xline(ax,d,'--'); end
ax=nexttile(tl); plot(ax,t,T.throttle_cmd,'LineWidth',1.2); hold(ax,'on'); plot(ax,t,T.throttle_trim,'--'); grid(ax,'on'); ylabel(ax,'throttle');
for d=dropTimes, xline(ax,d,'--'); end
ax=nexttile(tl); plot(ax,t,T.preview_minus_reactive_elevator,'LineWidth',1.2); hold(ax,'on'); plot(ax,t,T.preview_minus_reactive_throttle,'LineWidth',1.2); grid(ax,'on'); ylabel(ax,'preview-reactive'); legend(ax,{'elev','thr'},'Location','best');
for d=dropTimes, xline(ax,d,'--'); end
xlabel(ax,'time (s)'); title(tl,sprintf('Physics-MPC v0.6.0 %s controls / preview action',scenarioName),'Interpreter','none');
p=fullfile(outDir,'preview_controls.png'); exportgraphics(fig,p,'Resolution',150); close(fig); files(end+1)=p;

fig=figure('Visible','off','Color','w','Position',[100 100 1500 850]);
tl=tiledlayout(fig,3,1,'TileSpacing','compact','Padding','compact');
ax=nexttile(tl); semilogy(ax,t,max(T.pred_err_norm_inf,eps),'LineWidth',1.2); grid(ax,'on'); ylabel(ax,'pred err norm'); for d=dropTimes, xline(ax,d,'--'); end
ax=nexttile(tl); plot(ax,t,1e3*T.qp_time_s,'LineWidth',1.2); grid(ax,'on'); ylabel(ax,'QP ms'); for d=dropTimes, xline(ax,d,'--'); end
ax=nexttile(tl); yyaxis(ax,'left'); plot(ax,t,T.preview_transition_count,'LineWidth',1.2); ylabel(ax,'future cfg jumps'); yyaxis(ax,'right'); plot(ax,t,rad2deg(T.q_soft_slack_radps),'LineWidth',1.2); ylabel(ax,'q slack (deg/s)'); grid(ax,'on'); for d=dropTimes, xline(ax,d,'--'); end
xlabel(ax,'time (s)'); title(tl,sprintf('Physics-MPC v0.6.0 %s prediction / QP',scenarioName),'Interpreter','none');
p=fullfile(outDir,'preview_prediction_qp.png'); exportgraphics(fig,p,'Resolution',150); close(fig); files(end+1)=p;
end
