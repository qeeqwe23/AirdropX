function files=airdropx_phys_mpc_plot_closed_loop(T,outDir)
%AIRDROPX_PHYS_MPC_PLOT_CLOSED_LOOP Save compact closed-loop diagnostic plots.
arguments
    T table
    outDir (1,1) string
end
if ~isfolder(outDir), mkdir(outDir); end
files=strings(0,1);
fig=figure("Visible","off","Color","w","Position",[100 100 1200 760]);
tl=tiledlayout(fig,2,2,"TileSpacing","compact","Padding","compact");
nexttile(tl); plot(T.t_s,T.h_err_m,"LineWidth",1.2); grid on; ylabel("h error (m)"); xlabel("t (s)");
nexttile(tl); plot(T.t_s,T.Va_err_mps,"LineWidth",1.2); grid on; ylabel("Va error (m/s)"); xlabel("t (s)");
nexttile(tl); plot(T.t_s,rad2deg(T.gamma_err_rad),"LineWidth",1.2); grid on; ylabel("gamma error (deg)"); xlabel("t (s)");
nexttile(tl); plot(T.t_s,rad2deg(T.q_err_radps),"LineWidth",1.2); grid on; ylabel("q error (deg/s)"); xlabel("t (s)");
f=fullfile(outDir,"tracking_errors.png"); exportgraphics(fig,f,"Resolution",150); close(fig); files(end+1)=f;

fig=figure("Visible","off","Color","w","Position",[100 100 1200 520]);
tl=tiledlayout(fig,2,1,"TileSpacing","compact","Padding","compact");
nexttile(tl); plot(T.t_s,T.elevator_cmd,"LineWidth",1.2); grid on; ylabel("elevator norm"); xlabel("t (s)");
nexttile(tl); plot(T.t_s,T.throttle_cmd,"LineWidth",1.2); grid on; ylabel("throttle"); xlabel("t (s)");
f=fullfile(outDir,"control_commands.png"); exportgraphics(fig,f,"Resolution",150); close(fig); files(end+1)=f;

fig=figure("Visible","off","Color","w","Position",[100 100 1200 520]);
tl=tiledlayout(fig,2,1,"TileSpacing","compact","Padding","compact");
nexttile(tl); semilogy(T.t_s,max(T.pred_err_norm_inf,eps),"LineWidth",1.2); grid on; ylabel("1-step pred. err / scale"); xlabel("t (s)");
nexttile(tl); plot(T.t_s,1e3*T.qp_time_s,"LineWidth",1.2); grid on; ylabel("QP solve (ms)"); xlabel("t (s)");
f=fullfile(outDir,"prediction_and_qp.png"); exportgraphics(fig,f,"Resolution",150); close(fig); files(end+1)=f;
end
