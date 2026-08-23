function [warmAbsNext,warmSlackNext]=airdropx_phys_mpc_preview_shift_warmstart(sol)
%AIRDROPX_PHYS_MPC_PREVIEW_SHIFT_WARMSTART Shift absolute planned commands one sample.
if ~isstruct(sol) || ~isfield(sol,"U_abs") || isempty(sol.U_abs) || any(~isfinite(sol.U_abs),'all')
    warmAbsNext=[]; warmSlackNext=[]; return;
end
warmAbsNext=[sol.U_abs(:,2:end),sol.U_abs(:,end)];
if isfield(sol,"slack") && ~isempty(sol.slack) && all(isfinite(sol.slack))
    s=double(sol.slack(:)); warmSlackNext=[s(2:end);s(end)];
else
    warmSlackNext=[];
end
end
