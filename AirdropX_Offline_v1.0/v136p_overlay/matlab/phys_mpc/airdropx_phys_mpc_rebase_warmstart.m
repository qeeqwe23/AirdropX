function warmNew=airdropx_phys_mpc_rebase_warmstart(warmOld,oldCtrl,newCtrl)
%AIRDROPX_PHYS_MPC_REBASE_WARMSTART Rebase only the QP initial guess across cfg.
% This does not alter the optimal control law; it only expresses the previous
% absolute-command warm start relative to the new physics trim input.
arguments
    warmOld double
    oldCtrl (1,1) struct
    newCtrl (1,1) struct
end
if oldCtrl.m~=newCtrl.m || oldCtrl.N~=newCtrl.N
    warmNew=zeros(newCtrl.m*newCtrl.N,1);
    return;
end
warmOld=double(warmOld(:));
if numel(warmOld)~=oldCtrl.m*oldCtrl.N || any(~isfinite(warmOld))
    warmNew=zeros(newCtrl.m*newCtrl.N,1);
    return;
end
oldAbs=warmOld+repmat(oldCtrl.uref,oldCtrl.N,1);
warmNew=oldAbs-repmat(newCtrl.uref,newCtrl.N,1);
warmNew=min(max(warmNew,newCtrl.lb),newCtrl.ub);
end
