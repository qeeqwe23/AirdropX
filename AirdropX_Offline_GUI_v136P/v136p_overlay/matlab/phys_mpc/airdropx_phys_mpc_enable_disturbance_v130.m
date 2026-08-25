function ctrl=airdropx_phys_mpc_enable_disturbance_v130(ctrl,Gw)
%AIRDROPX_PHYS_MPC_ENABLE_DISTURBANCE_V130 Add generic additive-disturbance prediction.
%
% Base delta model:
%   dx(k+1) = A dx(k) + B du(k)
% Augmented prediction used by v1.3.0:
%   dx(k+1) = A dx(k) + B du(k) + g(k)
% where g(k)=Gw*dWind(k)+dResidual(k).
%
% The Hessian and hard input constraints are unchanged.  Only the QP linear
% term changes with the measured/forecast disturbance sequence.
arguments
    ctrl (1,1) struct
    Gw (:,1) double {mustBeFinite}
end
Gw=double(Gw(:));
if numel(Gw)~=ctrl.n
    error("AirdropX:WindMPC:BadGw","Gw must have %d elements.",ctrl.n);
end
n=ctrl.n; N=ctrl.N;
Gdist=zeros(n*N,n*N);
for i=1:N
    rows=(i-1)*n+(1:n);
    for j=1:i
        cols=(j-1)*n+(1:n);
        Gdist(rows,cols)=ctrl.A^(i-j);
    end
end
ctrl.Gw=Gw;
ctrl.Gdist=Gdist;
ctrl.Fdist=2*(ctrl.Gamma'*ctrl.Qbar*Gdist);
ctrl.wind_disturbance_model="delta-wind measured disturbance + decaying residual";
end
