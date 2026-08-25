function dx=airdropx_phys_mpc_state_error(x,xref)
%AIRDROPX_PHYS_MPC_STATE_ERROR State error with angular wrapping for gamma/theta.
x=double(x(:)); xref=double(xref(:));
if numel(x)~=7 || numel(xref)~=7
    error("AirdropX:PhysMPC:BadState","Expected seven-state vectors.");
end
dx=x-xref;
dx(3)=atan2(sin(dx(3)),cos(dx(3)));
dx(4)=atan2(sin(dx(4)),cos(dx(4)));
end
