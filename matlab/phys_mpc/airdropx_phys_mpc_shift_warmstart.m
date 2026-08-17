function Ushift=airdropx_phys_mpc_shift_warmstart(U,m,N)
%AIRDROPX_PHYS_MPC_SHIFT_WARMSTART Shift the previous optimal sequence by one sample.
U=double(U(:));
if numel(U)~=m*N
    error("AirdropX:PhysMPC:BadWarmStart","U has the wrong size.");
end
Um=reshape(U,m,N);
Um=[Um(:,2:end),Um(:,end)];
Ushift=Um(:);
end
