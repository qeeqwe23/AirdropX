function [Phi, Gamma] = airdropx_mpc_prediction_matrices(A, B, horizon)
%AIRDROPX_MPC_PREDICTION_MATRICES Build stacked prediction matrices.

n = size(A, 1);
m = size(B, 2);
N = double(horizon);

Phi = zeros(N * n, n);
Gamma = zeros(N * n, N * m);

for i = 1:N
    Ai = A ^ i;
    Phi((i - 1) * n + (1:n), :) = Ai;
    for j = 1:i
        Aij = A ^ (i - j);
        rows = (i - 1) * n + (1:n);
        cols = (j - 1) * m + (1:m);
        Gamma(rows, cols) = Aij * B;
    end
end
end
