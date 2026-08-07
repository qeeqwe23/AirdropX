function [Phi, Gamma] = airdropx_mpc_prediction_matrices(A, B, horizon)
%AIRDROPX_MPC_PREDICTION_MATRICES Build stacked discrete predictions.

A = double(A);
B = double(B);
N = double(horizon);
n = size(A, 1);
m = size(B, 2);

Phi = zeros(N * n, n);
Gamma = zeros(N * n, N * m);

for row = 1:N
    rows = (row - 1) * n + (1:n);
    Phi(rows, :) = A ^ row;
    for col = 1:row
        cols = (col - 1) * m + (1:m);
        Gamma(rows, cols) = A ^ (row - col) * B;
    end
end
end
