function test_math_only
% Does not require JSBSim. Checks unified Q/R and horizon utilities.
[Q,R,m]=airdropx_phys_bryson_qr; %#ok<ASGLU>
assert(isequal(size(Q),[7 7]) && isequal(size(R),[2 2]));
assert(all(diag(Q)>=0) && all(diag(R)>0));
A=diag([.99 .96 .92 .90 .85 .75 .7]); B=zeros(7,2); B(1,1)=.1; B(2,2)=.2; B(3,1)=.1; B(4,1)=.15; B(5,1)=.2; B(6,2)=.1; B(7,2)=.1;
% engine states have zero Q by design, so add tiny regularization for this synthetic controllability test only.
info=airdropx_phys_autohorizon(A,B,Q+1e-8*eye(7),R);
assert(info.rho<1 && info.Np>=15);
disp("test_math_only PASS");
end
