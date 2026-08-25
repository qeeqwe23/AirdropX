function ctrl = airdropx_phys_mpc_condense(vertex,opts)
%AIRDROPX_PHYS_MPC_CONDENSE Build a dense, box-constrained finite-horizon QP.
%
% Delta model around a certified trim:
%   dx(k+1)=A dx(k)+B du(k)
% Cost:
%   sum_{k=0}^{N-1} dx'Q dx + du'R du + dx(N)'P dx(N)
% Decision vector U=[du(0);...;du(N-1)].
arguments
    vertex (1,1) struct
    opts.Horizon (1,1) double = NaN
    opts.ElevatorBounds (1,2) double = [-1 1]
    opts.ThrottleBounds (1,2) double = [0 1]
end
A=double(vertex.lin.Ad); B=double(vertex.lin.Bd);
Q=double(vertex.Q); R=double(vertex.R); P=double(vertex.terminal.P);
xref=double(vertex.trim.x(:)); uref=double(vertex.trim.u(:));
n=size(A,1); m=size(B,2);
if n~=7 || m~=2
    error("AirdropX:PhysMPC:BadDimensions","Expected certified 7-state/2-input vertex, got n=%d m=%d.",n,m);
end
if ~isequal(size(A),[n n]) || ~isequal(size(B),[n m]) || ...
        ~isequal(size(Q),[n n]) || ~isequal(size(R),[m m]) || ~isequal(size(P),[n n])
    error("AirdropX:PhysMPC:BadDimensions","A/B/Q/R/P dimensions are inconsistent.");
end
if any(~isfinite(A),'all') || any(~isfinite(B),'all') || any(~isfinite(Q),'all') || any(~isfinite(R),'all') || any(~isfinite(P),'all')
    error("AirdropX:PhysMPC:NonfiniteModel","Certified vertex matrices contain non-finite entries.");
end
if ~isfield(vertex.qrMeta,"StateScale") || ~isfield(vertex.qrMeta,"InputScale")
    error("AirdropX:PhysMPC:BadVertex","qrMeta must contain StateScale and InputScale.");
end
if isnan(opts.Horizon)
    N=double(vertex.terminal.Np);
else
    N=double(opts.Horizon);
    if ~(isfinite(N) && N==round(N) && N>=1)
        error("AirdropX:PhysMPC:BadHorizon","Horizon must be NaN (use certified Np) or a positive integer.");
    end
end

Phi=zeros(n*N,n);
Gamma=zeros(n*N,m*N);
Apow=eye(n);
for i=1:N
    Apow=A*Apow;
    rows=(i-1)*n+(1:n);
    Phi(rows,:)=Apow;
    for j=1:i
        cols=(j-1)*m+(1:m);
        Gamma(rows,cols)=A^(i-j)*B;
    end
end
Qbar=zeros(n*N,n*N);
for i=1:N-1
    rows=(i-1)*n+(1:n);
    Qbar(rows,rows)=Q;
end
rows=(N-1)*n+(1:n); Qbar(rows,rows)=P;
Rbar=kron(eye(N),R);
Hqp=2*(Gamma'*Qbar*Gamma+Rbar);
Hqp=(Hqp+Hqp')/2;
Fx=2*(Gamma'*Qbar*Phi); % f = Fx*dx

umin=[opts.ElevatorBounds(1);opts.ThrottleBounds(1)];
umax=[opts.ElevatorBounds(2);opts.ThrottleBounds(2)];
if ~(umin(1)<umax(1) && umin(2)<umax(2))
    error("AirdropX:PhysMPC:BadInputBounds","Input bounds are invalid.");
end
if any(uref<=umin) || any(uref>=umax)
    error("AirdropX:PhysMPC:TrimOnInputBound","Trim input must be strictly inside hard command bounds.");
end
lb=repmat(umin-uref,N,1);
ub=repmat(umax-uref,N,1);

% Positive definiteness is expected because R>0. Refuse a malformed QP early.
[~,cholFlag]=chol(Hqp);
if cholFlag~=0
    error("AirdropX:PhysMPC:NonConvexQP","Condensed Hessian is not positive definite.");
end

qpopts=optimoptions("quadprog", ...
    "Algorithm","active-set", ...
    "Display","off", ...
    "ConstraintTolerance",1e-8, ...
    "OptimalityTolerance",1e-8, ...
    "StepTolerance",1e-9, ...
    "MaxIterations",max(1000,20*m*N));

ctrl=struct();
ctrl.A=A; ctrl.B=B; ctrl.Q=Q; ctrl.R=R; ctrl.P=P;
ctrl.xref=xref; ctrl.uref=uref; ctrl.N=N; ctrl.n=n; ctrl.m=m;
ctrl.Phi=Phi; ctrl.Gamma=Gamma; ctrl.Qbar=Qbar; ctrl.Rbar=Rbar;
ctrl.H=Hqp; ctrl.Fx=Fx; ctrl.lb=lb; ctrl.ub=ub;
ctrl.umin=umin; ctrl.umax=umax; ctrl.options=qpopts;
ctrl.K=double(vertex.terminal.K); ctrl.rho=double(vertex.terminal.rho);
if ~isequal(size(ctrl.K),[m n]), error("AirdropX:PhysMPC:BadTerminalGain","Terminal K must be 2x7."); end
ctrl.stateScale=double(vertex.qrMeta.StateScale(:));
ctrl.inputScale=double(vertex.qrMeta.InputScale(:));
if numel(ctrl.stateScale)~=n || numel(ctrl.inputScale)~=m || any(ctrl.stateScale<=0) || any(ctrl.inputScale<=0)
    error("AirdropX:PhysMPC:BadScales","Bryson state/input scales are invalid.");
end
end
