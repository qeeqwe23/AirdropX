function report=airdropx_phys_math_selftest
%AIRDROPX_PHYS_MATH_SELFTEST Cheap Q/R and terminal-design gate before JSBSim work.
[Q,R,meta]=airdropx_phys_bryson_qr; %#ok<ASGLU>
if ~isequal(size(Q),[7 7]) || ~isequal(size(R),[2 2]) || ...
        any(~isfinite(Q),'all') || any(~isfinite(R),'all') || ...
        any(diag(Q)<=0) || any(diag(R)<=0)
    error("AirdropX:PhysMPC:MathSelfTestQR","Unified Q/R construction failed structural checks.");
end
A=diag([.99 .96 .92 .90 .85 .75 .70]);
B=zeros(7,2);
B(1,1)=.10; B(2,2)=.20; B(3,1)=.10; B(4,1)=.15; B(5,1)=.20; B(6,2)=.10; B(7,2)=.10;
terminal=airdropx_phys_autohorizon(A,B,Q,R);
if ~(isfinite(terminal.rho) && terminal.rho<1 && terminal.Np>=15 && terminal.Nc==terminal.Np)
    error("AirdropX:PhysMPC:MathSelfTestTerminal","Terminal LQR/automatic horizon self-test failed.");
end
report=struct("pass",true,"rho",terminal.rho,"Np",terminal.Np);
end
