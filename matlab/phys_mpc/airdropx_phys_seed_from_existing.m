function z=airdropx_phys_seed_from_existing(varargin)
%AIRDROPX_PHYS_SEED_FROM_EXISTING Seed only; never accepts old trim as truth.
% Use current proven operating values merely to initialize fsolve.
p=inputParser;
addParameter(p,"ThetaDeg",5.5,@isnumeric);
addParameter(p,"ElevatorAbs",-0.34,@isnumeric);
addParameter(p,"Throttle",0.80,@isnumeric);
addParameter(p,"N1",86,@isnumeric);
addParameter(p,"N2",92,@isnumeric);
parse(p,varargin{:});
z=[deg2rad(p.Results.ThetaDeg);p.Results.ElevatorAbs;p.Results.Throttle;p.Results.N1;p.Results.N2];
end
