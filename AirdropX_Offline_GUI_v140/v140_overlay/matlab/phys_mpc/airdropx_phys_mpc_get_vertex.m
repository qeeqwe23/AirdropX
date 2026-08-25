function [vertex,rowIndex,bankMeta] = airdropx_phys_mpc_get_vertex(bankPath,H,V,cfgId,fuelScale)
%AIRDROPX_PHYS_MPC_GET_VERTEX Load one exact certified vertex from a physics bank.
arguments
    bankPath (1,1) string
    H (1,1) double {mustBeFinite}
    V (1,1) double {mustBeFinite,mustBePositive}
    cfgId (1,1) double {mustBeInteger}
    fuelScale (1,1) double {mustBeFinite}
end
if cfgId<0 || cfgId>4, error("AirdropX:PhysMPC:BadCfg","cfgId must be 0..4."); end
if fuelScale<0 || fuelScale>1.2, error("AirdropX:PhysMPC:BadFuel","fuelScale must be in [0,1.2]."); end
if ~isfile(bankPath)
    error("AirdropX:PhysMPC:BankMissing","Physics bank not found: %s",bankPath);
end
S=load(bankPath,"vertices","rows","info");
if ~isfield(S,"vertices") || ~iscell(S.vertices) || ~isfield(S,"rows") || ~istable(S.rows)
    error("AirdropX:PhysMPC:BadBank","Bank must contain cell vertices and table rows.");
end
r=S.rows;
need={'H_m','V_mps','cfg','fuel_scale','pass'};
for k=1:numel(need)
    if ~ismember(need{k},r.Properties.VariableNames)
        error("AirdropX:PhysMPC:BadBank","Bank rows missing column %s.",need{k});
    end
end
mask=abs(r.H_m-H)<=1e-9 & abs(r.V_mps-V)<=1e-9 & r.cfg==cfgId & abs(r.fuel_scale-fuelScale)<=1e-12;
idx=find(mask);
if numel(idx)~=1
    error("AirdropX:PhysMPC:VertexNotUnique", ...
        "Expected exactly one H=%.6g V=%.6g cfg=%d fuel=%.6g vertex, found %d.",H,V,cfgId,fuelScale,numel(idx));
end
rowIndex=idx(1);
if ~logical(r.pass(rowIndex))
    error("AirdropX:PhysMPC:UncertifiedVertex","Requested bank vertex exists but pass=false.");
end
vertex=S.vertices{rowIndex};
if isempty(vertex) || ~isstruct(vertex) || ~isfield(vertex,"trim") || ~isfield(vertex,"lin") || ~isfield(vertex,"terminal")
    error("AirdropX:PhysMPC:BadVertex","Requested vertex payload is missing required fields.");
end
if ~isfield(vertex,"Q") || ~isfield(vertex,"R") || ~isfield(vertex,"qrMeta")
    error("AirdropX:PhysMPC:BadVertex","Requested vertex has no unified Q/R metadata.");
end
if ~vertex.trim.pass || ~vertex.lin.converged || ~(isfinite(vertex.terminal.rho) && vertex.terminal.rho<1)
    error("AirdropX:PhysMPC:UncertifiedVertex","Vertex internal certification gates are not all PASS.");
end
bankMeta=struct();
if isfield(S,"info"), bankMeta.info=S.info; else, bankMeta.info=struct(); end
bankMeta.row=r(rowIndex,:);
bankMeta.path=bankPath;
end
