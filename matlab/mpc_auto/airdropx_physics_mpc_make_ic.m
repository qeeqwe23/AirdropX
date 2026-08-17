function icPath = airdropx_physics_mpc_make_ic(varargin)
%AIRDROPX_PHYSICS_MPC_MAKE_IC Create a per-run JSBSim IC XML without touching shared runtime IC.
opts=local_options(varargin{:});root=local_root(opts.ProjectRoot);
aircraft=char(string(opts.AircraftName));
template=fullfile(root,'aircraft',aircraft,'reset_20m.xml');
if ~isfile(template),error('AirdropX:PhysicsMPC:MissingICTemplate','IC template not found: %s',template);end
icPath=char(string(opts.OutputFile));
if isempty(icPath),error('AirdropX:PhysicsMPC:MissingICOutput','OutputFile is required.');end
if isempty(regexp(icPath,'^[A-Za-z]:[\\/]|^/|^\\\\','once')),icPath=fullfile(root,icPath);end
[p,n,e]=fileparts(icPath);if isempty(e),icPath=fullfile(p,[n '.xml']);end
if ~isfolder(fileparts(icPath)),mkdir(fileparts(icPath));end
xml=fileread(template);
alpha=double(opts.PitchDeg)-double(opts.FlightPathDeg);
u=double(opts.AirspeedMps)*cosd(alpha);w=double(opts.AirspeedMps)*sind(alpha);
xml=local_replace(xml,'<ubody\s+unit="M/SEC">[^<]*</ubody>',sprintf('<ubody unit="M/SEC">%.10g</ubody>',u),'ubody');
xml=local_replace(xml,'<wbody\s+unit="M/SEC">[^<]*</wbody>',sprintf('<wbody unit="M/SEC">%.10g</wbody>',w),'wbody');
xml=local_replace(xml,'<theta\s+unit="DEG">[^<]*</theta>',sprintf('<theta unit="DEG">%.10g</theta>',double(opts.PitchDeg)),'theta');
xml=local_replace(xml,'<altitude\s+unit="M">[^<]*</altitude>',sprintf('<altitude unit="M">%.10g</altitude>',double(opts.AltitudeM)),'altitude');
xml=local_replace(xml,'<psi\s+unit="DEG">[^<]*</psi>',sprintf('<psi unit="DEG">%.10g</psi>',double(opts.HeadingDeg)),'psi');
fid=fopen(icPath,'w','n','UTF-8');if fid<0,error('AirdropX:PhysicsMPC:ICWriteFailed','Cannot write IC: %s',icPath);end
c=onCleanup(@()fclose(fid));fprintf(fid,'%s',xml);
end
function s=local_replace(s,expr,repl,label)
if isempty(regexp(s,expr,'once')),error('AirdropX:PhysicsMPC:BadICTemplate','IC template missing %s.',label);end
s=regexprep(s,expr,repl,'once');
end
function root=local_root(x)
if strlength(string(x))>0,root=char(string(x));else,a=fileparts(mfilename('fullpath'));root=fileparts(fileparts(a));end
end
function opts=local_options(varargin)
opts.ProjectRoot="";opts.AircraftName="MQ9_Reaper";opts.OutputFile="";opts.AirspeedMps=50;opts.AltitudeM=200;opts.PitchDeg=4;opts.FlightPathDeg=0;opts.HeadingDeg=0;
if mod(numel(varargin),2)~=0,error('Options must be name-value pairs.');end
for i=1:2:numel(varargin),n=string(varargin{i});if ~isfield(opts,n),error('Unknown option: %s',n);end,opts.(n)=varargin{i+1};end
end
