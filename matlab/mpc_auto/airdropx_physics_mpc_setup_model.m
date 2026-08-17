function modelPath = airdropx_physics_mpc_setup_model(varargin)
%AIRDROPX_PHYSICS_MPC_SETUP_MODEL Create the Physics-MPC closed-loop model copy.
opts=local_options(varargin{:});root=char(string(opts.ProjectRoot));if isempty(root),root=fileparts(fileparts(fileparts(mfilename('fullpath'))));end
addpath(fullfile(root,'matlab'));addpath(fullfile(root,'matlab','mpc'));addpath(fullfile(root,'matlab','mpc_auto'));addpath(fullfile(root,'matlab','sfunc_jsbsim'));
source=fullfile(root,'matlab','mpc','airdropx_mpc_closed_loop.slx');
if ~isfile(source),source=fullfile(root,'matlab','mpc_auto','airdropx_auto_mpc_closed_loop.slx');end
if ~isfile(source),error('AirdropX:PhysicsMPC:MissingModel','Missing closed-loop model template.');end
modelName=char(string(opts.ModelName));modelPath=fullfile(root,'matlab','mpc_auto',[modelName '.slx']);
copyfile(source,modelPath,'f');load_system(modelPath);c=onCleanup(@()local_close(modelName));
blk=[modelName '/MPC_Controller'];
if getSimulinkBlockHandle(blk)<0,error('AirdropX:PhysicsMPC:MissingControllerBlock','Missing block %s',blk);end
set_param(blk,'FunctionName','sfun_airdropx_physics_mpc_controller');set_param(blk,'Parameters',"''");
set_param(modelName,'InitFcn','');save_system(modelName,modelPath);set_param(modelName,'Dirty','off');
fprintf('[PHYS-MPC] closed-loop model ready: %s\n',modelPath);
end
function local_close(n),if bdIsLoaded(n),close_system(n,0);end,end
function opts=local_options(varargin)
opts.ProjectRoot="";opts.ModelName="airdropx_physics_mpc_closed_loop";
if mod(numel(varargin),2)~=0,error('Options must be name-value pairs.');end
for i=1:2:numel(varargin),n=string(varargin{i});if ~isfield(opts,n),error('Unknown option: %s',n);end,opts.(n)=varargin{i+1};end
end
