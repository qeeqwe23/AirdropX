function modelPath = airdropx_v32_setup_model(varargin)
%AIRDROPX_V32_SETUP_MODEL Create a clean v32 closed-loop SLX copy.
opts=local_options(varargin{:});
root=string(opts.ProjectRoot); if strlength(root)==0, root=string(fileparts(fileparts(fileparts(mfilename('fullpath'))))); end
mpcDir=fullfile(root,'matlab','mpc'); autoDir=fullfile(root,'matlab','mpc_auto');
addpath(fullfile(root,'matlab')); addpath(mpcDir); addpath(autoDir); addpath(fullfile(root,'matlab','sfunc_jsbsim'));
source=fullfile(mpcDir,'airdropx_mpc_closed_loop.slx'); modelPath=fullfile(autoDir,char(string(opts.ModelName)+".slx"));
if ~isfile(source)
    source=fullfile(autoDir,'airdropx_auto_mpc_closed_loop.slx');
end
if ~isfile(source), error('AirdropX:V32:MissingModel','Missing closed-loop model template under matlab/mpc or matlab/mpc_auto.'); end
copyfile(source,modelPath,'f');
load_system(modelPath); c=onCleanup(@()local_close(char(opts.ModelName)));
blk=[char(opts.ModelName) '/MPC_Controller'];
set_param(blk,'FunctionName','sfun_airdropx_v32_mpc_controller');
set_param(blk,'Parameters',"''");
set_param(char(opts.ModelName),'InitFcn','');
save_system(char(opts.ModelName),modelPath); set_param(char(opts.ModelName),'Dirty','off');
fprintf('[V32] clean closed-loop model ready: %s\n',modelPath);
end
function local_close(n), if bdIsLoaded(n), close_system(n,0); end, end
function opts=local_options(varargin)
opts.ProjectRoot=""; opts.ModelName="airdropx_v32_mpc_closed_loop";
if mod(numel(varargin),2)~=0,error('Options must be name-value pairs.');end
for i=1:2:numel(varargin),n=string(varargin{i});if ~isfield(opts,n),error('Unknown option: %s',n);end,opts.(n)=varargin{i+1};end
end
