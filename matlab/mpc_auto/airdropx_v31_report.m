function R = airdropx_v31_report(varargin)
%AIRDROPX_V31_REPORT Compact read-only progress report for v31.
opts=local_options(varargin{:});
projectRoot=string(opts.ProjectRoot); if strlength(projectRoot)==0, projectRoot=string(pwd); end
root=local_resolve(projectRoot,opts.OutputRoot); kb=local_resolve(projectRoot,opts.KnowledgeBankRoot);
R=struct(); R.output_root=root; R.knowledge_bank_root=kb;
statusFile=fullfile(root,'v31_envelope_status.csv'); curriculumFile=fullfile(root,'v31_curriculum.csv');
if isfile(statusFile), R.envelope_status=readtable(statusFile,'TextType','string'); else, R.envelope_status=table(); end
if isfile(curriculumFile), R.curriculum=readtable(curriculumFile,'TextType','string'); else, R.curriculum=table(); end
R.knowledge=airdropx_v31_knowledge_bank('Action','report','ProjectRoot',projectRoot,'Root',kb);
fprintf('\n[V31 REPORT]\n');
if ~isempty(R.envelope_status), disp(R.envelope_status); end
fprintf('KnowledgeBank: physics=%d controller=%d mission=%d state=%d\n', ...
    R.knowledge.counts.physics,R.knowledge.counts.controller,R.knowledge.counts.mission,R.knowledge.counts.state);
if ~isempty(R.curriculum)
    n=min(12,height(R.curriculum)); fprintf('\nLatest curriculum rows:\n'); disp(R.curriculum(max(1,height(R.curriculum)-n+1):end,:));
end
end
function p=local_resolve(root,p), p=string(p); c=char(p); a=startsWith(c,'/')||startsWith(c,'\\')||~isempty(regexp(c,'^[A-Za-z]:[\\/]','once')); if ~a, p=string(fullfile(root,p)); end, end
function opts=local_options(varargin)
opts=struct('ProjectRoot',"",'OutputRoot',"matlab/results/mpc_auto_v31",'KnowledgeBankRoot',"matlab/results/mpc_auto_v31_knowledge_bank");
if mod(numel(varargin),2)~=0, error('Options must be name-value pairs.'); end
for i=1:2:numel(varargin), n=string(varargin{i}); if ~isfield(opts,n), error('Unknown option: %s',n); end, opts.(n)=varargin{i+1}; end
end
