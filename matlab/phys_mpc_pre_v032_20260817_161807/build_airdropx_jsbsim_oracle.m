function build_airdropx_jsbsim_oracle(jsbsimRoot)
%BUILD_AIRDROPX_JSBSIM_ORACLE Transactional build of Physics Oracle v0.3.2.
% Performs a cheap header/API compatibility check before invoking the linker.
% The currently working MEX is never deleted before a candidate links.
thisDir=fileparts(mfilename("fullpath"));
projectRoot=fileparts(fileparts(thisDir));
if nargin<1 || strlength(string(jsbsimRoot))==0
    jsbsimRoot=fullfile(projectRoot,"third_party","jsbsim-win64");
end
includeDir=fullfile(jsbsimRoot,"include");
jsbsimIncludeDir=fullfile(includeDir,"JSBSim");
libDir=fullfile(jsbsimRoot,"lib");
assert(isfolder(includeDir) && isfolder(jsbsimIncludeDir), ...
    "JSBSim include dirs not found: %s",jsbsimRoot);
assert(isfolder(libDir),"JSBSim lib directory not found: %s",libDir);

% Report the exact compiler MATLAB will use before touching any MEX file.
try
    cc=mex.getCompilerConfigurations('C++','Selected');
catch ME
    error("AirdropX:PhysMPC:CompilerQueryFailed", ...
        "Could not query the selected MATLAB C++ MEX compiler: %s",ME.message);
end
if isempty(cc)
    error("AirdropX:PhysMPC:NoCppCompiler", ...
        "No C++ MEX compiler is selected. Run mex -setup C++ first.");
end
fprintf("MEX C++ compiler: %s | version %s | %s\n", ...
    string(cc(1).Name),string(cc(1).Version),string(cc(1).Location));

% Fail early with a precise message if the installed headers are older/different
% than the API used by v0.3.2, instead of surfacing a long compiler error later.
localRequireSymbols(fullfile(jsbsimIncludeDir,"initialization","FGInitialCondition.h"), ...
    ["InitializeIC"]);
localRequireSymbols(fullfile(jsbsimIncludeDir,"initialization","FGTrim.h"), ...
    ["SetGammaFallback","SetMaxCycles","SetMaxCyclesPerAxis","EditState","DoTrim"]);
localRequireSymbols(fullfile(jsbsimIncludeDir,"models","FGPropulsion.h"), ...
    ["GetSteadyState","InitRunning","GetNumEngines","GetNumTanks","GetTank", ...
     "SetFuelFreeze","GetFuelFreeze"]);
localRequireSymbols(fullfile(jsbsimIncludeDir,"models","propulsion","FGTank.h"), ...
    ["GetContents","SetContents","GetCapacity"]);
localRequireSymbols(fullfile(jsbsimIncludeDir,"FGFDMExec.h"), ...
    ["SetDebugLevel","GetPropulsion","GetFCS","RunIC","Run", ...
     "ResetToInitialConditions","DONT_EXECUTE_RUN_IC", ...
     "SuspendIntegration","ResumeIntegration","IntegrationSuspended","GetPropagate"]);
localRequireSymbols(fullfile(jsbsimIncludeDir,"models","FGFCS.h"), ...
    ["SetThrottleCmd","SetThrottlePos","GetThrottleCmd","GetThrottlePos","SetDeCmd","GetDeCmd", ...
     "SetPitchTrimCmd","SetRollTrimCmd","SetYawTrimCmd","SetDaCmd","SetDrCmd"]);
localRequireSymbols(fullfile(jsbsimIncludeDir,"models","FGPropagate.h"), ...
    ["InitializeDerivatives"]);

libCandidates=[fullfile(libDir,"JSBSim.lib");fullfile(libDir,"jsbsim.lib"); ...
               fullfile(libDir,"libJSBSim.lib");fullfile(libDir,"libjsbsim.lib")];
libFile="";
for i=1:numel(libCandidates)
    if isfile(libCandidates(i)), libFile=libCandidates(i); break; end
end
assert(libFile~="","JSBSim library not found under %s",libDir);
src=fullfile(thisDir,"airdropx_jsbsim_oracle_mex.cpp");
assert(isfile(src),"Oracle source not found: %s",src);

% Compile/link under a temporary output name/location. No destructive action is
% performed until a complete candidate MEX exists.
tmp=tempname; mkdir(tmp); cleanup=onCleanup(@()localCleanup(tmp)); %#ok<NASGU>
base="airdropx_jsbsim_oracle_mex";
try
    mex("-v","-R2018a","CXXFLAGS=$CXXFLAGS /std:c++17","-DJSBSIM_STATIC_LINK", ...
        "-I"+includeDir,"-I"+jsbsimIncludeDir,src,libFile,"wsock32.lib","ws2_32.lib", ...
        "-outdir",tmp,"-output",base);
catch ME
    fprintf(2,"Oracle candidate build FAILED; existing working MEX was not touched.\n");
    rethrow(ME);
end
candidate=fullfile(tmp,base+"."+mexext);
assert(isfile(candidate),"Candidate MEX was not produced: %s",candidate);
target=fullfile(thisDir,base+"."+mexext);

% Verify MATLAB can at least load the freshly linked candidate before replacing
% the working copy. Use a temporary path and the side-effect-free version call.
oldPath=path; pathCleanup=onCleanup(@()path(oldPath)); %#ok<NASGU>
oldPwd=pwd; pwdCleanup=onCleanup(@()cd(oldPwd)); %#ok<NASGU>
addpath(tmp,"-begin");
cd(tmp); % current-folder precedence now points at the candidate, never an old MEX.
clear airdropx_jsbsim_oracle_mex;
try
    v=feval(base,"version");
    assert(contains(string(v),"v0.3.2"),"Candidate Oracle returned unexpected version: %s",string(v));
catch ME
    clear airdropx_jsbsim_oracle_mex;
    fprintf(2,"Oracle candidate load/version check FAILED; existing working MEX was not touched.\n");
    rethrow(ME);
end
clear airdropx_jsbsim_oracle_mex;
cd(oldPwd);

hadTarget=isfile(target);
backup="";
if hadTarget
    stamp=char(datetime("now","Format","yyyyMMdd_HHmmss"));
    backup=target+".backup_"+string(stamp);
    copyfile(target,backup,"f");
    fprintf("Backed up current Oracle MEX: %s\n",backup);
end
try
    clear airdropx_jsbsim_oracle_mex;
    copyfile(candidate,target,"f");
    rehash;

    % Final check must resolve the installed target, not the temporary candidate.
    cd(thisDir);
    clear airdropx_jsbsim_oracle_mex;
    vInstalled=feval(base,"version");
    assert(contains(string(vInstalled),"v0.3.2"), ...
        "Installed Oracle returned unexpected version: %s",string(vInstalled));
    clear airdropx_jsbsim_oracle_mex;
    cd(oldPwd);
catch ME
    clear airdropx_jsbsim_oracle_mex;
    cd(oldPwd);
    if hadTarget && strlength(backup)>0 && isfile(backup)
        copyfile(backup,target,"f");
        rehash;
        fprintf(2,"Oracle install/load validation FAILED; previous working MEX was restored from %s.\n",backup);
    elseif ~hadTarget && isfile(target)
        delete(target);
        rehash;
        fprintf(2,"Oracle install/load validation FAILED; incomplete new target was removed.\n");
    end
    rethrow(ME);
end
fprintf("Installed and load-verified Oracle MEX transactionally: %s\n",target);
end

function localRequireSymbols(header,symbols)
assert(isfile(header),"Required JSBSim header not found: %s",header);
txt=fileread(header);
missing=strings(0,1);
for k=1:numel(symbols)
    if ~contains(txt,symbols(k))
        missing(end+1,1)=symbols(k); %#ok<AGROW>
    end
end
if ~isempty(missing)
    error("AirdropX:PhysMPC:JSBSimApiMismatch", ...
        "JSBSim header %s lacks required v0.3.2 API symbol(s): %s", ...
        header,strjoin(missing,", "));
end
end

function localCleanup(p)
if isfolder(p)
    try, rmdir(p,"s"); catch, end
end
end
