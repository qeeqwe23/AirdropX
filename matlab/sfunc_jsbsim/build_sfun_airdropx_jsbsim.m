function build_sfun_airdropx_jsbsim(jsbsimRoot)
%BUILD_SFUN_AIRDROPX_JSBSIM Transactional S-Function build.
% The currently working sfun_airdropx_jsbsim.mexw64 is left untouched if
% compilation/linking fails.
thisDir=fileparts(mfilename("fullpath"));
projectRoot=fileparts(fileparts(thisDir));
if nargin<1 || strlength(string(jsbsimRoot))==0
    jsbsimRoot=fullfile(projectRoot,"third_party","jsbsim-win64");
end
src=fullfile(thisDir,"sfun_airdropx_jsbsim.cpp");
includeDir=fullfile(jsbsimRoot,"include");
jsbsimIncludeDir=fullfile(includeDir,"JSBSim");
libDir=fullfile(jsbsimRoot,"lib");
assert(isfile(src),"S-Function source not found: %s",src);
assert(isfolder(includeDir) && isfolder(jsbsimIncludeDir) && isfolder(libDir),"Bad JSBSim root: %s",jsbsimRoot);
libCandidates=[fullfile(libDir,"JSBSim.lib");fullfile(libDir,"jsbsim.lib"); ...
               fullfile(libDir,"libJSBSim.lib");fullfile(libDir,"libjsbsim.lib")];
libFile="";
for i=1:numel(libCandidates)
    if isfile(libCandidates(i)), libFile=libCandidates(i); break; end
end
assert(libFile~="","No JSBSim .lib found under %s",libDir);

tmp=tempname; mkdir(tmp); cleanup=onCleanup(@()localCleanup(tmp)); %#ok<NASGU>
base="sfun_airdropx_jsbsim";
try
    mex("-v","-R2018a","CXXFLAGS=$CXXFLAGS /std:c++17","-DJSBSIM_STATIC_LINK", ...
        "-I"+includeDir,"-I"+jsbsimIncludeDir,src,libFile,"wsock32.lib","ws2_32.lib", ...
        "-outdir",tmp,"-output",base);
catch ME
    fprintf(2,"S-Function candidate build FAILED; existing working MEX was not touched.\n");
    rethrow(ME);
end
candidate=fullfile(tmp,base+"."+mexext);
assert(isfile(candidate),"Candidate S-Function MEX was not produced.");
target=fullfile(thisDir,base+"."+mexext);
hadTarget=isfile(target); backup="";
if hadTarget
    stamp=char(datetime("now","Format","yyyyMMdd_HHmmss"));
    backup=target+".backup_"+string(stamp);
    copyfile(target,backup,"f");
    fprintf("Backed up current S-Function MEX: %s\n",backup);
end
try
    clear sfun_airdropx_jsbsim;
    copyfile(candidate,target,"f");
    rehash;
catch ME
    clear sfun_airdropx_jsbsim;
    if hadTarget && strlength(backup)>0 && isfile(backup)
        copyfile(backup,target,"f"); rehash;
        fprintf(2,"S-Function install FAILED; previous working MEX was restored from %s.\n",backup);
    elseif ~hadTarget && isfile(target)
        delete(target); rehash;
    end
    rethrow(ME);
end
fprintf("Installed S-Function MEX transactionally: %s\n",target);
end

function localCleanup(p)
if isfolder(p)
    try, rmdir(p,"s"); catch, end
end
end
