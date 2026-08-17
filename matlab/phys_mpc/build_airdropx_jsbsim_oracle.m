function build_airdropx_jsbsim_oracle(jsbsimRoot)
%BUILD_AIRDROPX_JSBSIM_ORACLE Build the discrete JSBSim physics-oracle MEX.
thisDir = fileparts(mfilename("fullpath"));
projectRoot = fileparts(fileparts(thisDir));
if nargin < 1 || strlength(string(jsbsimRoot)) == 0
    jsbsimRoot = fullfile(projectRoot,"third_party","jsbsim-win64");
end
includeDir = fullfile(jsbsimRoot,"include");
jsbsimIncludeDir = fullfile(includeDir,"JSBSim");
libDir = fullfile(jsbsimRoot,"lib");
libCandidates = [fullfile(libDir,"JSBSim.lib"); fullfile(libDir,"jsbsim.lib"); ...
                 fullfile(libDir,"libJSBSim.lib"); fullfile(libDir,"libjsbsim.lib")];
libFile="";
for i=1:numel(libCandidates)
    if isfile(libCandidates(i)), libFile=libCandidates(i); break; end
end
assert(isfolder(includeDir) && isfolder(jsbsimIncludeDir),"JSBSim include dirs not found: %s",jsbsimRoot);
assert(libFile~="","JSBSim import library not found under %s",libDir);
src=fullfile(thisDir,"airdropx_jsbsim_oracle_mex.cpp");
mex("-v","-R2018a","CXXFLAGS=$CXXFLAGS /std:c++17","-DJSBSIM_STATIC_LINK", ...
    "-I"+includeDir,"-I"+jsbsimIncludeDir,src,libFile,"wsock32.lib","ws2_32.lib", ...
    "-outdir",thisDir);
fprintf("Built %s\n",fullfile(thisDir,"airdropx_jsbsim_oracle_mex."+mexext));
end
