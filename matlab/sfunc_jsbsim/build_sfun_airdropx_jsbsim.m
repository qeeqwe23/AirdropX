function build_sfun_airdropx_jsbsim(jsbsimRoot)
%BUILD_SFUN_AIRDROPX_JSBSIM Build the JSBSim C++ S-Function MEX file.
%
% Usage:
%   build_sfun_airdropx_jsbsim
%   build_sfun_airdropx_jsbsim("C:\path\to\jsbsim\install")
%
% Expected JSBSim layout:
%   <root>/include/FGFDMExec.h or <root>/include/JSBSim/FGFDMExec.h
%   <root>/lib/jsbsim.lib, libJSBSim.lib, or equivalent

thisDir = fileparts(mfilename("fullpath"));
projectRoot = fileparts(fileparts(thisDir));
if nargin < 1 || strlength(string(jsbsimRoot)) == 0
    jsbsimRoot = fullfile(projectRoot, "third_party", "jsbsim-win64");
end

src = fullfile(thisDir, "sfun_airdropx_jsbsim.cpp");

includeDir = fullfile(jsbsimRoot, "include");
jsbsimIncludeDir = fullfile(includeDir, "JSBSim");
libDir = fullfile(jsbsimRoot, "lib");

if ~isfolder(includeDir)
    error("JSBSim include directory not found: %s", includeDir);
end
if ~isfolder(jsbsimIncludeDir)
    error("JSBSim nested include directory not found: %s", jsbsimIncludeDir);
end
if ~isfolder(libDir)
    error("JSBSim lib directory not found: %s", libDir);
end

libCandidates = [
    fullfile(libDir, "JSBSim.lib")
    fullfile(libDir, "jsbsim.lib")
    fullfile(libDir, "libJSBSim.lib")
    fullfile(libDir, "libjsbsim.lib")
];

libFile = "";
for i = 1:numel(libCandidates)
    if isfile(libCandidates(i))
        libFile = libCandidates(i);
        break
    end
end

if libFile == ""
    error("Could not find a JSBSim .lib in %s. Edit this script to point at your JSBSim import library.", libDir);
end

mex("-v", "-R2018a", "CXXFLAGS=$CXXFLAGS /std:c++17", ...
    "-DJSBSIM_STATIC_LINK", ...
    "-I" + includeDir, "-I" + jsbsimIncludeDir, ...
    src, libFile, "wsock32.lib", "ws2_32.lib", "-outdir", thisDir);

fprintf("Built %s\n", fullfile(thisDir, "sfun_airdropx_jsbsim." + mexext));
end
