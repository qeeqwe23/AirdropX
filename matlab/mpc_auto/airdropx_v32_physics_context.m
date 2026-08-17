function ctx = airdropx_v32_physics_context(projectRoot, contextReferenceMassKg)
%AIRDROPX_V32_PHYSICS_CONTEXT Describe the physics behind v32 context metadata.
%
% The existing precompiled JSBSim MEX exposes a 20-D context mass that is
% intentionally dry aircraft + remaining payload. JSBSim dynamics, however,
% also include XML tank contents. v32.1.4 keeps the runtime 20-D interface
% unchanged (no MEX rebuild required), but records both meanings explicitly
% and fingerprints the aircraft XML tree so stale Physics/MPC certificates
% cannot silently survive an aircraft-model change.

projectRoot = char(string(projectRoot));
contextReferenceMassKg = double(contextReferenceMassKg);
aircraftDir = fullfile(projectRoot,'aircraft','MQ9_Reaper');
mainXml = fullfile(aircraftDir,'MQ9_Reaper.xml');
ctx = struct();
ctx.schema_version = "v32.1.4_physics_context";
ctx.context_mass_semantics = "dry_plus_remaining_payload_excludes_fuel";
ctx.context_reference_mass_kg = contextReferenceMassKg;
ctx.initial_fuel_mass_kg = NaN;
ctx.estimated_actual_reference_mass_kg = NaN;
ctx.aircraft_xml_path = string(mainXml);
ctx.aircraft_fingerprint = "missing_aircraft_xml";
ctx.fuel_source = "unavailable";

if isfile(mainXml)
    try
        txt = fileread(mainXml);
        tankBlocks = regexp(txt,'<tank\b[\s\S]*?</tank>','match');
        fuelLbs = 0.0;
        nFuel = 0;
        for k = 1:numel(tankBlocks)
            tok = regexp(tankBlocks{k},'<contents\b[^>]*>\s*([-+0-9.eE]+)\s*</contents>','tokens','once');
            if ~isempty(tok)
                x = str2double(tok{1});
                if isfinite(x), fuelLbs = fuelLbs + x; nFuel = nFuel + 1; end
            end
        end
        if nFuel > 0
            ctx.initial_fuel_mass_kg = fuelLbs * 0.45359237;
            ctx.estimated_actual_reference_mass_kg = contextReferenceMassKg + ctx.initial_fuel_mass_kg;
            ctx.fuel_source = "aircraft_xml_initial_tank_contents";
        end
    catch ME
        warning('AirdropX:V32:PhysicsContextFuel','Could not parse initial fuel mass: %s',ME.message);
    end
end

% Fingerprint every XML file under the active aircraft tree, including
% Controls/engine-linked local XML. This is intentionally implemented without
% external hashing/toolboxes so it works on the user's current MATLAB setup.
try
    D = dir(fullfile(aircraftDir,'**','*.xml'));
    keys=strings(numel(D),1);
    for kk=1:numel(D),keys(kk)=lower(string(fullfile(D(kk).folder,D(kk).name)));end
    [~,ord] = sort(keys); D = D(ord);
    mod1 = 2147483629; mod2 = 2147483587;
    h1 = 17; h2 = 29; totalBytes = 0;
    for k = 1:numel(D)
        p = fullfile(D(k).folder,D(k).name);
        fid = fopen(p,'rb');
        if fid < 0, continue; end
        b = fread(fid,Inf,'*uint8'); fclose(fid);
        totalBytes = totalBytes + numel(b);
        rel = strrep(p,[aircraftDir filesep],'');
        rb = uint8(unicode2native(rel,'UTF-8'));
        z = double([rb(:); uint8(0); b(:)]);
        if isempty(z), continue; end
        idx = (1:numel(z)).';
        h1 = mod(h1 + sum(mod(z .* mod(idx,65521),mod1)),mod1);
        h2 = mod(h2 + sum(mod((z+1) .* mod(idx.*idx,65519),mod2)),mod2);
    end
    ctx.aircraft_fingerprint = string(sprintf('xmltree_n%d_b%d_h%010u_%010u',numel(D),totalBytes,round(h1),round(h2)));
catch ME
    warning('AirdropX:V32:PhysicsFingerprint','Could not fingerprint aircraft XML tree: %s',ME.message);
end
end
