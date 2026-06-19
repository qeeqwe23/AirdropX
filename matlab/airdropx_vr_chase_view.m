function [position, orientation] = airdropx_vr_chase_view(pos_n_m, pos_e_m, altitude_m, heading_deg)
%AIRDROPX_VR_CHASE_VIEW Camera pose that follows the aircraft.
%
% VRML coordinates used by matlab/vr/airdropx_scene.wrl:
%   X = East, Y = Up, Z = -North.

heading = deg2rad(double(heading_deg));

aircraft_pos = [double(pos_e_m); double(altitude_m); -double(pos_n_m)];

forward = [sin(heading); 0.0; -cos(heading)];
right = [cos(heading); 0.0; sin(heading)];
back = -forward;

% Oblique chase view: behind, above, and slightly to the right. This keeps
% the aircraft, released cargo, and ground impact area in the same view.
position = aircraft_pos + 185.0 * back + 85.0 * right + [0.0; 115.0; 0.0];
target = aircraft_pos + 120.0 * forward + [0.0; -28.0; 0.0];

orientation = look_at_orientation(position, target);
end

function aa = look_at_orientation(eye, target)
forward = target - eye;
n = norm(forward);
if n < 1.0e-9
    aa = [0.0; 1.0; 0.0; 0.0];
    return;
end
forward = forward / n;

% VRML camera looks along local -Z. Build a camera-to-world matrix.
up_world = [0.0; 1.0; 0.0];
right = cross(forward, up_world);
nr = norm(right);
if nr < 1.0e-9
    right = [1.0; 0.0; 0.0];
else
    right = right / nr;
end
up = cross(right, forward);

R = [right, up, -forward];
aa = matrix_to_axis_angle(R);
end

function aa = matrix_to_axis_angle(R)
v = (trace(R) - 1.0) / 2.0;
v = max(min(v, 1.0), -1.0);
angle = acos(v);

if abs(angle) < 1.0e-8
    axis = [0.0; 1.0; 0.0];
else
    axis = [R(3,2) - R(2,3);
            R(1,3) - R(3,1);
            R(2,1) - R(1,2)] / (2.0 * sin(angle));
    n = norm(axis);
    if n > 1.0e-12
        axis = axis / n;
    else
        axis = [0.0; 1.0; 0.0];
    end
end

aa = [axis; angle];
end
