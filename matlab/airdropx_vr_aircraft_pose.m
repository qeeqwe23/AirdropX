function [translation, rotation] = airdropx_vr_aircraft_pose(pos_n_m, pos_e_m, altitude_m, roll_deg, pitch_deg, heading_deg)
%AIRDROPX_VR_AIRCRAFT_POSE Convert JSBSim N/E/Up attitude to VRML pose.
%
% VRML coordinates used by matlab/vr/airdropx_scene.wrl:
%   X = East, Y = Up, Z = -North.
%
% The simple aircraft model in the scene points along its local +X axis.

translation = [double(pos_e_m); double(altitude_m); -double(pos_n_m)];

roll = deg2rad(double(roll_deg));
pitch = deg2rad(double(pitch_deg));
heading = deg2rad(double(heading_deg));

% Local +X is the aircraft nose. For heading 0 deg, nose points to -Z
% (north). For heading 90 deg, nose points to +X (east).
yaw_vr = pi/2 - heading;

R = rot_y(yaw_vr) * rot_z(pitch) * rot_x(roll);
rotation = matrix_to_axis_angle(R);
end

function R = rot_x(a)
c = cos(a);
s = sin(a);
R = [1 0 0; 0 c -s; 0 s c];
end

function R = rot_y(a)
c = cos(a);
s = sin(a);
R = [c 0 s; 0 1 0; -s 0 c];
end

function R = rot_z(a)
c = cos(a);
s = sin(a);
R = [c -s 0; s c 0; 0 0 1];
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
