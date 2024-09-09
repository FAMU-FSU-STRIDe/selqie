clc
clear
close all
%%

% Params
nominal_x = 0;
nominal_z = -0.18914;

loading_z = -0.10;
launching_z = -0.20;

z_load = linspace(nominal_z, loading_z, 200);
z_launch = linspace(loading_z, launching_z, 5);
z_hold = linspace(launching_z, launching_z, 100);
z_return = linspace(launching_z, nominal_z, 25);

z = [z_load, z_launch, z_hold, z_return];
x = nominal_x * ones(size(z));
y = zeros(size(z));
t = linspace(0, 1.0, size(z, 2));

pos_traj = [x; y; z];

figure
title("Z Position Trajectory")
plot(t, z)
xlabel("X (m)")
ylabel("Z (m)")
axis equal

vel_traj = zeros(size(pos_traj));
% vel_traj = [diff(pos_traj(1,:))./diff(t), 0;
%             diff(pos_traj(2,:))./diff(t), 0;
%             diff(pos_traj(3,:))./diff(t), 0];
% 
% figure
% title("Z Velocity Trajectory")
% plot(t, vel_traj(3,:))
% xlabel("Vx (m/s)")
% ylabel("Vz (m/s)")
% axis equal

force_traj = zeros(size(pos_traj));

t = floor(t * 1E3);

output = [];
for leg = 0:3

    sz = size(t);
    leg_id = leg * ones(sz);
    control_mode = 3*ones(sz);
    input_mode = 1*ones(sz);

    output = [output;
              t', leg_id', control_mode', input_mode', pos_traj', vel_traj', force_traj']

end

writematrix(output, 'jump.txt', 'Delimiter', ' ');


