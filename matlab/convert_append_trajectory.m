clc
clear
close all
%%

input_files = ["SwimFront_TestFootSpace.csv", "SwimBack_TestFootSpace.csv"];
output_file = "swim.txt";

file_idx = [1 2 2 1];

for i = 1:length(file_idx)

    traj_dat = readmatrix(input_files(file_idx(i)), "Delimiter", ',');
    sz = size(traj_dat(:,1), 1);
    t = floor(linspace(0, 1E3, sz)).';

    r0 = (i - 1) * sz + 1;
    r = r0:(r0 + sz - 1);

    output(r, 1) = t; % time
    output(r, 2) = i - 1; % leg id
    output(r, 3) = 3; % control mode
    output(r, 4) = 1; % input mode
    output(r, 5) = traj_dat(:, 2) * 1E-3; % x
    % skip y = 0
    output(r, 7) = traj_dat(:, 3) * 1E-3; % z
    % skip vx, vy, vz = 0
    % skip fx, fy, fz = 0

end

writematrix(output, output_file, 'Delimiter', ' ');
