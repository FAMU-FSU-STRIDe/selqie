clc
clear
close all
%%

input_files = ["SwimFront_TestFootSpace.csv", "SwimBack_TestFootSpace.csv"];
output_file = "swim2.txt";

file_idx = [1 1 1 1];
offsets = [0 0.5 0.5 0];

for i = 1:length(file_idx)

    traj_dat = readmatrix(input_files(file_idx(i)), "Delimiter", ',');
    sz = size(traj_dat(:,1), 1);
    t = floor(linspace(0, 1E3, sz)).';

    r0 = (i - 1) * sz + 1;
    r = r0:(r0 + sz - 1);

    off = offsets(i);
    shift = [sz*off+1:sz, 1:sz*off];

    output(r, 1) = t; % time
    output(r, 2) = i - 1; % leg id
    output(r, 3) = 3; % control mode
    output(r, 4) = 1; % input mode
    output(r, 5) = traj_dat(shift, 2) * 1E-3; % x
    output(r, 6) = 0; % y
    output(r, 7) = traj_dat(shift, 3) * 1E-3; % z
    output(r, 8:10) = zeros(numel(r), 3); % v
    output(r, 11:13) = zeros(numel(r), 3); % f

end

writematrix(output, output_file, 'Delimiter', ' ');
