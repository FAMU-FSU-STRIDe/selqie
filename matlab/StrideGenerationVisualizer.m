% Jonathan Boylan
clc; clear; close all;

%% Params

L1 = 0.066;
L2 = 0.150;
body_height = 0.18;
k = linspace(0,1,60);

point_style = '.';
point_size = 20;
point_width = 1.5;
line_width = 2;
handle_off = {'HandleVisibility', 'off'};

%% Plot

figure('Color', [1 1 1])
hold on
grid on
axis equal
legend('Interpreter', 'latex', 'FontSize', 14)
xlim([-0.25 0.25])
ylim([-0.25 0.05])
xlabel('X [m]', 'Interpreter', 'latex', 'FontSize', 18)
ylabel('Z [m]', 'Interpreter', 'latex', 'FontSize', 18)
% title('Gait Strides in Leg Space', 'Interpreter', 'latex', 'FontSize', 18)
set(gca, 'FontSize', 14)
set(gca, 'FontName', 'georgia')

%% Bounds

Rmax = L1 + L2;
Rmin = L2 - L1;
Rmax_x = Rmax * cos(2 * pi * k);
Rmax_z = Rmax * sin(2 * pi * k);
Rmin_x = Rmin * cos(2 * pi * k);
Rmin_z = Rmin * sin(2 * pi * k);

Rinf = 0.50;
Rinf_x = Rinf * cos(2 * pi * k);
Rinf_z = Rinf * sin(2 * pi * k);

plot(Rmin_x, Rmin_z, '--r', 'LineWidth', 1, handle_off{:})
plot(Rmax_x, Rmax_z, '--r', 'LineWidth', 1, handle_off{:})

fill([Rmin_x flip(Rmax_x)], [Rmin_z flip(Rmax_z)], 'g', 'FaceAlpha', 0.05, 'EdgeColor','none', handle_off{:})
fill(Rmin_x, Rmin_z, 'r', 'FaceAlpha', 0.05, 'EdgeColor','none', handle_off{:})
fill([Rmax_x flip(Rinf_x)], [Rmax_z flip(Rinf_z)], 'r', 'FaceAlpha', 0.05, 'EdgeColor','none', handle_off{:})

%% Walking

delta = -0.25;
step_height = 0.025;
stance_length = 0.175;
k2 = k(1:2:end);

stance_x = ((delta + 1)/2 - k2) * stance_length;
stance_z = -body_height * ones(size(k2));
swing_x = (delta - cos(pi*k2)) * stance_length / 2;
swing_z = -body_height + step_height * sin(pi * k2);

forest_green = [46 111 64] / 255;
plot(stance_x, stance_z, '-', 'LineWidth', line_width, 'Color', forest_green, handle_off{:})
plot(stance_x, stance_z, point_style, 'LineWidth', point_width, 'MarkerSize', point_size, 'Color', forest_green, 'DisplayName', 'Walk Stride Points')
plot(swing_x, swing_z, '-', 'LineWidth', line_width, 'Color', forest_green, handle_off{:})
plot(swing_x, swing_z, point_style, 'LineWidth', point_width, 'MarkerSize', point_size, 'Color', forest_green, handle_off{:})

%% Jumping

z_crouch = -0.095;
d_jump = -0.210;
jump_vx = 0.1;
jump_vz = 1;
jump_v = norm([jump_vx jump_vz]);
k4 = k(1:4:end);

crouch_x = zeros(size(k4));
crouch_z = -body_height + (z_crouch + body_height) * k4;
jump_x = [crouch_x(end), d_jump * jump_vx / jump_v];
jump_z = [crouch_z(end), d_jump * jump_vz / jump_v];

neon_orange = [255 92 0] / 255;
plot(crouch_x, crouch_z, '-', 'LineWidth', line_width, 'Color', neon_orange, handle_off{:});
plot(crouch_x, crouch_z, point_style, 'LineWidth', point_width, 'MarkerSize', point_size, 'Color', neon_orange, 'DisplayName', 'Jump Stride Points');
plot(jump_x, jump_z, '-', 'LineWidth', line_width, 'Color', neon_orange, handle_off{:});
plot(jump_x, jump_z, point_style, 'LineWidth', point_width, 'MarkerSize', point_size, 'Color', neon_orange, handle_off{:});

%% Swimming

leg_length = 0.18;
z_amp = 0.005;
alpha_x = 5;
swim_vx = 0.25;
swim_vz = 0.15;
swim_v = norm([swim_vx swim_vz]);

x_amp = swim_v / alpha_x;
phi = -2*atan2(swim_vz - swim_v, swim_vx);

swim_xp = x_amp * cos(2 * pi * k);
swim_zp = z_amp * sin(2 * pi * k) - leg_length;
swim_x = swim_xp * cos(phi) + swim_zp * sin(phi);
swim_z = swim_zp * cos(phi) - swim_xp * sin(phi);

royal_blue = [48 92 222] / 255;
plot(swim_x, swim_z, '-', 'LineWidth', line_width, 'Color', royal_blue, handle_off{:})
plot(swim_x, swim_z, point_style, 'LineWidth', point_width, 'MarkerSize', point_size, 'Color', royal_blue, 'DisplayName', 'Swim Stride Points')

%% Default

plot(0,0,'.k','MarkerSize', 50, 'DisplayName', 'Leg Origin Point')
plot(0, -body_height, '.r', 'LineWidth', 5, 'MarkerSize', 50, 'DisplayName', 'Default Leg Position');