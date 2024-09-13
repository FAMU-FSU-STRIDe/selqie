clc
clear
close all
%%

swim_data = parseROSBag("experiment_09092024_2/swim_test_3");

% Front Left Leg Command vs Estimate

cmd_time = swim_data.legFL_command.Time;
est_time = swim_data.legFL_estimate.Time;
cmd_legZ = swim_data.legFL_command.PosSetpoint(:,3);
est_legZ = swim_data.legFL_estimate.PosEstimate(:,3);

figure
hold on
plot(cmd_time, cmd_legZ)
plot(est_time, est_legZ)
legend(["cmd", "est"])

% Power draw
info0_time = swim_data.odrive0_info.Time;
info0_current = swim_data.odrive0_info.BusCurrent;
info0_voltage = swim_data.odrive0_info.BusVoltage;
info0_power = info0_current .* info0_voltage;

figure
subplot(3, 1, 1)
plot(info0_time, info0_current)
subplot(3, 1, 2)
plot(info0_time, info0_voltage)
subplot(3, 1, 3)
plot(info0_time, info0_power)