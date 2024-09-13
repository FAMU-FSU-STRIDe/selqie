clc
clear
close all
%%

jump_data = parseROSBag("experiment_09092024_3/jump_test_0_75Hz");

% Front Left Leg Command vs Estimate

cmd_time = jump_data.legFL_command.Time;
est_time = jump_data.legFL_estimate.Time;
cmd_legZ = jump_data.legFL_command.PosSetpoint(:,3);
est_legZ = jump_data.legFL_estimate.PosEstimate(:,3);

figure
hold on
plot(cmd_time, cmd_legZ)
plot(est_time, est_legZ)
legend(["cmd", "est"])

% Power draw
info0_time = jump_data.odrive0_info.Time;
info0_current = jump_data.odrive0_info.BusCurrent;
info0_voltage = jump_data.odrive0_info.BusVoltage;
info0_power = info0_current .* info0_voltage;

figure
subplot(3, 1, 1)
plot(info0_time, info0_current)
subplot(3, 1, 2)
plot(info0_time, info0_voltage)
subplot(3, 1, 3)
plot(info0_time, info0_power)

CurrentForWires = rms(info0_current)