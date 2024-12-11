clc; clear; close all
%%

timing_file = "/tmp/cla_final_experiment_timing.txt";
error_file = "/tmp/cla_final_experiment_error.txt";

timing = readmatrix(timing_file);
error = readmatrix(error_file);

timing = smoothdata(timing, 1, "movmean", 100);
error = smoothdata(error, 1, "movmean", 100);

timing_mean = mean(timing, 1)
error_mean = mean(error, 1)

figure('Color', [1 1 1])
hold on
grid on
plot(timing(:,1), 'b-', 'DisplayName', 'Householder QR', 'LineWidth', 3)
plot(timing(:,2), 'r-', 'DisplayName', 'SVD', 'LineWidth', 3)
plot(timing(:,3), 'g-', 'DisplayName', 'Normal Equations', 'LineWidth', 3)
xlabel("Test #")
ylabel("Timing (us)")
legend

figure()
hold on
plot(error(:,1), 'b-', 'DisplayName', 'Householder QR', 'LineWidth', 3)
plot(error(:,2), 'r-', 'DisplayName', 'SVD', 'LineWidth', 3)
plot(error(:,3), 'g-', 'DisplayName', 'Normal Equations', 'LineWidth', 3)
xlabel("Test #")
ylabel("Squared Norm Error")
legend