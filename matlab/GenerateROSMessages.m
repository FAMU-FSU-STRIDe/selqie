function GenerateROSMessages()
% Generates custom ROS 2 messages for use in MATLAB

% Define source and destination message directories
packages = {
    "actuation",      "actuation_msgs";
    "leg_control",    "leg_control_msgs";
    "mpc",            "mpc_msgs"
};

% Loop through each package and copy msg files
for i = 1:size(packages, 1)
    src_pkg = packages{i,1};
    msg_name = packages{i,2};

    src_path = fullfile("..", src_pkg, msg_name, "msg");
    dst_path = fullfile(pwd, "custom", msg_name, "msg");

    % Create destination folder if it doesn't exist
    if ~exist(dst_path, 'dir')
        mkdir(dst_path);
    end

    % Copy .msg files
    if exist(src_path, 'dir')
        copyfile(fullfile(src_path, "*.msg"), dst_path, 'f');
    else
        warning("Source folder not found: %s", src_path);
    end
end

% Generate messages
custom_path = fullfile(pwd, "custom");
if exist(custom_path, 'dir')
    ros2genmsg(custom_path);
else
    error("Custom message folder not found.");
end
