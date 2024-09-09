function generateMsgs()

msg_path = fullfile(pwd, "custom/robot_msgs/msg/");
% srv_path = fullfile(pwd, "custom/robot_msgs/srv/");
% action_path = fullfile(pwd, "custom/robot_msgs/action/");
copyfile("../robot_msgs/msg/", msg_path, 'f');
% copyfile("../robot_msgs/srv/", srv_path, 'f');
% copyfile("../robot_msgs/action", action_path, 'f');

folder_path = fullfile(pwd, "custom");
ros2genmsg(folder_path)