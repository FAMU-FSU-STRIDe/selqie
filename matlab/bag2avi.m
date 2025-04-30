function bag2avi(bag_data, output_file, topic, video_format, framerate)
% BAG2AVI Converts ROS2 bag video data into an AVI file.
%
%   This function extracts image messages from a specified topic within
%   a ROS2 bag file and encodes them into a video file using the specified
%   format and frame rate.
%
%   Syntax:
%     bag2avi(bag_data)
%     bag2avi(bag_data, output_file)
%     bag2avi(bag_data, output_file, topic)
%     bag2avi(bag_data, output_file, topic, video_format, framerate)
%
%   Inputs:
%     bag_data     - ros2bagreader object or loaded bag data
%     output_file  - Output video file name (e.g., 'my_video.avi')
%     topic        - Image topic in the bag (e.g., '/camera/image_raw')
%     video_format - Format supported by VideoWriter (e.g., 'Motion JPEG AVI')
%     framerate    - Frame rate of the output video (e.g., 30)
%
%   Example:
%     bag = ros2bagreader("/home/user/bags/test_bag");
%     bag2avi(bag, "video_output.avi", "/camera/image_raw", "Motion JPEG AVI", 25);
%
%   This will save the images published on /camera/image_raw from the bag
%   into a file named 'video_output.avi' with 25 frames per second.
%

% Define input argument types and defaults using MATLAB's 'arguments' block
arguments
    bag_data {};                          % Input bag data as a cell array or ros2bagreader
    output_file {string} = 'output.avi'; % Output file name (default)
    topic {string} = 'stereo/left/image_raw'; % ROS2 image topic
    video_format {string} = 'Motion JPEG AVI'; % Video format type
    framerate {int32} = 30;              % Output video frame rate
end

% Extract messages from the specified image topic
video_data = readMessages(select(bag_data, "Topic", topic));

% Initialize video writer object with specified format and frame rate
video_io = VideoWriter(output_file, video_format);
video_io.FrameRate = framerate;

% Open the video writer to begin writing frames
video_io.open();

% Iterate through each image message and write to video file
for i = 1:length(video_data)
    img = rosReadImage(video_data{i});  % Convert ROS image message to MATLAB image
    writeVideo(video_io, img);          % Write the image to the AVI file
end

% Finalize and close the video file
video_io.close();
