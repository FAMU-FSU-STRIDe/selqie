clc; clear; close all;

%% Load ROS2 Bag File
bag_file = "/home/shared/rosbags/2025-02-PoolTests/Rock_Standing_Farther";
topic = "stereo/left/image_raw";
bag_data = ros2bagreader(bag_file);
video_data = readMessages(select(bag_data, "Topic", topic));

%% Set Up Figure for Interactive Display
hFig = figure('Name', 'Video Frame and RGB Histogram', 'NumberTitle', 'off');

% Create subplots
hAx1 = subplot(1,2,1);
hImg = imshow(zeros(480,640,3, 'uint8')); % Initialize empty image
title('Video Frame');

hAx2 = subplot(1,2,2);
hZoom = imshow(zeros(50, 50, 3, 'uint8')); % Placeholder for zoomed-in pixel
title('Mean RGB Value');

% Enable data cursor mode
dcm = datacursormode(hFig);
set(dcm, 'Enable', 'on', 'UpdateFcn', @(obj, event) displayRGB(obj, event, hZoom));

% Storage for RGB values
global rgb_values;
rgb_values = [];

%% Interactive Frame Navigation
frameIdx = 1;
while frameIdx <= length(video_data)
    frame = rosReadImage(video_data{frameIdx});
    
    % Update Image
    set(hImg, 'CData', frame);
    
    % Wait for user input to move to next frame
    waitforbuttonpress;
    key = get(hFig, 'CurrentCharacter');
    
    if key == 'q'  % Quit if 'q' is pressed
        break;
    elseif key == 'd'  % Move forward if 'd' is pressed
        frameIdx = min(frameIdx + 1, length(video_data));
    elseif key == 'a'  % Move backward if 'a' is pressed
        frameIdx = max(frameIdx - 1, 1);
    end
end

% Compute final statistics if data exists
if ~isempty(rgb_values)
    mean_rgb = mean(rgb_values, 1);
    std_rgb = std(double(rgb_values), 0, 1);
    fprintf('Mean RGB: [%.2f, %.2f, %.2f]\n', mean_rgb(1), mean_rgb(2), mean_rgb(3));
    fprintf('Std Dev RGB: [%.2f, %.2f, %.2f]\n', std_rgb(1), std_rgb(2), std_rgb(3));
end

%% Function to Display RGB Values on Hover
function txt = displayRGB(~, event_obj, hZoom)
    global rgb_values;
    
    pos = round(event_obj.Position);
    cData = get(event_obj.Target, 'CData');
    
    % Extract RGB values at cursor position
    R = cData(pos(2), pos(1), 1);
    G = cData(pos(2), pos(1), 2);
    B = cData(pos(2), pos(1), 3);
    
    % Store the RGB value
    rgb_values = [rgb_values; R, G, B];
    
    % Compute the current mean RGB value
    mean_rgb = mean(rgb_values, 1);
    
    % Update zoomed display with mean color
    zoomedColor = uint8(ones(50, 50, 3) .* reshape(mean_rgb, 1, 1, 3));
    set(hZoom, 'CData', zoomedColor);
    
    txt = {['X: ', num2str(pos(1))], ...
           ['Y: ', num2str(pos(2))], ...
           ['R: ', num2str(R)], ...
           ['G: ', num2str(G)], ...
           ['B: ', num2str(B)], ...
           ['Mean R: ', num2str(mean_rgb(1))], ...
           ['Mean G: ', num2str(mean_rgb(2))], ...
           ['Mean B: ', num2str(mean_rgb(3))]};
end
