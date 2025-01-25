function bag2avi(bag_data, output_file, topic, video_format, framerate)
arguments
    bag_data {};
    output_file {string} = 'output.avi';
    topic {string} = 'stereo/left/image_raw';
    video_format {string} = 'Motion JPEG AVI';
    framerate {int32} = 30;
end

% Isolate video stream by topic name
video_data = readMessages(select(bag_data, "Topic", topic));

% Create video IO stream
video_io = VideoWriter(output_file, video_format);
video_io.FrameRate = framerate;

% Write video to file
video_io.open()
for i = 1:length(video_data)
    img = rosReadImage(video_data{i});
    writeVideo(video_io, img)
end
video_io.close()