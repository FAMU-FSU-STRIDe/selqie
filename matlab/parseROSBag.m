% parse_JumpTest
% Jonathan Boylan
function parsedData = parseROSBag(BAG_FILE)

% Read the ROS bag file
rosbag = ros2bagreader(strcat("../rosbags/", BAG_FILE));

% Initialize the structure to store the parsed data
parsedData = struct();

% Extract available topics
topics = rosbag.AvailableTopics;
topicNames = topics.Properties.RowNames;

% Retrieve the MessageList table (this contains timestamps)
messageList = rosbag.MessageList;

% Iterate over all topics
for i = 1:length(topicNames)
    topic = topicNames{i};
    
    % Select the messages for this topic
    msgs = readMessages(select(rosbag, 'Topic', topic));

    % Extract timestamps for the selected topic from the MessageList
    topicMessages = messageList(strcmp(cellstr(messageList.Topic), topic), :);
    timestamps = topicMessages.Time;

    % Extract the message type for this topic
    msgType = topics.MessageType(i);
    
    % Store parsed messages in a structure based on message type
    switch msgType
        case 'sensor_msgs/Imu'
            % For sensor_msgs/Imu type
            data = struct('Time', [], 'Orientation', [], 'AngularVelocity', [], 'LinearAcceleration', []);
            
            for j = 1:length(msgs)
                imu = msgs{j};
                data.Time(end+1) = timestamps(j);
                data.Orientation(end+1, :) = [imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w];
                data.AngularVelocity(end+1, :) = [imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z];
                data.LinearAcceleration(end+1, :) = [imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z];
            end
            
        case 'robot_msgs/LegCommand'
            % For robot_msgs/LegCommand type
            data = struct('Time', [], 'ControlMode', [], 'PosSetpoint', [], 'VelSetpoint', [], 'ForceSetpoint', []);
            
            for j = 1:length(msgs)
                cmd = msgs{j};
                data.Time(end+1) = timestamps(j);
                data.ControlMode(end+1) = cmd.control_mode;
                data.PosSetpoint(end+1, :) = [cmd.pos_setpoint.x, cmd.pos_setpoint.y, cmd.pos_setpoint.z];
                data.VelSetpoint(end+1, :) = [cmd.vel_setpoint.x, cmd.vel_setpoint.y, cmd.vel_setpoint.z];
                data.ForceSetpoint(end+1, :) = [cmd.force_setpoint.x, cmd.force_setpoint.y, cmd.force_setpoint.z];
            end

        case 'robot_msgs/LegEstimate'
            % For robot_msgs/LegEstimate type
            data = struct('Time', [], 'PosEstimate', [], 'VelEstimate', [], 'ForceEstimate', []);
            
            for j = 1:length(msgs)
                est = msgs{j};
                data.Time(end+1) = timestamps(j);
                data.PosEstimate(end+1, :) = [est.pos_estimate.x, est.pos_estimate.y, est.pos_estimate.z];
                data.VelEstimate(end+1, :) = [est.vel_estimate.x, est.vel_estimate.y, est.vel_estimate.z];
                data.ForceEstimate(end+1, :) = [est.force_estimate.x, est.force_estimate.y, est.force_estimate.z];
            end
            
        case 'robot_msgs/MotorInfo'
            % For robot_msgs/MotorInfo type
            data = struct('Time', [], 'AxisError', [], 'AxisState', [], 'IqSetpoint', [], 'IqMeasured', [], 'FetTemperature', [], 'MotorTemperature', [], 'BusVoltage', [], 'BusCurrent', []);
            
            for j = 1:length(msgs)
                info = msgs{j};
                data.Time(end+1) = timestamps(j);
                data.AxisError(end+1) = info.axis_error;
                data.AxisState(end+1) = info.axis_state;
                data.IqSetpoint(end+1) = info.iq_setpoint;
                data.IqMeasured(end+1) = info.iq_measured;
                data.FetTemperature(end+1) = info.fet_temperature;
                data.MotorTemperature(end+1) = info.motor_temperature;
                data.BusVoltage(end+1) = info.bus_voltage;
                data.BusCurrent(end+1) = info.bus_current;
            end
        otherwise
            % If the message type is not recognized, skip or handle accordingly
            continue;
    end
    
    % Store the parsed data in the structure under the topic name
    parsedData.(strrep(topic(2:end), '/', '_')) = data;
end

