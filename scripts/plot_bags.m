% Initialize ROS node
ip_address = '192.168.0.3';
d = rosdevice(ip_address, 'andrew', 'MynewPCisballin');

% Subscribe to bag topics
% sub_octomap = rossubscriber('X4/octomap_full');
sub_odometry = rossubscriber('X4/odometry');
sub_path = rossubscriber('X4/planned_path');
sub_edt = rossubscriber('X4/edt');
voxel_size = 0.2;

% Get map statistics
position_history = zeros(0,4);
map_size_history = [];
t_map = [];
t_odometry_prev = 0.0;
t_map_prev = 0.0;

while d.isCoreRunning()
    % Parse odometry data
    msg_odometry = receive(sub_odometry, 0.1);
    p = msg_odometry.pose.pose.position;
    q = msg_odometry.pose.pose.orientation;
    t_odometry = msg_odometry.header.stamp.sec;
    if (t_odometry ~= t_odometry_prev) || (size(position_history,1) == 0)
        pose_history = [position_history; t_odometry, p.x, p.y, p.z, q.w, q.x, q.y, q.z];
    end
    
    % Parse Map Data
    msg_edt = receive(sub_edt, 0.1);
    map_size = length(msg_edt, 'intensity');
    t_map = msg_edt.header.stamp.sec;
    if (t_map ~= t_map_prev) || (size(map_size_history) == 0)
        map_size_history = [map_size_history; t_map, map_size];
    end
    % Parse planning time data (need to add this topic to planner node)
end

% Plot metrics

% Map volume vs time
figure(1)
plot(map_size_history(:,1), map_size_history(:,2)*(voxel_size^3));
xlabel('time (sec)');
ylabel('Volume explored')

% Percent Map volume vs time
total_map_size = 100;

% Save mat files for easy import to do multi-test comparison