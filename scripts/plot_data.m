% Read in .csv files for map and pose histories
tests = 1:2;
% tests = 1:4;
file_dir = "/home/andrew/tests/data/gain_urban_test_";
% file_dir = "/home/andrew/tests/data/test";
voxel_sizes = [];
map_data_sizes = [];
% map_history = cell(1,length(tests));
% pose_history = cell(1,length(tests));
for i=tests
    % Get filenames
    map_filename = sprintf('%s%d_map.csv', file_dir, i);
    pose_filename = sprintf('%s%d_poses.csv', file_dir, i);
    
    % Read in data
    voxel_sizes = [voxel_sizes, csvread(map_filename,0,0,[0,0,0,0])];
    map_data = csvread(map_filename,1,0);
    map_history{i} = map_data;
    map_data_sizes = [map_data_sizes, size(map_data,1)];
    pose_history{i} = csvread(pose_filename);
end

% Plot
figure(1)
title("Total Map Volume vs Time");
xlabel("time ($s$)", 'Interpreter','latex');
ylabel("Map volume ($m^3$)", 'Interpreter','latex');
hold on

for i=tests
    map_data = map_history{i};
    t = map_data(:,1);
    t = t - t(1);
    volume = (voxel_sizes(i)^3)*(map_data(:,2) + map_data(:,3));
    plot(t,volume);
end

% Calculate the mean and standard deviation across all tests
[data_len, max_data_set] = max(map_data_sizes); 
volume_stats = zeros(length(tests),data_len);

for i=tests
    map_data = map_history{i};
    volume_stats(i,1:map_data_sizes(i)) = (voxel_sizes(i)^3)*(map_data(:,2) + map_data(:,3));
end
populations = sum((volume_stats > 0.0),1);
means = sum(volume_stats,1)./populations;
std_devs = sqrt(sum(((volume_stats) > 0.0).*((volume_stats - repmat(means,length(tests),1)).^2), 1)./populations);
t = map_history{max_data_set}(:,1)';
t = t - t(1);

figure(2)
title("Total Map Volume vs Time");
xlabel("time ($s$)", 'Interpreter','latex');
ylabel("Map volume ($m^3$)", 'Interpreter','latex');
hold on
plot(t, means, 'b');
y1 = means - 2*std_devs;
y2 = means + 2*std_devs;
plot(t, y1, 'b--');
plot(t, y2, 'b--');
fill([t,fliplr(t)], [y1,fliplr(y2)], 'b', 'FaceAlpha', 0.1, 'EdgeAlpha', 0.0);
