clear all;
close all;
clc;

% Read in the video.
obj = VideoReader('videos/pipe_video/trial3.avi');
frames = read(obj);
n = 500;
% load('pipe1.mat');
% n = size(frames, 4);

% Get the odometry bounding box.
imshow(frames(:,:,:,2));
[x, y] = ginput(2);
odom_rect = [x' y'];

disp('Starting preprocessing');

%%%%
% Preprocess the data for bundle adjustment.
%%%%
[K, D, M, Hto1] = preprocessBAData(frames, odom_rect, n);
old_Hto1 = Hto1;

save('pipe1_data3.mat', 'K', 'D', 'M', 'Hto1', 'n');

disp('Preprocessing complete. Starting bundle adjustment.');

%%%%
%% Bundle Adjust all transforms.
%%%%
Hto1 = old_Hto1;
for i=1:n-3
    % Put everything into the frame of camera 1.
    H1_inv = [Hto1{i}(1:3, 1:3)' -Hto1{i}(1:3, 1:3)'*Hto1{i}(1:3, 4); 0 0 0 1];
    H2 = H1_inv * Hto1{i+1};
    H3 = H1_inv * Hto1{i+2};
    H4 = H1_inv * Hto1{i+3};
    
    
    % Perform the bundle adjustment.
    [Hi1, Hi2, Hi3] = calculateBATransform(K{i}, D{i}, K{i+1}, D{i+1}, ...
                                           K{i+2}, D{i+2}, K{i+3}, D{i+3}, ...
                                           M{i}, M{i+1}, M{i+2}, ...
                                           Hto1{i+1}, Hto1{i+2}, Hto1{i+3}, i);
                                       
    % Update the transforms. Put everything into the frame of the world.
    Hto1{i+1} = Hto1{i}*[Hi1; 0 0 0 1];
    Hto1{i+2} = Hto1{i}*[Hi2; 0 0 0 1];
    Hto1{i+3} = Hto1{i}*[Hi3; 0 0 0 1];
    
    if (mod(i, 100) == 0)
        disp(i);
    end
end

%% Draw the adjusted robot position and orientation.
figure;
hold on;

for i=1:n
    o = Hto1{i}(1:3, 4);
    x = Hto1{i}(1:3, 1)*.1 + o;
    y = Hto1{i}(1:3, 2)*.1 + o;
    z = Hto1{i}(1:3, 3)*.1 + o;
    plot3([o(1) x(1)], [o(2) x(2)], [o(3) x(3)], 'r');
    plot3([o(1) y(1)], [o(2) y(2)], [o(3) y(3)], 'g');
    plot3([o(1) z(1)], [o(2) z(2)], [o(3) z(3)], 'b');
    plot3(o(1), o(2), o(3), 'o', 'MarkerSize', 2, 'LineWidth', 2);    
end

plot3(o(1), o(2), o(3), 'rx', 'MarkerSize', 15, 'LineWidth', 2);
plot3(0, 0, 0, 'gx', 'MarkerSize', 15, 'LineWidth', 2);
axis equal;

%% Draw the original robot position and orientation.
figure;
hold on;

for i=1:n
    o = old_Hto1{i}(1:3, 4);
    x = old_Hto1{i}(1:3, 1)*.1 + o;
    y = old_Hto1{i}(1:3, 2)*.1 + o;
    z = old_Hto1{i}(1:3, 3)*.1 + o;
    plot3([o(1) x(1)], [o(2) x(2)], [o(3) x(3)], 'r');
    plot3([o(1) y(1)], [o(2) y(2)], [o(3) y(3)], 'g');
    plot3([o(1) z(1)], [o(2) z(2)], [o(3) z(3)], 'b');
    plot3(o(1), o(2), o(3), 'ro', 'MarkerSize', 2, 'LineWidth', 2);    
end

plot3(o(1), o(2), o(3), 'rx', 'MarkerSize', 15, 'LineWidth', 2);
plot3(0, 0, 0, 'gx', 'MarkerSize', 15, 'LineWidth', 2);
axis equal;

%% Save off the x-coordinate positions
figure;
hold on;
color = ['r', 'g', 'b'];
title('trial2');

for j=1:3
    zs = zeros(n, 1);
    for i=1:n
        zs(i) = -Hto1{i}(j, 4)*0.0025;
    end

    plot(zs, color(j));
end