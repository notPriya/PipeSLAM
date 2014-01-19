clear all;
close all;
clc;

% Read in the video.
obj = VideoReader('pipe1_clean.mp4');
frames = read(obj);

start_frame = 1000;
end_frame = 1571;

num_frames = end_frame - start_frame;

% Get the odometry bounding box.
imshow(frames(:,:,:,1));
[x, y] = ginput(2);
odom_rect = [x' y'];

% Calculate the transform from one frame to the next.
H = cell(num_frames, 1);
E = cell(num_frames, 1);
for i=1:num_frames
    I = frames(:,:,:,i+start_frame);
    I2 = frames(:,:,:,i+1+start_frame);
    [H2to1, E2to1] = calculateTransform(I, I2, odom_rect);
    H{i} = H2to1;
    E{i} = E2to1;
end

% Calculate the transform to frame 1.
Hto1 = cell(num_frames, 1);
Hto1{1} = H{1};
for i=2:num_frames
    Hto1{i} = Hto1{i-1} * H{i};
end

% Draw the robot position and orientation.
figure;
hold on;
for i=1:num_frames+1
    if i == 1
        plot3([0 .1], [0 0], [0 0], 'r');
        plot3([0 0], [0 .1], [0 0], 'g');
        plot3([0 0], [0 0], [0 .1], 'b');        
        plot3(0, 0, 0, 'o', 'MarkerSize', 2, 'LineWidth', 2);
    else
        o = Hto1{i-1}(1:3, 4);
        x = Hto1{i-1}(1:3, 1)*.1 + o;
        y = Hto1{i-1}(1:3, 2)*.1 + o;
        z = Hto1{i-1}(1:3, 3)*.1 + o;
        plot3([o(1) x(1)], [o(2) x(2)], [o(3) x(3)], 'r');
        plot3([o(1) y(1)], [o(2) y(2)], [o(3) y(3)], 'g');
        plot3([o(1) z(1)], [o(2) z(2)], [o(3) z(3)], 'b');
        plot3(o(1), o(2), o(3), 'o', 'MarkerSize', 2, 'LineWidth', 2);
    end
end

plot3(o(1), o(2), o(3), 'rx', 'MarkerSize', 15, 'LineWidth', 2);
plot3(0, 0, 0, 'gx', 'MarkerSize', 15, 'LineWidth', 2);
axis equal;