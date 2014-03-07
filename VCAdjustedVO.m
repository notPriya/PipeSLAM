% clear all;
close all;
clc;

% % Read in the video.
% obj = VideoReader('videos/pipe_video/trial1.avi');
% frames = read(obj);
% n = size(frames, 4);
% 
% % Get the odometry bounding box.
% imshow(frames(:,:,:,2));
% [x, y] = ginput(2);
% odom_rect = [x' y'];
% close;
% 
% % Load the motion model data.
% load('vc_trial4_data.mat');
% n = size(recordLog.misc.Motions, 1);

disp('Starting processing');

%%%%
% Preprocess the data for virtual chassis adjustment.
%%%%
% [K, D, M, Hto1, errors] = calculateVCTransform(frames, f_ind, odom_rect, recordLog, n, K, D, M);

[Hto1_new, errors2, M_new] = calculateVCAdjustment(K, D, Hto1, recordLog, n, 10);

% save('trial4_data2.mat', 'K', 'D', 'M', 'Hto1', 'n');

disp('Processing complete.');

%% Calculate the overall transforms using the motion model.

T0 = eye(4);
T = cell(n,1);
for i=1:n
    if i == 1
        T{i} = T0 * recordLog.misc.Motions{i};
    else
        T{i} = T{i-1} * recordLog.misc.Motions{i};
    end
end

%% Save off the x-coordinate positions
figure;
hold on;
color = ['r', 'g', 'b'];
title('trial1');

for j=1
    zs = zeros(n, 1);
    gzs = zeros(n, 1);
    bzs = zeros(n, 1);
    for i=1:n
        zs(i) = Hto1{i}(j, 4);
        gzs(i) = T{i}(j, 4);
        bzs(i) = Hto1_new{i}(j, 4);
    end
end

threshold = .1;

ind = 1:length(zs);
% plot(ind(errors>threshold), zs(errors>threshold), 'r.');
% plot(ind(errors<threshold), zs(errors<threshold), 'k.');
plot(ind, zs, 'r.');
plot(ind, gzs, 'g.');
plot(ind, bzs, 'b.');