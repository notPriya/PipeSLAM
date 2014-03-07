% % Distance using Pipe Joint tracking.
% close all;
% clear all;
% 
% % Load the frames.
% load('pipe1.mat');

%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize variables %
%%%%%%%%%%%%%%%%%%%%%%%%

% Constants based on pipe video.
pipe_radius = 10;
camera_f = 151.9179;

% Constants for loop.
n = size(frames, 4);
start = 1;

% Initialize loop variables.
c = [];
r = [];
pos = zeros(n, 3);

%% Iterate through the video.
for i = start:n
    % Extract the frame we want to process.
    I = frames(17:344, 10:430, :, i);
    
    % Find all the circles.
    if isempty(r)
        [c, r, p] = visualizePipeJoints(I);
    else
        [c, r, p] = visualizePipeJoints(I, c, r, p);
    end
    
    ratio = pipe_radius/r(1);
    
    delta = [(c(1, 1) - size(I, 2)/2) * ratio ...
             (c(1, 2) - size(I, 1)/2) * ratio ...
             ratio * camera_f];

    % Set the initial position so that the first frame is at (0, 0, 0).
    if i==1
        initial_pos = delta + [0 0 3];
    end 

         
    pos(i, :) = initial_pos - delta;
    
    if mod(i, 50) == 1
        disp(sprintf('Iteration %d finished', i));
    end
end

%% Visualize Results.

load('pipe1_groundtruth.mat');

figure;
hold on;
plot(pos, 'LineWidth', 2);
plot(ground_truth, 'k', 'LineWidth', 2);