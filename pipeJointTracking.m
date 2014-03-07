% Pipe Joint tracking.
close all;
clear all;

% Load the frames.
load('pipe1.mat');

doMovie = true;

if doMovie
    % Open a movie object.
    movie = VideoWriter('pipe_joint_tracking5.avi');
    open(movie);
end

start = 1;

% I = frames(17:344, 10:430, :, i);
% imshow(I);
% [x,y] = ginput(2);
% 
% c = [x(1) y(1)];
% r = sqrt( (x(1)-x(2))^2 + (y(1)-y(2))^2 );
% p = 1;

% Create each frame of the video.
c = [];
r = [];
for i = start:size(frames, 4)
    % Extract the frame we want to process.
    I = frames(17:344, 10:430, :, i);
    
    % Find all the circles.
    if isempty(r)
        [c, r, p] = visualizePipeJoints(I);
    else
        [c, r, p] = visualizePipeJoints(I, c, r, p);
    end
    
    if doMovie
        % Write the frame to the video.
        frame = getframe;
        writeVideo(movie, frame);
    end
    
    if ~doMovie
        pause(0.05);
        if mod(i, 50) == 0
            close;
        end
    else
        close;
    end
end

if doMovie
    % Finish the movie.
    close(movie);
end