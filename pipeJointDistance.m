% % Distance using Pipe Joint tracking.
% close all;
% clear all;
% 
% % Load the frames.
% load('pipe2.mat');


%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize variables %
%%%%%%%%%%%%%%%%%%%%%%%%

% Constants based on pipe video.
pipe_radius = 5;
camera_f = 364.2857;  % MAGIC

% Constants for loop.
n = size(frames, 4);
start = 1;

% Flag for visualizing.
evaluation = true;

% Setup stuff for making a movie.
doMovie = true;
if doMovie
    % Open a movie object.
    movie = VideoWriter('pipe_joint_tracking9.avi');
    open(movie);
    evaluation = false;
end


% % Get the initialization of the first circle.
% I = frames(10:end, 10:end, :, start);
% % I = frames(17:344, 10:430, :, start);
% imshow(I);
% [x,y] = ginput(2);
% 
% % Create the first state from the initialization.
% c = [x(1) y(1)];
% r = sqrt( (x(1)-x(2))^2 + (y(1)-y(2))^2 );
% init_state = [c r 0 0 0];

% Initialize loop variables.
state = init_state;

smaller_state = [];

pos = zeros(n, 3);


%% Iterate through the video.
for i = start:n
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Get Frame             %
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Extract the frame we want to process.
    I = frames(10:end,10:end,:, i);
%     I = frames(17:344, 10:430, :, i);
    if ~evaluation
        imshow(I);
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Track Circles         %
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Track the bigger circle.
    [state] = visualizePipeJoints(I, state, evaluation);
        
    % Track the smaller circle.
    if ~isempty(smaller_state)
        [smaller_state] = visualizePipeJoints(I, smaller_state, evaluation);
        
        % Swap the bigger and smaller states once the inner circle gets big
        % enough or the outer circle gets too big.
        if (smaller_state(3) > 80 || state(3) > 220)
            state = smaller_state;
            smaller_state = [];
            
            % Change the initial pos to be that of this circle.
            initial_pos = initial_pos2;
            initial_pos2 = [];
        end
    % If the circle gets too large, its hard to track, and we want to
    % initialize the smaller circle to track.
    elseif (state(3) > 180)
        % Initialize the smaller circle with radius 50.
        [smaller_state] = visualizePipeJoints(I, [state(1:2) 50 0 0 0], evaluation);        
    end
       
    if (isempty(state))
        disp('ERROR: Did not find a circle');
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Calculate Distances   %
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Get the scale ratio.
    ratio = pipe_radius/state(3);
    
    % Convert measurements from pixels to deci-feet.
    delta = [(state(1) - size(I, 2)/2) * ratio ...
             (state(2) - size(I, 1)/2) * ratio ...
             ratio * camera_f];
         
    % Average in information from the smaller circle.
    if ~isempty(smaller_state)
        ratio2 = pipe_radius/smaller_state(3);
        
        delta2 = [(smaller_state(1) - size(I, 2)/2) * ratio2 ...
                  (smaller_state(2) - size(I, 1)/2) * ratio2 ...
                  ratio2 * camera_f];
          
        % Set the initial position so that the first frame is at the
        % current position.
        if ~exist('initial_pos2', 'var') || isempty(initial_pos2)
            initial_pos2 = delta2 + initial_pos - delta;
        end
    end

    % Set the initial position so that the first frame is at (0, 0, 0).
    if i==start
        initial_pos = delta + [0 0 259];
    end 
         
    % Calculate the position of the camera.
    if ~isempty(smaller_state)
        pos(i, :) = .7*(initial_pos - delta) + .3*(initial_pos2 - delta2);
    else
        pos(i, :) = initial_pos - delta;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Visualization Stuff   %
    %%%%%%%%%%%%%%%%%%%%%%%%%

    % Add text displaying the distance traveled.
    if ~evaluation
        text(440, 83, sprintf('%.1f ft', pos(i, 3)/10), 'FontSize', 30, 'Color', 'blue');
    end
    
    if doMovie
        % Write the frame to the video.
        frame = getframe;
        writeVideo(movie, frame);
        
        close;
    else  
        if ~evaluation
            pause(0.05);
        end
        if mod(i, 50) == 1
            disp(sprintf('Iteration %d finished', i));
        end
    end
end

if doMovie
    % Finish the movie.
    close(movie);
end

%% Display position results.

load('pipe1_groundtruth.mat');

figure;
hold on;
plot(pos, 'LineWidth', 2);
plot(ground_truth, 'k', 'LineWidth', 2);