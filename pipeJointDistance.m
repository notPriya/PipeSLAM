clc; close all;

pipe_name = 'pipe2';

if ~exist('frames', 'var')
    % Distance using Pipe Joint tracking.
    close all;
    clear all;
    clc;

    pipe_name = 'pipe2';
    
    % Load the frames.
    load([pipe_name '.mat']);
end

%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize variables %
%%%%%%%%%%%%%%%%%%%%%%%%

% Constants based on pipe video.
pipe_radius = 5;
camera_f = 510;  % MAGIC

% Constants for loop.
n = size(frames, 4);
start = 1;

% Weights on the features for picking best measurement.
weights = [1.73962175432486;0;3;3.34010706169493;6.71403253431558];

% Smaller circle intialization.
small_radius_guess = 55;  % MAGIC.
small_delta_radius_guess = .3; % MAGIC.

% Number on the distance counter in the first frame.
initial_camera_distance = 259;

% Interpolation factor on estimating distance using the larger and
% smaller tracked circles.
alpha = .7;  % Weight on the larger circle estimation

% Flag for visualizing.
evaluation = true;

% Setup stuff for making a movie.
doMovie = false;
if doMovie
    % Open a movie object.
    movie = VideoWriter('pipe_joint_tracking7.avi');
    movie.FrameRate = 15;
    open(movie);
    evaluation = false;
end

if ~exist('init_state', 'var')
    % Get the initialization of the first circle.
    I = frames(:, :, :, start);
    imshow(I);
    [x,y] = ginput(2);

    % Create the first state from the initialization.
    c = [x(1); y(1)];
    r = sqrt( (x(1)-x(2))^2 + (y(1)-y(2))^2 );
    init_state = [c; r; 0; 0; 0];
end

% Initialize loop variables.
big_circle.state = init_state;
big_circle.sigma = diag([10 10 10 5 5 5]);
big_circle.real = true;

small_circle = [];

pos = zeros(n, 3);


%% Iterate through the video.
for i = start:n
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Get Frame             %
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Extract the frame we want to process.
%     I = frames(1:376,:,:, i);
    I = frames(:,:,:, i);
    if ~evaluation
        imshow(frames(:,:,:, i));
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Track Circles         %
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Track the bigger circle.
    % HACK: if the circle is not real, move it towards the black blob in
    % the image.
    if ~big_circle.real
        ind = findDarkRegions(I);
        centroid = median(ind)';  % More outlier insensitive than mean.
        velocity = (centroid - big_circle.state(1:2))/100;
        big_circle.state(4:5) = velocity;
    end
    [big_circle] = pipeJointTracker(I, weights, big_circle, evaluation);
        
    % Track the smaller circle.
    if ~isempty(small_circle)
        % HACK: if the smaller circle is fake, make the center the same as
        % the bigger one.
        if ~small_circle.real
            small_circle.state(1:2) = big_circle.state(1:2);
        end
        [small_circle] = pipeJointTracker(I, weights, small_circle, evaluation);
    end
 
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Handle new circles    %
    %%%%%%%%%%%%%%%%%%%%%%%%%
    
    % If the big circle gets too large, initialize the smaller circle.
    if big_circle.state(3) > 200 && isempty(small_circle)
        % Create a guess for where our circle should be.
        small_circle_guess.state = [big_circle.state(1:2); small_radius_guess; 0; 0; small_delta_radius_guess];
        small_circle_guess.sigma = big_circle.sigma;
        small_circle_guess.real = false;
        
        % Start tracking the smaller circle.
        [small_circle] = pipeJointTracker(I, weights, small_circle_guess, evaluation);
    end
    
    if big_circle.state(3) > 240
        % Swap the bigger and smaller circles.
        big_circle = small_circle;
        small_circle = [];
                
        % Change the initial pos to be that of this circle.
        initial_pos = initial_pos2;
        initial_pos2 = [];
    end
    
    if (isempty(big_circle.state))
        disp('ERROR: Did not find a circle');
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Calculate Distances   %
    %%%%%%%%%%%%%%%%%%%%%%%%%    
    % Get the scale ratio.
    ratio = pipe_radius/big_circle.state(3);
    
    % Convert measurements from pixels to deci-feet.
    delta = [(big_circle.state(1) - size(I, 2)/2) * ratio ...
             (big_circle.state(2) - size(I, 1)/2) * ratio ...
             ratio * camera_f];

    % Set the initial position so that the first frame is at (0, 0, 0).
    if i==start
        initial_pos = delta + [0 0 initial_camera_distance];
    end 
         
    % Get distance information from the smaller circle.
    if ~isempty(small_circle)
        ratio2 = pipe_radius/small_circle.state(3);
        
        delta2 = [(small_circle.state(1) - size(I, 2)/2) * ratio2 ...
                  (small_circle.state(2) - size(I, 1)/2) * ratio2 ...
                  ratio2 * camera_f];
          
        % Set the initial position so that the first frame is at the
        % current position.
        if ~exist('initial_pos2', 'var') || isempty(initial_pos2)
            initial_pos2 = delta2 + initial_pos - delta;
        end
    end

         
    % Calculate the position of the camera.
    if ~isempty(small_circle) && small_circle.real
        pos(i, :) = alpha*(initial_pos - delta) + (1-alpha)*(initial_pos2 - delta2);
    else
        pos(i, :) = initial_pos - delta;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Visualization Stuff   %
    %%%%%%%%%%%%%%%%%%%%%%%%%

    % Add text displaying the distance traveled.
%     if ~evaluation
%         text(447, 90, sprintf('%.1f ft', pos(i, 3)/10), 'FontSize', 30, 'Color', 'blue');
%     end
    
    if doMovie
        % Write the frame to the video.
        frame = getframe;
        writeVideo(movie, frame);
        
        close;
    else  
        if ~evaluation
            drawnow;
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

load([pipe_name '_groundtruth.mat']);

figure;
hold on;
plot(pos, 'LineWidth', 2);
plot(ground_truth, 'k', 'LineWidth', 2);

% Calculate the error per unit distance.
mean(abs(ground_truth(:) - pos(:, 3)))
(ground_truth(end) - pos(end, 3))/(ground_truth(1) - ground_truth(end))