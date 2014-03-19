% Tracks pipe joints using a Kalman Filter and a Hough Transform to take
% measurements of the actual circles in the image.
function [state_posterior, covariance_posterior, features] = visualizePipeJoints(I, weights, previous_state, previous_covariance, evaluation)

    % Model parameters.
    A = [eye(3) eye(3); zeros(3) eye(3)];
    H = [eye(3) zeros(3)];
    
    % Uncertianty of the state (robot movement).
    Q = [0 0 0 0 0 0;
         0 0 0 0 0 0;
         0 0 0 0 0 0;
         0 0 0 10 0 0;
         0 0 0 0 10 0;
         0 0 0 0 0 10];
    
    % Uncertainty of the measurement (circle prediction).
    R = [10 0 0;
         0 10 0;
         0 0 20];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Time Update (Prediction)          %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Estimate the new state based on model:  x_k = A * x_{k-1}
    state_prior = A * previous_state;

    % Estimate the covariance:  P_k = A * P_{k-1} * A^T + Q
    covariance_prior = A * previous_covariance * A' + Q;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Take a measurement                %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%     % Smooth the image.
%     G = fspecial('Gaussian', 10, 5);
%     I2 = imfilter(I, G, 'replicate');
%     [~, t] = edge(rgb2gray(I2));
%     I2 = edge(rgb2gray(I2), 'canny', t + [.1 .3]);
    
    % Estimate the position of the next circle based on the prior information.
    center_range = round([.95 1.05]*state_prior(3));
    center_range = [min(-5+floor(state_prior(3)), center_range(1)) max(5+ceil(state_prior(3)), center_range(2))];
    % HACK: keep the smaller circle from collapsing into itself.
    if (center_range(1) < 40)
        center_range = [40 center_range(2)];
    end
    [center, radius, metric] = imfindcircles(I, center_range, 'EdgeThreshold', .05, 'Sensitivity', .995);
    
    % Extract the features for each of the circles.
    features = getTrackingFeatures(I, [center radius metric], state_prior);
    
    % Determine the score from the learned weights.
    metric = features * weights;
        
    % Sort by the metric again.
    if ~isempty(radius)
        [~, ind] = sort(metric, 'descend');
        center = center(ind, :);
        radius = radius(ind);
        metric = metric(ind);
        features = features(ind, :);
    end

    % The measurement is the best circle.
    measurement = [center(1, :) radius(1)]';
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Measurement Update (Correction)   %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Determine the Kalman Gain:  K = P_k * H^T (H * P_k * H^T + R)^-1
    kalman_gain = covariance_prior * H' / (H * covariance_prior * H' + R);
    
    % Update the state measure:  x_k = x_k + K (z_k - H * x_k)
    state_posterior = state_prior + kalman_gain * (measurement - H * state_prior);
    
    % Update the covariance measure:  P_k = (I - K*H) P_k
    covariance_posterior = (eye(6) - kalman_gain * H) * covariance_prior;
    
   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Visualize the found circles %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Determine value of k, for visualizing.
    k = min(3, size(radius, 1));

    if ~evaluation
        % Visualize the strongest circle in blue and all other circles in red.
%         imshow(I);
        hold on;
        % Draw the center covariance.
        rectangle('Position', [state_posterior(1)-covariance_posterior(1,1)/2 state_posterior(2)-covariance_posterior(2,2)/2 covariance_posterior(1,1) covariance_posterior(2,2)], ...
                  'LineWidth',2,'LineStyle','-', 'EdgeColor', 'c');
        hold off;

        if ~isempty(radius)
%             viscircles(center(1:k, :), radius(1:k), 'EdgeColor', 'r');
%             viscircles(state_prior(1:2)', state_prior(3), 'EdgeColor', 'm');
            viscircles(center(1, :), radius(1), 'EdgeColor', 'g');
            viscircles(state_posterior(1:2)', state_posterior(3)-covariance_posterior(3,3), 'EdgeColor', 'b','LineStyle','--');
            viscircles(state_posterior(1:2)', state_posterior(3)+covariance_posterior(3,3), 'EdgeColor', 'b','LineStyle','--');
            viscircles(state_posterior(1:2)', state_posterior(3), 'EdgeColor', 'b');
        end
    end
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Helper Functions      %
    %%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Computes features for a new set of circles.
    function f = getTrackingFeatures(I, new_circles, predicted_circle)
        % Number of datapoints.
        n = size(new_circles, 1);
        
        %%%%%
        % MAGIC NUMBERS BELOW:
        %%%%%
        
        % Some small number.
        epsilon = 0.0001;
        % Approximate maximum that the distance measures can be.
        distance_max = 7000;
        % Approximate maximum that the radius can be.
        radius_max = 10;
        % Number of pixels the image center must be from the border of the
        % circle.
        center_containment_threshold = 5;
        
        %%%%%
        % Feature Computation
        %%%%%
        
        % Compute distance to the center of the image.
        center_distance = (new_circles(:, 1) - size(I, 2)/2).^2 + (new_circles(:,2) - size(I, 1)/2).^2;
        center_distance = (distance_max - center_distance)./distance_max;
        % If the we are further than the approximate max, give a low score.
        center_distance(center_distance <= 0) = epsilon;
                
        % Compute binary feature of whether the circle contains the Image
        % center.
        distance = sqrt((new_circles(:, 1) - size(I, 2)/2).^2 + (new_circles(:,2) - size(I, 1)/2).^2);
        distance = new_circles(:, 3) - distance;
        contains_center = distance > center_containment_threshold;   
        % If the we do not contain the center, give a low score.
        contains_center(contains_center <= 0) = epsilon;

        % Compute the features for difference to predicted circle.
        % This one does difference from the center of the predicted circle.
        center_diff = (new_circles(:, 1) - predicted_circle(1)).^2 + (new_circles(:,2) - predicted_circle(2)).^2;
        center_diff = (distance_max - center_diff)./distance_max;
        % If the we are further than the approximate max, give a low score.
        center_diff(center_diff <= 0) = epsilon;

        % Compute the features for difference to predicted circle.
        % This one does difference of the radius of the predicted circle.
        radius_diff = abs(new_circles(:, 3) - predicted_circle(3));
        radius_diff = (radius_max - radius_diff)./radius_max;
        % If the we are further than the approximate max, give a low score.
        radius_diff(radius_diff <= 0) = epsilon;
        
        % F = [appearance
        %      distance to center of Image
        %      contains center of Image
        %      difference in radii of previous timestep
        %      difference in centers of previous timestep]
        f = [new_circles(:, 4) center_distance contains_center radius_diff center_diff];
        f = log(f);  % Compute the negative log of scores.
    end
end