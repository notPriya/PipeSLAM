% Tracks pipe joints using a Kalman Filter and a Hough Transform to take
% measurements of the actual circles in the image.
function [new_circle, features] = pipeJointTracker(I, weights, previous_circle, evaluation)

    % Model parameters.
    A = [eye(3) eye(3); zeros(3) eye(3)];
    H = [eye(3) zeros(3)];

    % Uncertianty of the state (robot movement).
    Q = previous_circle.state(3) * [0 0 0 0 0 0;
                                    0 0 0 0 0 0;
                                    0 0 0 0 0 0;
                                    0 0 0 .1 0 0;
                                    0 0 0 0 .1 0;
                                    0 0 0 0 0 .1];
    
    % Uncertainty of the measurement (circle prediction).
    % Uses a trough-like function centered around 110, with width 20,
    % and height 20.
    %    \               /  ___ 30
    %     \______|______/   ___ 20
    %    100    110    120
    scale = min(abs(previous_circle.state(3) - 110)-10, 0) + 20;
    R = scale * [.5 0 0;
                 0 .5 0;
                 0 0 1];
     
    % Measurement rejection Threshold
    error_threshold = 15;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Time Update (Prediction)          %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Estimate the new state based on model:  x_k = A * x_{k-1}
    state_prior = A * previous_circle.state;

    % Estimate the covariance:  P_k = A * P_{k-1} * A^T + Q
    covariance_prior = A * previous_circle.sigma * A' + Q;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Take a measurement                %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if previous_circle.real
        [measurement, features] = trackJoint(state_prior, covariance_prior, weights);
        found_circle = true;
    else
        [measurement, features] = initializeJoint(state_prior, covariance_prior, weights);
        % HACK: this makes sure that we arent going in the direction of the
        % update to our measurement in the next step.
        if ~isempty(measurement)
            state_prior(1:3) = measurement;
        end
        found_circle = ~isempty(measurement);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Outlier Rejection                 %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if ~isempty(measurement)
        S = H * covariance_prior * H' + R;
        error = measurement - H * state_prior;
        
        weighted_norm = (error'/S)*error;
        
        if weighted_norm > error_threshold
            measurement = [];
            found_circle = previous_circle.real;
        end
    end
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Measurement Update (Correction)   %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    % If there is no measurement, do not do the update step.
    if isempty(measurement)
        % The posteriors are the same as the priors.
        state_posterior = state_prior;
        covariance_posterior = covariance_prior;
        
        % Create a fake circle for visualization.
        measurement = [state_prior(1:2); state_prior(3)];
    else
        % Determine the Kalman Gain:  K = P_k * H^T (H * P_k * H^T + R)^-1
        kalman_gain = covariance_prior * H' / (H * covariance_prior * H' + R);

        % Update the state measure:  x_k = x_k + K (z_k - H * x_k)
        state_posterior = state_prior + kalman_gain * (measurement - H * state_prior);

        % Update the covariance measure:  P_k = (I - K*H) P_k
        covariance_posterior = (eye(6) - kalman_gain * H) * covariance_prior;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Create the return state     %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    new_circle.state = state_posterior;
    new_circle.sigma = covariance_posterior;
    new_circle.real = found_circle;
   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Visualize the found circles %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if ~evaluation
        % Take square root of the covariance. We take simple square root.
        sigma = sqrt(covariance_posterior);
        
        % Show the image.
%         imshow(I);
        hold on;
        % Draw the center covariance.
        if found_circle
            rectangle('Position', ...
                      [state_posterior(1)-sigma(1,1) ...
                       state_posterior(2)-sigma(2,2) ...
                       2*sigma(1,1) ...
                       2*sigma(2,2)], ...
                      'LineWidth',2,'LineStyle','-', 'EdgeColor', 'c');
        end 
        hold off;

        % Draw the final circle and the one sigma bound.
        viscircles(measurement(1:2)', measurement(3), 'EdgeColor', 'g');
        viscircles(state_posterior(1:2)', state_posterior(3), 'EdgeColor', 'k', 'LineStyle','--');
        % Dont do 1 sigma bounds for fake circles because they are often
        % negative.
        if found_circle
            if state_posterior(3) > 2*sigma(3,3)
                viscircles(state_posterior(1:2)', state_posterior(3)-2*sigma(3,3), 'EdgeColor', 'b','LineStyle','--');
            end
            viscircles(state_posterior(1:2)', state_posterior(3)+2*sigma(3,3), 'EdgeColor', 'b','LineStyle','--');
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
        
        %%%%%
        % Feature Computation
        %%%%%
        
        % Compute distance to the center of the image.
        center_distance = (new_circles(:, 1) - size(I, 2)/2).^2 + (new_circles(:,2) - size(I, 1)/2).^2;
        center_distance = (distance_max - center_distance)./distance_max;
        % If the we are further than the approximate max, give a low score.
        center_distance(center_distance <= 0) = epsilon;
                
        % Compute binary feature of whether the circle contains the dark blob.
        % Find the dark blob.
        ind = findDarkRegions(I);
        % Determine if it is contained.
        dist = pdist2(new_circles(:, 1:2), ind);
        contained = dist - repmat(new_circles(:, 3), 1, size(ind, 1)) < 0;
        % Score is the percentage of black blob you contain.
        black_blob_score = (sum(contained, 2)+epsilon)/(size(contained, 2)+epsilon);

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
        f = [new_circles(:, 4) center_distance black_blob_score radius_diff center_diff];
        f = log(f);  % Compute the negative log of scores.
    end

    % Finds the circles in the image that are close to the prior.
    function [measurement, features] = trackJoint(state_prior, covariance_prior, weights)
        % Estimate the position of the next circle based on the prior information.
        center_range = round([.95 1.06]*state_prior(3));
        center_range = [min(-5+floor(state_prior(3)), center_range(1)) max(5+ceil(state_prior(3)), center_range(2))];
        % HACK: keep the smaller circle from collapsing into itself.
        if (center_range(1) < 40)
            center_range = [40 max(center_range(2), 45)];
        end
        [center, radius, metric] = imfindcircles(I, center_range, 'EdgeThreshold', .05, 'Sensitivity', .995);

        measurement = [];
        features = [];
        
        if ~isempty(radius)
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
        end
    end

    % Finds the circles in the image that are good.
    function [measurement, features] = initializeJoint(state_prior, covariance_prior, weights)
        % Estimate the position of the next circle based on the prior information.
        center_range = round([.8 1.2]*state_prior(3));
        % HACK: keep the smaller circle from collapsing into itself.
        if (center_range(1) < 40)
            center_range = [40 max(center_range(2), 45)];
        end
        % HACK: smaller circles have a stricter threshold.
        if state_prior(3) < 80
            [center, radius, metric] = imfindcircles(I, center_range, 'EdgeThreshold', .1, 'Sensitivity', .98);
        else
            [center, radius, metric] = imfindcircles(I, center_range, 'EdgeThreshold', .1, 'Sensitivity', .99);
        end

        measurement = [];
        features = [];
        
        if ~isempty(radius)
            % Extract the features for each of the circles.
            features = getTrackingFeatures(I, [center radius metric], state_prior);

            % Determine the score from the learned weights.
            weights(1) = weights(1)*3; % Boost appearance score.
            weights(5) = weights(5)/3; % Demote center score.
            metric = features * weights;
            
            % Sort by the metric again.
            if ~isempty(radius)
                [~, ind] = sort(metric, 'descend');
                center = center(ind, :);
                radius = radius(ind);
                metric = metric(ind);
                features = features(ind, :);
            end

            % We found a bad circle.
            % HACK: smaller circles have lower threshold.
            if (metric < -23 & state_prior(3) < 90) | (metric < -30 & state_prior(3) > 90) 
                measurement = [];
                features = [];
                return;
            end

            % The measurement is the best circle.
            measurement = [center(1, :) radius(1)]';
        end
    end
end