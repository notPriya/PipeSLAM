% Finds pipe joints using Hough Transform.
function [state, features] = visualizePipeJoints(I, weights, previous_state, evaluation)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Estimate position of new circles. %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % The predicted placement of the new circle is the first order
    % approximation.
    new_circles = previous_state(1:3); % + previous_state(4:6);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Find the possible circles. %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Estimate the position of the next circle based on the previous state.
    % Currently finds circles within some pixels of the 0th-order estimation.
    center_range = round([.95 1.05]*new_circles(3));
    center_range = [min(-5+round(new_circles(3)), center_range(1)) max(5+round(new_circles(3)), center_range(2))];
    [center, radius, metric] = imfindcircles(I, center_range, 'EdgeThreshold', .05, 'Sensitivity', .995);
                
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Find the possible lines and intersections. %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%     % We only care about the first one because it has the highest
%     % appearance score.
%     % TODO: do another sort of Max Hypothesis filtering here.
%     lines = imfindlines(I);
%     
%     % If we couldnt actually find two lines use the center of the image for
%     % plotting purposes.
%     if ~isempty(lines)
%         intersect = findIntersection(lines);
%     else
%         intersect = [size(I, 2)/2 size(I, 1)/2];
%     end
%     
%     % If we couldnt find an intersection point, use the center of the
%     % image.
%     if isempty(intersect)
%         intersect = [size(I, 2)/2 size(I, 1)/2];
%     end
%     
%     % If the intersect point is bad, use the center of the image.
%     dist = sum([1 0; 0 .1]*(intersect' - [size(I,2)/2; size(I,1)/2]).^2);
%     if (dist > 1000)
%         intersect = [size(I, 2)/2 size(I, 1)/2];
%     end
    intersect = [size(I, 2)/2 size(I, 1)/2];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Extract features and determine the score. %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if nargin > 1
        % Extract the features for each of the circles.
        features = getTrackingFeatures(I, intersect, [center radius metric], new_circles);
    else
        % Extract the features for each of the circles.
        features = getTrackingFeatures(I, intersect, [center radius metric]);        
    end
    
    % Determine the score from the learned weights.
    weights = [weights(1:2); 0; weights(3:5)];
%     weights = [1.18965619416973,0.773250960852945,1.92789150286924,20.44340934195353,3.44340934195353,6.44340934195353]';
    metric = features * weights;
        
    % Sort by the metric again.
    if ~isempty(radius)
        [~, ind] = sort(metric, 'descend');
        center = center(ind, :);
        radius = radius(ind);
        metric = metric(ind);
        features = features(ind, :);
    end
   
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Visualize the Circles %
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Determine value of k, for visualizing and returning the k best
    % circles.
    k = min(3, size(radius, 1));

    if ~evaluation
        % Visualize the strongest circle in blue and all other circles in red.
%         imshow(I);
        hold on;
        plot(size(I, 2)/2, size(I, 1)/2, 'm.', 'MarkerSize', 15);
        plot(center(1, 1), center(1, 2), 'c.', 'MarkerSize', 15);
        plot(intersect(1), intersect(2), 'y.', 'MarkerSize', 15);
        hold off;

        if ~isempty(radius)
%             viscircles(center(1:k, :), radius(1:k), 'EdgeColor', 'r');
            viscircles(center(1, :), radius(1), 'EdgeColor', 'b');
        end
    end
    
    % Return the best circle.
    state = [];
    if ~isempty(radius)
        state = [center(1, :) radius(1)];
        state = [state state-previous_state(1:3)];
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Helper Functions      %
    %%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Function that converts the metric into a probability distribution.
    % Currently just normalizes the metric into a probability.
    function p = metric2prob(m)
        % Convert back to exponentials.
        p = exp(m);
        % Normalize.
        p = p/sum(p);
    end

    % Function that calculates the intersection of two lines.
    function intersect = findIntersection(line)
        xy1 = [line(1).point1; line(1).point2];
        xy2 = [line(2).point1; line(2).point2];
        
        % Calculate intersection of the two lines.
        [x, y] = polyxpoly(xy1(:,1), xy1(:,2), xy2(:, 1), xy2(:, 2));
        intersect = [x y];
    end

    function f = getTrackingFeatures(I, intersect, new_circles, predicted_circle)
        % Number of datapoints.
        n = size(new_circles, 1);
        
        % Compute distance to the center of the image.
        center_distance = (new_circles(:, 1) - size(I, 2)/2).^2 + (new_circles(:,2) - size(I, 1)/2).^2;
        center_distance = (7000 - center_distance)./7000;
        % If the we are further than 7000, make the probability near 0.
        center_distance(center_distance <= 0) = 0.0001;
        
        % Compute distance to the intersection of the water level lines.
        intersect = repmat(intersect', 1, n);
        line_distance = sum([1 0; 0 .1] * (new_circles(:,1:2)' - intersect).^2);
        line_distance = (7000 - line_distance')./7000;
        % If the we are further than 7000, make the probability near 0.
        line_distance(line_distance <= 0) = 0.0001;
        
        % Compute binary feature of whether the circle contains the Image
        % center.
        distance = sqrt((new_circles(:, 1) - size(I, 2)/2).^2 + (new_circles(:,2) - size(I, 1)/2).^2);
        distance = new_circles(:, 3) - distance;
        contains_center = distance > 5;   
        % If we dont actually contain the center, make the probability near 0.
        contains_center(contains_center <= 0) = 0.0001;

        % Compute the features for difference to predicted circle.
        % This one does difference from the center of the predicted circle.
        center_diff = (new_circles(:, 1) - predicted_circle(1)).^2 + (new_circles(:,2) - predicted_circle(2)).^2;
        center_diff = (7000 - center_diff)./7000;
        % If the we are further than 7000, make the probability near 0.
        center_diff(center_diff <= 0) = 0.0001;

        % Compute the features for difference to predicted circle.
        % This one does difference of the radius of the predicted circle.
        radius_diff = abs(new_circles(:, 3) - predicted_circle(3));
        radius_diff = (10 - radius_diff)./10;
        % If the we are larger than 10, make the probability near 0.
        radius_diff(radius_diff <= 0) = 0.0001;
        

        % F = [appearance
        %      distance to center of Image
        %      distance to intersection of waterlevels
        %      contains center of Image
        %      difference in radii of previous timestep
        %      difference in centers of previous timestep]
        f = [new_circles(:, 4) center_distance line_distance contains_center radius_diff center_diff];
        f = log(f);  % Compute the negative log.
    end
end