% Finds pipe joints using Hough Transform.
function [c, r, p, features] = visualizePipeJoints(I, previous_c, previous_r, previous_p)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Find the possible circles. %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % This is not very sensitive since we have such poor illumination and
    % crap features. We try not to miss any circles.
    [center1, radius1, metric1] = imfindcircles(I, [50 100], 'EdgeThreshold', .05, 'Sensitivity', .98);
    [center2, radius2, metric2] = imfindcircles(I, [100 150], 'EdgeThreshold', .05, 'Sensitivity', .99);
    [center3, radius3, metric3] = imfindcircles(I, [130 150], 'EdgeThreshold', .05, 'Sensitivity', .995);
    
    % Put all the circle information together to find the best circle.
    % Metric 3 circles suck, so add a boost to their metric since they can
    % be more likely.
    center = [center1; center2; center3];
    radius = [radius1; radius2; radius3];
    metric = [metric1; metric2; metric3*2];
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Find the possible lines and intersections. %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % We only care about the first one because it has the highest
    % appearance score.
    % TODO: do another sort of Max Hypothesis filtering here.
    lines = imfindlines(I);
    
    % If we couldnt actually find two lines use the center of the image for
    % plotting purposes.
    if ~isempty(lines)
        intersect = findIntersection(lines);
    else
        intersect = [size(I, 2)/2 size(I, 1)/2];
    end
    
    % If we couldnt find an intersection point, use the center of the
    % image.
    if isempty(intersect)
        intersect = [size(I, 2)/2 size(I, 1)/2];
    end
    
    % If the intersect point is bad, use the center of the image.
    dist = sum([1 0; 0 .1]*(intersect' - [size(I,2)/2; size(I,1)/2]).^2);
    if (dist > 1000)
        intersect = [size(I, 2)/2 size(I, 1)/2];
    end
        

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Extract features and determine the score. %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if nargin > 1
        % Extract the features for each of the circles.
        features = getTrackingFeatures(I, intersect, [center radius metric], [previous_c previous_r previous_p]);
    else
        % Extract the features for each of the circles.
        features = getTrackingFeatures(I, intersect, [center radius metric]);        
    end
    
    % Determine the score from the learned weights.
    weights = [1.18965619416973,0.773250960852945,1.92789150286924,20.44340934195353,6.44340934195353]';
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

    % Visualize the strongest circle in blue and all other circles in red.
    imshow(I);
    hold on;
    plot(size(I, 2)/2, size(I, 1)/2, 'm.', 'MarkerSize', 15);
    plot(center(1, 1), center(1, 2), 'c.', 'MarkerSize', 15);
    plot(intersect(1), intersect(2), 'y.', 'MarkerSize', 15);
    hold off;
    
    if ~isempty(radius)
%         viscircles(center(1:k, :), radius(1:k), 'EdgeColor', 'r');
        viscircles(center(1, :), radius(1), 'EdgeColor', 'b');
    end
    
    % Return the best k circles.
    c = [];
    r = [];
    p = [];
    if ~isempty(radius)
        c = center(1:k, :);
        r = radius(1:k);
        p = metric2prob(metric(1:k));
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

    % Add information about the previous radius and center.
    % This penalizes changes from the all datapoints weighted by their
    % prior probability.
    function mult_hypothesis_prior = multipleHypothesis(new_circles, old_circles)
        n = size(new_circles, 1);
        m = size(old_circles, 1);
        
        % TODO: add alpha here if it doesnt work.
        mult_hypothesis_prior = zeros(n, 1);
        % Iterate through all the old circles and compute the individual
        % likelihoods of originating from that circle.
        for i=1:m
            closest = i*ones(n, 1);
            
            % Determine the penality for the new point based on difference to
            % the old point.
            radius_diff = abs(new_circles(:, 3) - old_circles(closest, 3));
            radius_diff = (200 - radius_diff)./200;
            % Logify.
            radius_diff = log(radius_diff);

            center_diff = sum((new_circles(:, 1:2) - old_circles(closest, 1:2)).^2, 2);
            center_diff = (7000 - center_diff)./7000;
            % If the we are further than 7000, make the probability near 0.
            center_diff(center_diff < 0) = 0.0001;
            % Logify.
            center_diff = log(center_diff);
            
            previous_likelihood = log(old_circles(closest, 4));
            
            circle_prior = exp(radius_diff + center_diff + previous_likelihood);
            mult_hypothesis_prior = mult_hypothesis_prior + circle_prior;
        end
    end

    function f = getTrackingFeatures(I, intersect, new_circles, old_circles)
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

        % Compute the features for the multiple hypothesis. 
        mult_hypothesis = ones(n, 1);
        if nargin == 4
            mult_hypothesis = multipleHypothesis(new_circles, old_circles);
        end     
        
        % F = [appearance
        %      distance to center of Image
        %      distance to intersection of waterlevels
        %      contains center of Image
        %      difference in radii of previous timestep
        %      difference in centers of previous timestep
        %      probability estimate of previous circle]
        f = [new_circles(:, 4) center_distance line_distance contains_center mult_hypothesis];
        f = log(f);  % Compute the negative log.
    end
end