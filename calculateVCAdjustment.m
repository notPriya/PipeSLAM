% Another minimization step in order to ensure correct distances.
function [H_new, errors, M] = calculateVCAdjustment(K, D, H, recordLog, n, stepsize)
    disp('Adjusting transforms');

    % Initialize variables.
    M = cell(floor(n/stepsize)-1, 1);
    H_new = cell(n, 1);
    H_new{1} = eye(4, 4);
    errors = zeros(floor(n/stepsize), 1);
    
    for i=1:stepsize:n-stepsize
        % Compute matches
        matches = matchSIFT(D{i}', D{i+stepsize}', 0.85, 3);
        
        M{(i+stepsize-1)/stepsize} = matches;

        % Extract the matched points.
        p1 = K{i}(matches(1, :), 1:2);
        p2 = K{i+stepsize}(matches(2, :), 1:2);
        
%         Hi_inv = [H{i}(1:3,1:3)' -H{i}(1:3,1:3)'*H{i}(1:3,4); 0 0 0 1];
%         T = Hi_inv*H{i+stepsize};
        
        T = eye(4);
        for j=1:stepsize
            T = T * recordLog.misc.Motions{i+j};
        end
        
        % Extract the translation from the transform.
        x0 = T(1,4);

        % Extract the transform from head to VC.
        H_tohead_old = recordLog.misc.snake_shape{i}(:,:,1);
        H_tohead_new = recordLog.misc.snake_shape{i+stepsize}(:,:,1);

        % Optimize transform to minimize reprojection errors.
        options = optimset('MaxFunEvals', 100000, 'MaxIter', 100000, 'Algorithm', 'levenberg-marquardt', 'Display', 'off');
        [xfinal, finalerror] = fminsearch(@(x) calculateReprojectionError(x, T, H_tohead_old, H_tohead_new, p1, p2, false), x0, options);  
        errors((i+stepsize-1)/stepsize) = finalerror;

%         % If the optimization went horribly wrong, just stick with the
%         % motion model estimate.
%         if (abs(xfinal) > 1.5*abs(x0))
%             xfinal = x0;
%         end

%         % Linearly interpolate the distance from start to end.
%         for j=1:stepsize
%             % Compute transform from optimized rotation for all inbetween
%             % transforms.
%             T = Hi_inv*H{i+j};
%             T(1,4) = j*xfinal/stepsize;
% 
%             H_new{i+j} = H_new{i} * T;
%         end

        % Linearly interpolate the distance from start to end.
        T = eye(4);
        for j=1:stepsize
            % Compute transform from optimized rotation for all inbetween
            % transforms.
            T = T * recordLog.misc.Motions{i+j};
            T(1,4) = j*xfinal/stepsize;

            H_new{i+j} = H_new{i} * T;
        end

        if (mod(i, 50) == 1)
            disp(i);
        end
    end
    
    % Fill in the blank cells at the end that were not a multiple of 5.
    left_overs = mod(n-stepsize-1, stepsize); % extra -1 for one indexing.
    for i=left_overs:-1:1
        % Find the transform between the end-i and end-i+1 frames of the
        % original transform.
        Hi_inv = [H{end-i}(1:3,1:3)' -H{end-i}(1:3,1:3)'*H{end-i}(1:3,4); 0 0 0 1];
        T = Hi_inv*H{end-i+1};
        
        % The transform to the end-i+1 frame is the old transform applied
        % to the end-i frame.
        H_new{end-i+1} = H_new{end-i} * T;
    end
end