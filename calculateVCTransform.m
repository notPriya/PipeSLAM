% Preprocess the features, descriptors, and matches for Virtual Chassis adjustment.
% Then calculates the transforms to the first frame.
function [K, D, M, H, errors] = calculateVCTransform(frames, f_ind, odom_rect, recordLog, n, K, D, M)
    disp('Preprocessing the features and descriptors');
    
%     % Preprocess the features and descriptors.
%     K = cell(n, 1);
%     D = cell(n, 1);
%     for i = 1:n
%         % Preprocess the image.
%         I = frames(:,:,:,f_ind(i));
%         im = preprocessImage(I);
%         
%         % Calculate the features.
%         k1 = calculateFeatures(im, odom_rect);
%         
%         % Make sure we have at least 10 features. Bounce around before and
%         % after the intended frame until we get nicer images. 
%         deviation = 1;
%         while size(k1, 1) < 10
%             if deviation > 0
%                 deviation = -deviation;
%             else
%                 deviation = -deviation + 1;
%             end
%             I = frames(:,:,:,f_ind(i)+deviation);
%             im = preprocessImage(I);
%             k1 = calculateFeatures(im, odom_rect);
%         end
%         
%         % Calculate the descriptors.
%         d1 = calculateDescriptors(im, k1, I);
%         
%         % Save off the results.
%         K{i} = k1;
%         D{i} = d1;
%         
%         if (mod(i, 50) == 1)
%             disp(i);
%         end
%     end
%     
    
    disp('Preprocessing the matches and transforms');
    
    % Preprocess the matches and transforms.
%     M = cell(n-1, 1);
    H = cell(n, 1);
    H{1} = eye(4, 4);
    errors = zeros(n, 2);
    for i=1:n-1
        % Compute matches
%         matches = matchSIFT(D{i}', D{i+1}', 0.85, 3);
%         M{i} = matches;

        matches = M{i};

        % Extract the matched points.
        p1 = K{i}(matches(1, :), 1:2);
        p2 = K{i+1}(matches(2, :), 1:2);
        
        % Get the transform from the motion model.
        T = recordLog.misc.Motions{i+1};

        % Extract rotation and translation from the transform.
        x0 = [T(1,4);  atan2(T(3,2), T(2,2))];
        
        % Extract the transform from head to VC.
        H_tohead_old = recordLog.misc.snake_shape{i}(:,:,1);
        H_tohead_new = recordLog.misc.snake_shape{i+1}(:,:,1);
        
        original_error = calculateReprojectionError(x0, T, H_tohead_old, H_tohead_new, p1, p2, true);
%         H_old_inv = [H_tohead_old(1:3, 1:3)' -H_tohead_old(1:3, 1:3)'*H_tohead_old(1:3, 4); 0 0 0 1];
%         meow = H_tohead_new*T*H_old_inv;

        % Optimize transform to minimize reprojection errors.
        options = optimset('MaxFunEvals', 100000, 'MaxIter', 100000, 'Algorithm', 'levenberg-marquardt', 'Display', 'off');
        [xfinal, finalerror] = fminsearch(@(x) calculateReprojectionError(x, T, H_tohead_old, H_tohead_new, p1, p2, false), x0, options);  
        errors(i, 1) = original_error;
        errors(i, 2) = finalerror;
        
%         % If the optimization went horribly wrong, just stick with the
%         % motion model estimate.
%         if (abs(xfinal(1)) > 1.5*abs(x0))
%             xfinal = x0;
%         end
        
        % Compute transform from optimized rotation and translation.
        T(1,4) = xfinal(1);
        T(2:3, 2:3) = [cos(xfinal(2)) -sin(xfinal(2)); sin(xfinal(2)) cos(xfinal(2))];
        
        % Calculate the transform to the head.
        H{i+1} = H{i} * T;
        
        if (mod(i, 50) == 1)
            disp(i);
        end
    end

end

