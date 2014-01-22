% Preprocess the features, descriptors, and matches for bundle adjustment.
% Then calculates the transforms to the first frame.
function [K, D, M, H] = preprocessBAData(frames, odom_rect, n)
    disp('Preprocessing the features and descriptors');
    
    % Preprocess the features and descriptors.
    K = cell(n, 1);
    D = cell(n, 1);
    for i = 1:n
        % Preprocess the image.
        I = frames(:,:,:,i);
        im = preprocessImage(I);
        
        % Calculate the features.
        k1 = calculateFeatures(im, odom_rect);
        
        % Calculate the descriptors.
        d1 = calculateDescriptors(im, k1, I);
        
        % Save off the results.
        K{i} = k1;
        D{i} = d1;
        
        if (mod(i, 100) == 1)
            disp(i);
        end
    end
    
    disp('Preprocessing the matches and transforms');
    
    % Preprocess the matches and transforms.
    M = cell(n-1, 1);
    H = cell(n, 1);
    H{1} = eye(4, 4);
    for i=1:n-1
        % Compute matches
        matches = matchSIFT(D{i}', D{i+1}', 0.85, 3);
        M{i} = matches;

        % Extract the matched points.
        p1 = K{i}(matches(1, :), 1:2);
        p2 = K{i+1}(matches(2, :), 1:2);

        % Compute the transform.
        T = computeEssentialMatrix(p1, p2);
        
        % Calculate the transform to the head.
        H{i+1} = H{i} * T;
        
        if (mod(i, 100) == 1)
            disp(i);
        end
    end

end

