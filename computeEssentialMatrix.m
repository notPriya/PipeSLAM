function [H, E, F] = computeEssentialMatrix(x1, x2)
    % This computes the fundamental matrix. We need to get determine the
    % camera's internal calibration matrices in order to convert to E 
    % (E = K1' * F * K1).
    [F, inliers] = estimateFundamentalMatrix(x1, x2, 'Method', 'RANSAC', 'NumTrials', 2000);

    % Random estimate of the K matrix.
    K = [526.37013657 0 313.68782938; 0 526.37013657 259.01834898; 0 0 1];
    E = K'*F*K;
    
    % Decompose E into a rotation and translation components.
    M = decomposeE(E);
    
    % Rescale translation into meters and put it into a transformation
    % matrix format.
    M(:, 4, :) = M(:, 4, :)/10;
    if (trace(M(:, 1:3, 2)) > trace(M(:, 1:3, 4)))
        H = [M(:, :, 2); 0 0 0 1];
    else
        H = [M(:, :, 4); 0 0 0 1];
    end
end

function M2s = decomposeE(E)
% Enforce that E only has 2 eigenvalues and that they are the same one.
[U,S,V] = svd(E);
m = (S(1,1)+S(2,2))/2;
E = U*[m,0,0;0,m,0;0,0,0]*V';

[U,S,V] = svd(E);
W = [0,-1,0;1,0,0;0,0,1];

% Make sure we return rotation matrices with det(R) == 1
if (det(U*W*V')<0)
    W = -W;
end

M2s = zeros(3,4,4);
M2s(:,:,1) = [U*W*V',U(:,3)];
M2s(:,:,2) = [U*W*V',-U(:,3)];
M2s(:,:,3) = [U*W'*V',U(:,3)];
M2s(:,:,4) = [U*W'*V',-U(:,3)];
end
