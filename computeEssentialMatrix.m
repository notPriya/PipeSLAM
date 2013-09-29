function [H, E] = computeEssentialMatrix(x1, x2)
    % This computes the fundamental matrix. We need to get determine the
    % camera's internal calibration matrices in order to convert to E 
    % (E = K1' * F * K1). However for now, we will just use F as E.
    [F, inliers] = estimateFundamentalMatrix(x1, x2, 'Method', 'RANSAC');
    
    % Random estimate of the K matrix.
%     K = [645.24 0 661.96; 0 645.24 194.13; 0 0 1];
    K = eye(3);
    E = K'*F*K;
    
    % Decompose E into a rotation and translation components.
    M = decomposeE(E);
    
    % Compute relative scale are rescale the translation matrix.
    H = [M(:, :, 1); 0 0 0 1];
end

function M2s = decomposeE(E)
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
M2s(:,:,1) = [U*W*V',U(:,3)./max(abs(U(:,3)))];
M2s(:,:,2) = [U*W*V',-U(:,3)./max(abs(U(:,3)))];
M2s(:,:,3) = [U*W'*V',U(:,3)./max(abs(U(:,3)))];
M2s(:,:,4) = [U*W'*V',-U(:,3)./max(abs(U(:,3)))];
end
