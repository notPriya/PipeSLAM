function F = eightpoint(pts1, pts2, M)
    % Calculate transform
    T = [1/M 0 0; 0 1/M 0; 0 0 1];

    % Scale Points
    p1 = T * [pts1 ones(size(pts1, 1), 1)]';
    p2 = T * [pts2 ones(size(pts2, 1), 1)]';
    
    % Compute U from the correspondences
    U = [];
    for i=1:size(p1, 2)
        u = p1(:, i) * p2(:, i)';
        U = [U; u(:)'];
    end
    
    % Compute the eigenvector corresponding to the smallest
    % eigenvalue
    [~, ~, V] = svd(U);
    
    % Compute F
    f = V(:, end);
    F = reshape(f, 3, 3)';
    
    % Ensure F is singular
    [W D V] = svd(F);
    D = [1 0 0; 0 1 0; 0 0 0] * D;
    F = W*D*V';

    % Refine F
    F = refineF(F, p1', p2');
    
    % Unscale F
    F = T' * F * T;
    
end