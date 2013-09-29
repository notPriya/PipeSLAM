function F = sevenpoint(pts1, pts2, M)
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

    % Find eigenvectors of U^T*U
    [~, ~, V] = svd(U);

    % Compute two solutions to SVD
    f1 = V(:, end);
    f2 = V(:, end-1);

    F1 = reshape(f1, 3, 3)';
    F2 = reshape(f2, 3, 3)';

    % Solve solution of form F = (1-lambda)*F1 + lambda*F2
    % with constraint that Det(F) = 0
    l = sym('l', 'real');
    lambda = double(real(vpa(solve(det((1-l)*F1+l*F2)))));
    
    % For each value of lambda compute an F
    F = cell(length(lambda), 1);
    for i=1:length(lambda)
        % Compute F
        Fi = (1-lambda(i))*F1+lambda(i)*F2;
        
        % Refine F
        Fi = refineF(Fi, p1', p2');

        % Unscale F
        F{i} = T' * Fi * T;
    end
    
end