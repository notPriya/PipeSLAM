function P = triangulate(M1, p1, M2, p2)
    P = [];
    for i = 1:size(p1, 1)
        
        % Save off p1 and p2 for easy access
        v1 = p1(i, :);
        v2 = p2(i, :);
        
        % Calculate least squares error to algebraic reprojection error
        % Here we are computing |vi x MiP|
        A = [v1(2).*M1(3, :)-M1(2, :); M1(1, :) - v1(1).*M1(3, :); v1(1).*M1(2, :)-v1(2).*M1(1, :); ...
            v2(2).*M2(3, :)-M2(2, :); M2(1, :) - v2(1).*M2(3, :); v2(1).*M2(2, :)-v2(2).*M2(1, :)];

        % Vector that minimzies the solution to |vi x MiP| is the
        % minimum eigenvector of A^T A
        [~, ~, V] = svd(A);
        p = V(:, end)';

        % Normalize the current P
        p = p ./ p(4);

        % Save in vector of P's
        P = [P; p];
    end

end
        