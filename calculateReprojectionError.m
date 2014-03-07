%% Function that minimizes reprojection error.
function error = calculateReprojectionError(x, T, H_old, H_new, p1, p2, show_plots)
    % Calculate the updated transform.
    T(1, 4) = x(1);
    if length(x) > 1
        T(2:3, 2:3) = [cos(x(2)) -sin(x(2)); sin(x(2)) cos(x(2))];
    end
    
    % Put it in the frame of the old and new heads.
    H_old_inv = [H_old(1:3, 1:3)' -H_old(1:3, 1:3)'*H_old(1:3, 4); 0 0 0 1];
    T = H_new*T*H_old_inv;
    
    if( sum(sum(abs(T - eye(4)))) < 0.0001 )
        % TODO Put this error in the same terms as the error below.
        error = mynorm(p1, p2) + mynorm(p2, p1);
        return;
    end
    
    % Calculate the essential matrix.
    E = calculateE(T);
    E_inv = E';
    
    % Calculate the reprojected points.
    n = size(p1, 1);
    
    p2_hat = [p2 ones(n, 1)]*E*[p1'; ones(1, n)];
    p1_hat = [p1 ones(n, 1)]*E_inv*[p2'; ones(1, n)];
    
    error = trace(abs(p2_hat))+trace(abs(p1_hat));
    
%     % Normalization Hack.
%     if (T(3,4) > 0)
%         p1_hat = normalize2Dpoints(p1_hat(1:2, :));
%         p2_hat = normalize2Dpoints(p2_hat(1:2, :));
%         p1_hat(1:2, :) = [p1_hat(2, :); -p1_hat(1, :)];
%         p2_hat(1:2, :) = [-p2_hat(2, :); p2_hat(1, :)];
%         [~, T1] = normalize2Dpoints(p1');
%         [~, T2] = normalize2Dpoints(p2');
%         p1_hat = T1\p1_hat;
%         p2_hat = T2\p2_hat;
%     elseif (T(3,4) < 0)
%         p1_hat = normalize2Dpoints(p1_hat(1:2, :));
%         p2_hat = normalize2Dpoints(p2_hat(1:2, :));
%         p1_hat(1:2, :) = [p1_hat(2, :); -p1_hat(1, :)];
%         p2_hat(1:2, :) = [-p2_hat(2, :); p2_hat(1, :)];
%         [~, T1] = normalize2Dpoints(p1');
%         [~, T2] = normalize2Dpoints(p2');
%         p1_hat = T1\p1_hat;
%         p2_hat = T2\p2_hat;
%     end
%     
%     if show_plots
%         figure();
%         subplot(2, 1, 1);
%         hold on;    
%         plot3(p1_hat(1, :), p1_hat(2,:), p1_hat(3,:), 'r.');
%         plot3(p1(:, 1), p1(:, 2), zeros(1, n), 'bx');
% 
%         subplot(2, 1, 2);
%         hold on;
%         plot3(p2(:,1), p2(:,2), zeros(1, n), 'bx');
%         plot3(p2_hat(1, :), p2_hat(2, :), p1_hat(3,:), 'r.');
%         
%         figure();
%         hold on;
%         meow = p1 - p1_hat(1:2,:)';
%         meow2 = (abs(meow(:,1)) > 2) | (abs(meow(:,2)) > 2);
%         plot(p1(meow2, 1), p1(meow2, 2), 'r.');
%         
%     end
%         
%     % Calculate number of outliers.
%     pixel_error1 = abs(p1 - p1_hat(1:2,:)');
%     pixel_error2 = abs(p2 - p2_hat(1:2,:)');
%     error = (sum((pixel_error1(:,1) > 2) | (pixel_error1(:,2) > 2)) + ...
%             sum((pixel_error2(:,1) > 2) | (pixel_error2(:,2) > 2)))/(2*size(p1, 1));
    
%     % Calculate the reprojection error.
%     error = mynorm(p1, p1_hat(1:2, :)') + mynorm(p2, p2_hat(1:2, :)');
    
    function E = calculateE(T)
        E = T(1:3, 1:3)*[0 -T(3,4) T(2,4); T(3,4) 0 -T(1,4); -T(2,4) T(1,4) 0];
    end
    
    function norm = mynorm(p1, p2)
        norm = sum(sum((p1-p2).^2, 2));
    end
end
