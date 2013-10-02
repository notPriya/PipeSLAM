function [x2, y2] = epipolarCorrespondence(im1, im2, F, x1, y1, T)
    % Implementation Variables
    window_size = 31;
       
    % Compute Epipolar line
    l = F * [x1; y1; 1];
    
    % Normalize l
    l = l/(sqrt(l(1)^2 + l(2)^2));
    
    % Compute the points along the epipolar line
    if (abs(l(1)/l(2)) > 1) % If the slope is greater than 1 (vertical)
        ly = (y1-25):(y1+25);
        % by + c = -a x
        % x = -1/l(1)*(l(2)*y + l(3))
        lx = -(l(2).*ly + l(3))./l(1);
    else
        lx = (x1-25):(x1+25);
        % y = -1/l(2)*(l(1)*x + l(3))
        ly = -(l(1).*lx + l(3))./l(2);
    end
    
    % Find the indices of ly that are in bounds
    index = find(ly > 0 & ly < size(im2, 1));
    
    % Only look at the ly and lx that are in bounds.
    ly = ly(index);
    lx = lx(index);
    
    % Find the closest point in the epipolar line
    min_dist = NaN;
    x2 = 0;
    y2 = 0;
    for i=1:length(lx)
        % Compute a window around the new coordinates
        bounds = [max((ly(i)-window_size/2), 1); ...
            min((ly(i)+window_size/2), size(im2, 1)); ...
            max((lx(i)-window_size/2), 1); ...
            min((lx(i)+window_size/2), size(im2, 2))];
        test_win = im2(round(bounds(1):bounds(2)), round(bounds(3):bounds(4)));
        
        % Adjust the template window to match size of test window
        rect = [bounds(2)-bounds(1) bounds(4)-bounds(3)];
        win = im1(round((y1-rect(1)/2):(y1+rect(1)/2)), round((x1-rect(2)/2):(x1+rect(2)/2)));

%         % Visualize matching process.
%         subplot(1, 2, 1);
%         imshow(win);
%         subplot(1, 2, 2);
%         imshow(test_win);
%         pause(.3);
        
        % Compare window similarity by taking the distance between
        % every pair of pixels in win and test_win, but only summing
        % those values that are on the diagonal since those
        % correspond to the same pixels
        dist = pdist2(double(win(:)), double(test_win(:)));
        dist = trace(dist);
        
        % if we found the minimum distance so far, update x2 and y2
        if (isnan(min_dist)|| dist < min_dist)
            min_dist = dist;
            x2 = lx(i);
            y2 = ly(i);
        end
    end
    
end