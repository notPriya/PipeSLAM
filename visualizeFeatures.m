function visualizeFeatures(I, odom_rect)
    if (nargin < 2)
        odom_rect = zeros(1, 4);
    end

     I = preprocessImage(I);
     k = calculateFeatures(I, odom_rect);
     
     imshow(I);
     hold on;
     plot(k(1:min(500, length(k)), 1), k(1:min(500, length(k)), 2), 'rx', 'LineWidth', 2);
end

