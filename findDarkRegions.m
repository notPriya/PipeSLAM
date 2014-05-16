% Finds the darkest region in the image. This should correspond to
% regions that are far downstream the pipe.
function ind = findDarkRegions(I)
    % Constant for deviation from mean to count as being "dark" enough.
    k1 = 2;  % standard devations below mean.
    k2 = .25;  % standard devations above min value.

    % Convert image to grayscale.
    gray = rgb2gray(I(2:end-2, 2:end-2, :));

    % Compute some statistics on the image.
    thresh = mean(gray(:));
    dev = std(double(gray(:)));
    min_val = min(min(gray));
        
    % Find those beyond k1 std deviations. Sometimes this is below the
    % minimum value, so cap it with k2 standard deviations from the min
    % value.
    ind = find(gray < max(thresh-k1*dev, min_val+k2*dev));

    % Create a binary image of the dark pixels.
    BW = zeros(size(gray));
    BW(ind) = 1;
    
    % Find the connected components in the binary image.
    CC = bwconncomp(BW, 4);
    
    % Count the number of pixels in each connected component.
    numPixels = cellfun(@numel,CC.PixelIdxList);
    
    % Find the biggest connected component.
    [~,idx] = max(numPixels);
    
    % Return the indices of the largest connected component.
    [i j] = ind2sub(CC.ImageSize, CC.PixelIdxList{idx});
    ind = [j i];
    
%     % Visualize for debugging.
%     % TODO: remove.
%     for i=1:size(ind, 1)
%         I(ind(i, 2), ind(i, 1), :) = 0;
%     end
%     imshow(I);

end