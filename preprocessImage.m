% Does all preprocessing of the image.
% Converts the image to grayscale and normalizes the pixel intensity
% distribution from 0 to 255.
function im = preprocessImage(I)
    % Gray scale the image.
    im = rgb2gray(I);
    
    % Normalize the pixel intensity distribution.
    max_val = max(max(im));
    min_val = min(min(im));
    
    im = double(im - min_val)* 255/double(max_val - min_val);
    im = uint8(im);
end