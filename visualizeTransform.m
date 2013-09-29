% Computes the transform between one frame and the next and visualizes the
% results.
function visualizeTransform(I1, I2)
    % Calculate the transform between I1 and I2.
    H = calculateTransform(I1, I2);

    disp(H);
end