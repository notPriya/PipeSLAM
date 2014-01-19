% Manually input ground truth distance traveled for each frame.
% Input: Top left coordinates, bottom right coordinates.
function ground_truth = manualGroundTruth(frames, ground_truth, box)

    if nargin < 3
        % Determine the boundingbox.
        imshow(frames(:,:,:,1));
        [x, y] = ginput(2);
        box = int32([y x]);
        close;
    end
    
    % Initialize loop variables.
    %     ground_truth = zeros(size(frames, 4), 1);
    old_dist = NaN;
    
    for num_frame=3177:5:size(frames, 4)
        % Get I from the frames.
        I = frames(:, :, :, num_frame);
        
        % Extract only the bounding box.
        I = I(box(1, 1):box(2,1), box(1,2):box(2,2), :);

        % Show the image and get the distance.
        imshow(I);
        dist = input('Distance: ');
        while isempty(dist)  % Incase we dont input a distance.
            dist = input('Distance: ');
        end
        
        % If it is the first time around, set the groundtruth for only this
        % frame.
        if isnan(old_dist)
            ground_truth(num_frame) = dist;
        elseif old_dist == dist
        % Mark it as the same distance for the last 5 frames.
            ground_truth(num_frame-4:num_frame) = dist;
        else
        % Iterate through the intermediary frames one by one.
            ground_truth(num_frame) = dist;
            for i=num_frame-4:num_frame-1
                % Get I from the frames.
                I = frames(:, :, :, i);

                % Extract only the bounding box.
                I = I(box(1, 1):box(2,1), box(1,2):box(2,2), :);

                % Show the image and get the distance.
                imshow(I);
                temp_dist = input('Distance: ');
                while isempty(temp_dist)
                    temp_dist = input('Distance: ');
                end
                ground_truth(i) = temp_dist;
            end
        end
        
        % Update the old distance for next iteration.
        old_dist = dist;
        
        close all;
    end
end

