% Performs OCR on the input bounding box frame by frame of the video and
% gets the ground truth distance traveled.
% Input: Top left coordinates, bottom right coordinates.
function ground_truth = ocrGroundTruth(frames, box)
    % Add the OCR directory for this task.
    addpath './OCR';
    
    if nargin < 2
        % Determine the boundingbox.
        imshow(frames(:,:,:,1));
        [x, y] = ginput(2);
        box = [y x];
    end
    
    % Load templates
    load('OCR/templates.mat');
    global templates
    num_letters=size(templates,2);

    ground_truth = [];
    for num_frame=1:size(frames, 4)
        % Get I from the frames.
        I = frames(:, :, :, num_frame);
        
        % Extract only the bounding box.
        I = I(box(1, 1):box(2,1), box(1,2):box(2,2), :);

        % Convert to gray scale
        im=rgb2gray(I);
        % Convert to BW
        threshold = graythresh(im);
        im = ~im2bw(im,threshold);
        % Remove all object containing fewer than 30 pixels
        im = bwareaopen(im,30);

        word = [];
        
        %Separate the text into individual lines. Since we have a bounding
        %box, only process the first line.
        [first_line, ~]=lines(im);

        % Label and count connected components
        [labels, num_components] = bwlabel(first_line);    
        for n=[2 4]
            % Find the each component.
            [i,j] = find(labels==n);
            % Extract letter
            n1 = first_line(min(i):max(i),min(j):max(j));  
            % Resize letter (same size of template)
            n1 = imresize(n1,[42 24]);
    imshow(n1);
            % Convert image to text.
            letter = read_letter(n1, num_letters);
    disp(letter);
            %Only add numbers to the the word.
            if double(letter) > 47 && double(letter) < 58
                % Concatenate the letter to the word.
                word=[word letter];
            end
        end
        ground_truth = [ground_truth; str2double(word)];
    end
end

