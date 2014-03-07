%% Gets the pipe joint training data to learn the weights.
% close all;
% clear all;
% 
% % Load the frames.
% load('pipe1.mat');

% trainx = [];
% trainy = [];
%
% Accuracy 4/14
% Accuracy 4/14
% Accuracy 4/14

for i=4:50:size(frames, 4)  
    I = frames(17:344, 10:430, :, i);
    
    % Get the values carried forward from the previous timestep.
    [c, r, p] = visualizePipeJoints(frames(17:344, 10:430, :, i-1));
    
    % Get the circles and corresponding features for this image.
    [c, r, ~, f] = visualizePipeJoints(I, c, r, p);
    
    % Ask the user for the labels
    labels = zeros(size(r));
    for j=1:size(r, 1)
        imshow(I);
        viscircles(c(j, :), r(j), 'EdgeColor', 'b');
        
        meow = input('Correct?  ');
        while isempty(meow)
            meow = input('Correct?  ');
        end
        
        labels(j) = meow;
    end
    
    labels(labels == 2) = -1;
    
    % Add the examples to the training set.
    trainx = [trainx; f];
    trainy = [trainy; labels];
end

%% Learn some weights.
mdl = svmtrain(trainy, trainx, '-t 2');
weights = mdl.sv_coef' * mdl.SVs;
svmpredict(trainy, trainx, mdl);