function [new_weights, fval, exitflag, output] = learnTrackingWeights()
clc;

    % The initial set of weights to seed the optimization.
    initial_weights = [1.18965619416973,0.773250960852945,0,3.44340934195353,6.44340934195353]';
    
    % Set of frames to optimize over.
    load('pipe1.mat');
    
    % The initial circle to start off the tracker.
    initial_state = [196, 182, 69, 0, 0, 0];
    
    % Start optimization.
    options = optimset('MaxFunEvals', 1000, 'MaxIter', 100000, 'TolX', 0.01, 'Display', 'off', 'PlotFcns', @optimplotfval);
    [new_weights, fval, exitflag, output] = fminsearch(@(x) pipeJointTracking(x, initial_state, frames), initial_weights, options);
    
    disp(new_weights);

end