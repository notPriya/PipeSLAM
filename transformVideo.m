clear all;

% Read in the video.
obj = VideoReader('pipe1.mp4');
frames = read(obj);

num_frames = obj.NumberOfFrames-1;

% Calculate the transform from one frame to the next.
H = cell(num_frames, 1);
for i=1:num_frames
    I = frames(:,:,:,i);
    I2 = frames(:,:,:,i+1);
    H2to1 = calculateTransform(I, I2);
    H2to1 = H2to1./H2to1(3,3);
    H{i} = H2to1;
end

% Calculate the transform to frame 1.
Hto1 = cell(num_frames, 1);
Hto1{1} = H{1};
for i=2:num_frames
    Hto1{i} = Hto1{i-1} * H{i};
end

% Warp each frame into the first frame and write to the video.
mov = VideoWriter('pipe1_result.avi');
mov.FrameRate = 15;
open(mov);
writeVideo(mov, frames(:,:,:,1));
for i=2:num_frames
    I = frames(:,:,:,i);
    Ito1 = warpH(I, double(Hto1{i-1}), size(I), 3);
    writeVideo(mov, Ito1);
end
close(mov);