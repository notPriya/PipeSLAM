% Get I from the frames.
I = frames(:, :, :, 170);
imshow(I);

%%
close;

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

%%
n=4;

% Find the each component.
[i,j] = find(labels==n);
% Extract letter
n1 = first_line(min(i):max(i),min(j):max(j)); 

% Resize letter (same size of template)
n1 = imresize(n1,[42 24]);
imshow(n1);

%% Save
close;

imwrite(n1, 'letters_numbers/0_new.bmp', 'BMP');