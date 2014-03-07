%% Pipeline for doing bundle adjustment.
clear all;
close all;
clc;

load('pipe1.mat');

% Get the odometry bounding box.
imshow(frames(:,:,:,2));
[x, y] = ginput(2);
odom_rect = [x' y'];
close;

start_frame = 10;

%% Process all the images
im1 = preprocessImage(frames(:,:,:,start_frame));
k1 = calculateFeatures(im1, odom_rect);
d1 = calculateDescriptors(im1, k1, frames(:,:,:,2));

im2 = preprocessImage(frames(:,:,:,start_frame+1));
k2 = calculateFeatures(im2, odom_rect);
d2 = calculateDescriptors(im2, k2, frames(:,:,:,3));

im3 = preprocessImage(frames(:,:,:,start_frame+2));
k3 = calculateFeatures(im3, odom_rect);
d3 = calculateDescriptors(im3, k3, frames(:,:,:,4));

im4 = preprocessImage(frames(:,:,:,start_frame+3));
k4 = calculateFeatures(im4, odom_rect);
d4 = calculateDescriptors(im4, k4, frames(:,:,:,5));


%% Compute matches for every sequential pair of images.
matches1 = matchSIFT(d1', d2', 0.85, 3);
matches2 = matchSIFT(d2', d3', 0.85, 3);
matches3 = matchSIFT(d3', d4', 0.85, 3);

%% Compute matches of matches for every non-sequential pair of images.
same2 = ismember(matches2(1, :), matches1(2, :));
values = matches2(1, same2);
same2_rev = zeros(size(values));
for i=1:size(values,2)
    same2_rev(i) = find(matches1(2, :) == values(i), 1, 'first');
end

same3 = ismember(matches3(1, :), matches2(2, :));
values = matches3(1, same3);
same3_rev = zeros(size(values));
for i=1:size(values,2)
    same3_rev(i) = find(matches2(2, :) == values(i), 1, 'first');
end

%% Create the list of 3D points.
% Compute the transformation of subsequent cameras from the first camera image.
H12 = computeEssentialMatrix(k1(matches1(1,:), 1:2), k2(matches1(2, :), 1:2));
H13 = computeEssentialMatrix(k2(matches2(1,:), 1:2), k3(matches2(2, :), 1:2));
H14 = computeEssentialMatrix(k3(matches3(1,:), 1:2), k4(matches3(2, :), 1:2));

f = 526.37013657;
K = [f 0 313.68782938; 0 f 259.01834898; 0 0 1];


% Triangulate every group of matches. This has repeated 3D points. Ignore
% the second occurance of any repeated point.
P1 = triangulate(K*eye(3, 4), k1(matches1(1,:), 1:2), K*H12(1:3, :), k2(matches1(2, :), 1:2));
P2 = triangulate(K*H12(1:3, :), k2(matches2(1,:), 1:2), K*H13(1:3, :), k3(matches2(2, :), 1:2));
P3 = triangulate(K*H13(1:3, :), k3(matches3(1,:), 1:2), K*H14(1:3, :), k4(matches3(2, :), 1:2));

% This has size n1 + w2 + w3.
P = [P1; P2(~same2, :); P3(~same3, :)];

%% Create the list of observations.
n1 = size(matches1, 2);
w2 = sum(same2 == 0);
n2 = size(matches2, 2);
w3 = sum(same3 == 0);
n3 = size(matches3, 2);

% Index of P1 in the P array.
P_ind1 = 1:n1;

% Index of P2 in the P array.
P_ind2 = zeros(1, n2);
P_ind2(same2) = P_ind1(same2_rev);
P_ind2(~same2) = n1+1:n1+w2;

% Index of P3 in the P array.
P_ind3 = zeros(1, n3);
P_ind3(same3) = P_ind2(same3_rev);
P_ind3(~same3) = n1+w2+1:n1+w2+w3;


O1 = [zeros(n1, 1) P_ind1' k1(matches1(1, :), 1:2)];

O2 = [ones(n1+w2, 1) [1:n1 n1+1:n1+w2]' [k2(matches1(2, :), 1:2); k2(matches2(1, ~same2), 1:2)] ];

O3 = [2*ones(n2+w3, 1) [P_ind2 n1+w2+1:n1+w2+w3]' [k3(matches2(2, :), 1:2); k3(matches3(1, ~same3), 1:2)] ];

O4 = [3*ones(n3, 1) P_ind3' k4(matches3(2, :), 1:2) ];

O = [O1; O2; O3; O4];

%% Output results to a file for bundle adjustment

fid = fopen('pipeline_results.txt', 'w');
fprintf(fid, '%d %d %d\n', 4, size(P, 1), size(O, 1));

% Print the observations.
fprintf(fid, '%d %d %f %f\n', O');

% Print the camera matrices.
k1 = 0;
k2 = 0;
fprintf(fid, '%f\n', [0 0 0 0 0 0 f k1 k2]);
fprintf(fid, '%f\n', [HtoAxisAngle(H12) H12(1:3, 4)' f k1 k2]);
fprintf(fid, '%f\n', [HtoAxisAngle(H13) H13(1:3, 4)' f k1 k2]);
fprintf(fid, '%f\n', [HtoAxisAngle(H14) H14(1:3, 4)' f k1 k2]);

% Print the points.
fprintf(fid, '%f\n', P(:, 1:3)');

fclose(fid);

%% Call the bundle adjustment function.

! ../ceres-solver-1.8.0/bundle_adjustment.sh

fid = fopen('solver_log.txt');

fseek(fid, -100, 'eof');
data = fgets(fid);
disp(data);
fclose(fid);

% Read in results from bundle adjustment.

fid = fopen('solver_results.txt');
data = fscanf(fid, '%f\n', Inf);

% Extract the camera values from the data.
c1 = data(1:6);
c2 = data(7:12);
c3 = data(13:18);
c4 = data(19:24);

% Extract the points from the data.
P = data(25:end);
P = reshape(P, 3, size(P, 1)/3)';

% Get the transformation matrices.
H1 = AxisAngletoH(c1);
H2 = AxisAngletoH(c2);
H3 = AxisAngletoH(c3);
H4 = AxisAngletoH(c4);

% Put them in the frame of the first camera.
H1_inv = [H1(1:3, 1:3)' -H1(1:3, 1:3)'*H1(1:3, 4)];
H12n = H2*[H1_inv; 0 0 0 1];
H13n = H3*[H1_inv; 0 0 0 1];
H14n = H4*[H1_inv; 0 0 0 1];

% Plot the cameras.

figure;
hold on;
axis equal;
% plot3(P(:, 1), P(:, 2), P(:, 3), 'b.');
colors = ['r' 'g' 'b'];
% axis([-5 5 -5 5 -5 5]);

cameras = zeros(3, 4, 4);
cameras(:,:,1) = eye(3, 4);
cameras(:,:,2) = H12n;
cameras(:,:,3) = H13n;
cameras(:,:,4) = H14n;

% Plot the new cameras.
for i = 1:4
    o = cameras(:,4,i);
    for j = 1:3
        caxis = [o o+1*cameras(:,j,i)];
        plot3(caxis(1, :), caxis(2, :), caxis(3, :), colors(j), 'LineWidth', 2);
    end
    plot3(o(1), o(2), o(3), 'bx', 'MarkerSize', 10);
end

cameras(:,:,1) = eye(3, 4);
cameras(:,:,2) = H12(1:3, :);
cameras(:,:,3) = H13(1:3, :);
cameras(:,:,4) = H14(1:3, :);


% Plot the old cameras.
for i = 1:4
    o = cameras(:,4,i);
    for j = 1:3
        caxis = [o o+1*cameras(:,j,i)];
        plot3(caxis(1, :), caxis(2, :), caxis(3, :), colors(j), 'LineWidth', 2);
    end
    plot3(o(1), o(2), o(3), 'ro', 'MarkerSize', 10);
end