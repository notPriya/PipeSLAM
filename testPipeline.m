%% Script to test the pipeline.
clear all;
close all;

% Create a circle.
t = 0:.1:2*pi+.1;
P = [5*sin(t); 5*cos(t); -3*ones(size(t))];

% Intrinsic matrix.
K = [526.37013657 0 313.68782938; 0 526.37013657 259.01834898; 0 0 1];

% Create extrinsic matrices. (Camera to World).
deg = 20;
deg2 = 10;
M1 = eye(3, 4);
R = [1 0 0; 0 cosd(deg) -sind(deg); 0 sind(deg) cosd(deg)]*[cosd(deg2) -sind(deg2) 0; sind(deg2) cosd(deg2) 0; 0 0 1];
t = [2; 2; 5];
C2 = [R t]; % Location of camera.
M2 = [R' -R'*t];

% Create the simulated points from the first camera.
p1 = K*M1*[P; ones(1, 64)];
% Normalize so the third coordinate is 1.
p1 = p1./p1(3, 3);

% Create the simulated points from the second camera.
p2 = K*M2*[P; ones(1, 64)];
% Normalize so the third coordinate is 1.
p2 = p2./p2(3, 3);

figure;
hold on;
plot(p1(1, :), p1(2, :), 'b', 'LineWidth', 2);
plot(p2(1, :), p2(2, :), 'r', 'LineWidth', 2);

%% Test the function.
% Derive the rotation and translation of the camera given the point
% correspondences.
[H, E, F] = computeEssentialMatrix(p1(1:2, :)', p2(1:2, :)');

norm(H(1:3, 1:3) - M2(:, 1:3), 'fro')

%% Evaluate the function:
% Estimate scale.
H(1:3, 4) = H(1:3, 4) * 30;

% Calculate the 3D points that correspond to the given H matrix.
P2 = triangulate(K*eye(3, 4), p1', K*H(1:3, :), p2');

%% Visualize the generated 3D pointcloud.
figure;
hold on;
axis equal;
plot3(P2(:, 1), P2(:, 2), P2(:, 3), 'b.--');
axis([-5 5 -5 5 -5 5]);
colors = ['r' 'g' 'b'];
% Plot the first camera.
o = M1(:, 4);
for i=1:3
    caxis = [o o+1*M1(:, i)];
    plot3(caxis(1, :), caxis(2, :), caxis(3, :), colors(i), 'LineWidth', 2);
end
plot3(o(1), o(2), o(3), 'bx', 'MarkerSize', 10);
% Plot the second camera.
o = C2(:, 4);
for i=1:3
    caxis = [o o+1*C2(:, i)];
    plot3(caxis(1, :), caxis(2, :), caxis(3, :), colors(i), 'LineWidth', 2);
end
plot3(o(1), o(2), o(3), 'ro', 'MarkerSize', 10);
