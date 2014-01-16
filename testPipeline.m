%% Script to test the pipeline.
% clear all;
close all;

% Create a circle.
t = 0:.1:2*pi+.1;
P = [5*sin(t); 5*cos(t); -3*ones(size(t))];

% Intrinsic matrix.
% K = [526.37013657 0 313.68782938; 0 526.37013657 259.01834898; 0 0 1];
K = [526.37013657 0 0; 0 526.37013657 0; 0 0 1];
f = 526.37013657;
k1 = 0;
k2 = 0;


% Create extrinsic matrices. (Camera to World).
deg = -20;
deg2 = -10;
M1 = eye(3, 4);
R = [1 0 0; 0 cosd(deg) -sind(deg); 0 sind(deg) cosd(deg)]*[cosd(deg2) -sind(deg2) 0; sind(deg2) cosd(deg2) 0; 0 0 1];
t = [2; 2; 2];
C2 = [R t]; % Location of camera.
M2 = [R' -R'*t];

% Create the simulated points from the first camera.
p1 = M1*[P; ones(1, 64)];
% Normalize so the third coordinate is 1.
p1 = -p1./p1(3, 3);
% Radial distortion
len_p1 = sum(p1.^2);
len_p1 = [len_p1; len_p1; zeros(1, 64)];
p1 = f*(1 + k1*len_p1 + k2*len_p1.^2).*p1;
p1(3, :) = ones(1, 64);


% Create the simulated points from the second camera.
p2 = M2*[P; ones(1, 64)];
% Normalize so the third coordinate is 1.
p2 = -p2./p2(3, 3);
% Radial distortion
len_p2 = sum(p2.^2);
len_p2 = [len_p2; len_p2; zeros(1, 64)];
p2 = f*(1 + k1*len_p2 + k2*len_p2.^2).*p2;
p2(3, :) = ones(1, 64);


% Add noise to the observed points.
p1(1:2, :) = p1(1:2, :) + 5*randn(2, 64);
p2(1:2, :) = p2(1:2, :) + randn(2, 64);

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

%% Output results to a file for bundle adjustment

fid = fopen('test_pipeline_results.txt', 'w');
fprintf(fid, '%d %d %d\n', 2, size(P, 2), 2*size(P, 2));

% Print the observations.
fprintf(fid, '%d %d %f %f\n', [zeros(1, size(p1, 2)); 0:size(p1, 2)-1; p1(1, :); p1(2, :)]);
fprintf(fid, '%d %d %f %f\n', [ones(1, size(p2, 2)); 0:size(p2, 2)-1; p2(1, :); p2(2, :)]);

% Calculate the Rodrigues vector for the second camera.
theta = acos((trace(H(1:3, 1:3))-1)/2);
v = 1/(2*sin(theta)) * [H(3, 2)-H(2, 3); H(1, 3)-H(3, 1); H(2, 1)-H(1, 2)];
v = v*theta;

% Print the camera matrices.
fprintf(fid, '%f\n', [0 0 0 0 0 0 f k1 k2]);
fprintf(fid, '%f\n', [v(1) v(2) v(3) H(1, 4) H(2, 4), H(3, 4) f k1 k2]);

% Print the points.
fprintf(fid, '%f\n', P2(:, 1:3)');

fclose(fid);


%% Read in and plot results from bundle adjustment

fid = fopen('solver_result_points.txt');
data = fscanf(fid, '%f\n', Inf);
R = data(1:3);
T = data(4:6);
P3 = data(7:end);

% Reshape the bundle-adjusted points and scale.
P3 = reshape(P3, 3, size(P3, 1)/3)';
P3 = P3;

% Build the transformation matrix for the second camera.
theta = norm(R);
E = R/theta;
Ex = [0 -E(3) E(2); E(3) 0 -E(1); -E(2) E(1) 0];
R = eye(3)*cos(theta) + (1 - cos(theta))*E*E' + Ex*sin(theta);
C = [R T];

figure;
hold on;
axis equal;
plot3(P3(:, 1), P3(:, 2), P3(:, 3), 'b.--');
colors = ['r' 'g' 'b'];
% Plot the first camera.
o = M1(:, 4);
for i=1:3
    caxis = [o o+1*M1(:, i)];
    plot3(caxis(1, :), caxis(2, :), caxis(3, :), colors(i), 'LineWidth', 2);
end
plot3(o(1), o(2), o(3), 'bx', 'MarkerSize', 10);
% Plot the second camera.
o = C(:, 4);
for i=1:3
    caxis = [o o+1*C(:, i)];
    plot3(caxis(1, :), caxis(2, :), caxis(3, :), colors(i), 'LineWidth', 2);
end
plot3(o(1), o(2), o(3), 'ro', 'MarkerSize', 10);

