function [H12, H13, H14] = calculateBATransform(k1, d1, k2, d2, k3, d3, k4, d4, matches1, matches2, matches3, H12, H13, H14, meow)
    %%%%
    % Compute matches of matches for every non-sequential pair of images.
    %%%%
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

    %%%%
    % Create the list of 3D points.
    %%%%
    f = 526.37013657;
    K = [f 0 313.68782938; 0 f 259.01834898; 0 0 1];

    % Triangulate every group of matches. This has repeated 3D points. Ignore
    % the second occurance of any repeated point.
    P1 = triangulate(K*eye(3, 4), k1(matches1(1,:), 1:2), K*H12(1:3, :), k2(matches1(2, :), 1:2));
    P2 = triangulate(K*H12(1:3, :), k2(matches2(1,:), 1:2), K*H13(1:3, :), k3(matches2(2, :), 1:2));
    P3 = triangulate(K*H13(1:3, :), k3(matches3(1,:), 1:2), K*H14(1:3, :), k4(matches3(2, :), 1:2));

    % Final list of 3D points.
    % This has size n1 + w2 + w3.
    P = [P1; P2(~same2, :); P3(~same3, :)];

    %%%%
    % Create the list of observations.
    %%%%
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


    % List the observations for each image.
    O1 = [zeros(n1, 1) P_ind1' k1(matches1(1, :), 1:2)];

    O2 = [ones(n1+w2, 1) [1:n1 n1+1:n1+w2]' [k2(matches1(2, :), 1:2); k2(matches2(1, ~same2), 1:2)] ];

    O3 = [2*ones(n2+w3, 1) [P_ind2 n1+w2+1:n1+w2+w3]' [k3(matches2(2, :), 1:2); k3(matches3(1, ~same3), 1:2)] ];

    O4 = [3*ones(n3, 1) P_ind3' k4(matches3(2, :), 1:2) ];

    % Combined list of observations.
    O = [O1; O2; O3; O4];

    %%%%
    % Output results to a file for bundle adjustment
    %%%%

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

    %%%%
    % Run the bundle adjustment script.
    %%%%
    ! ../ceres-solver-1.8.0/bundle_adjustment.sh    
    
    %%%%
    % Read in results from bundle adjustment.
    %%%%

    fid = fopen('solver_results.txt');
    data = fscanf(fid, '%f\n', Inf);

    % Extract the camera values from the data.
    c1 = data(1:6);
    c2 = data(7:12);
    c3 = data(13:18);
    c4 = data(19:24);

    % Get the transformation matrices.
    H1 = AxisAngletoH(c1);
    H2 = AxisAngletoH(c2);
    H3 = AxisAngletoH(c3);
    H4 = AxisAngletoH(c4);

    % Put them in the frame of the first camera.
    H1_inv = [H1(1:3, 1:3)' -H1(1:3, 1:3)'*H1(1:3, 4)];
%     H12 = H2*[H1_inv; 0 0 0 1];
%     H13 = H3*[H1_inv; 0 0 0 1];
%     H14 = H4*[H1_inv; 0 0 0 1];
    H12 = H1_inv*[H2; 0 0 0 1];
    H13 = H1_inv*[H3; 0 0 0 1];
    H14 = H1_inv*[H4; 0 0 0 1];

end