%David Fouhey
%Carnegie Mellon University
%16-720 Fall 2012
%
%Runs RANSAC and Sequential RANSAC and visualizes the results.
function pipeline(imageName1, imageName2, t, mode);
    %imageName1 - path to image1
    %imageName2 - path to image2   
    %t - threshold for SIFT matching
    %Mode - controls the output style
    %   1 => RANSAC, show matches
    %   2 => RANSAC, show dots
    %   3 => Sequential RANSAC, show dots

    if nargin < 4
        mode = 1;
    end
   
    %load keypoints
    [k1,d1] = loadKeypoints([imageName1 '.desc.mat']);
    [k2,d2] = loadKeypoints([imageName2 '.desc.mat']);

    %compute matches
    matches = matchsift(d1,d2,t);

    Hs = {}; inlierMatches = {};
    if(mode ~= 3)
        %run single-model RANSAC
        [H,inliers] = ransacH2to1(k1,k2,matches);
        Hs = {H};
        inlierMatches = {inliers};
    else
        %run sequential RANSAC
        [Hs, inlierMatches] = sequentialRANSAC(k1,k2, matches);
    end

    %make the for loop cleaner for visualization
    images = {rgb2gray(imread(imageName1)), rgb2gray(imread(imageName2))};
    keypoints = {k1,k2};

    %surely you won't need more than a few colors
    colors = {'r','g','b','y','c','m'};
    for i=1:5
        colors = [colors, colors];
    end

    %close any figures
    close all;

    if(mode > 1)
        %dot-style visualization
        for imageI=1:2
            subplot(120+imageI);
            imshow(images{imageI}); 
            hold on;
            for i=1:numel(inlierMatches)
                %these are the inliers for model i (indices into the matches matrix)
                matchIndices = inlierMatches{i};
                %get the matches for this image
                relevantMatches = matches(imageI,matchIndices);

                %select x,y for the image
                x = keypoints{imageI}(1,relevantMatches);
                y = keypoints{imageI}(2,relevantMatches);

                %plot
                scatter(x,y,10,colors{i});
            end
            hold off;
        end

    else
        %visualize the output of RANSAC with lines 
        [h1,w1] = size(images{1});
        [h2,w2] = size(images{2});

        %join the images
        catImage = uint8(zeros([max(h1,h2),w1+w2]));
        catImage(1:h1,1:w1) = images{1};
        catImage(1:h2,w1+1:w1+w2) = images{2};

        %show the concatenated image
        imshow(catImage); hold on;
        corresp1 = keypoints{1}(1:2,matches(1,inlierMatches{1}));
        corresp2 = keypoints{2}(1:2,matches(2,inlierMatches{1}));

        %plot the lines
        for i=1:numel(inlierMatches{1})
            plot(   [corresp1(1,i),corresp2(1,i)+w1],...
                    [corresp1(2,i),corresp2(2,i)]);
        end
        hold off;
    end

end
