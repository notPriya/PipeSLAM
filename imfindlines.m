% Uses a Hough transform to find lines in the image.
function lines = imfindlines(I)
    % Filter the image.
    im = rgb2gray(I);
    
    % Find the edges in the image using the Canny detector.
    BW = edge(im, 'canny');

    % Compute the Hough transform of the image.
    [H,theta,rho] = hough(BW);
    
    % Find the peaks in the Hough transform matrix, H.
    P = houghpeaks(H, 40,'threshold',ceil(0.3*max(H(:))));
    
    % Find lines in the image using the houghlines function.
    lines = houghlines(BW,theta,rho,P,'FillGap',40,'MinLength', 300);
    
    % Find the forwards (positive slope) and backwards (negative slope) line.
    backwards_line = [];
    forwards_line = [];
    for i=1:size(lines, 2)
        if isempty(backwards_line) && lines(i).theta < 0
            backwards_line = i;
        end
        if isempty(forwards_line) && lines(i).theta > 0
            forwards_line = i;
        end
        if ~isempty(forwards_line) && ~isempty(backwards_line)
            break;
        end
    end
    
    % If we didnt find a backwards line throw an error.
    if isempty(backwards_line) || isempty(forwards_line)
        disp('Error in imfindlines: Did not find two lines!');
        lines = [];
        return;
    end
    
    % The two lines we care about are the best looking positive theta line
    % and the best looking negative theta line.
    lines = [lines(forwards_line) lines(backwards_line)];
    
%     imshow(I);
%     hold on;
%     for k=1:size(lines,2)
%         xy = [lines(k).point1; lines(k).point2];
%         plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
% 
%         % Plot beginnings and ends of lines
%         plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
%         plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
%     end
%     hold off;
end