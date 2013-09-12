function visualizeFeatures(I)
     I = preprocessImage(I);
     k = calculateFeatures(I);
     
     imshow(I);
     hold on;
     plot(k(1:min(500, length(k)), 1), k(1:min(500, length(k)), 2), 'rx', 'LineWidth', 2);
end

