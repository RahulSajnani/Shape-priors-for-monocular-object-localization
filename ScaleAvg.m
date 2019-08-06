function [pts vectors] = ScaleAvg(in_frame, H, W, L, vectors1)
    % in_frame 14 x 3 
%     m = size(in_frame,1);
%     max_L = 0;
%     min_L = 0;
%     
%     max_H = 0;
%     min_H = 0;
%     
%     max_W = 0;
%     min_W = 0;
%     for i = 1:m
%         if in_frame(i,1) > max_W
%             max_W = in_frame(i,1);
%         end
%         if in_frame(i,1) < min_W
%             min_W = in_frame(i,1);
%         end
%         
%         if in_frame(i,2) > max_L
%             max_L = in_frame(i,2);
%         end
%         if in_frame(i,2) < min_L
%             min_L = in_frame(i,2);
%         end
%         
%         if in_frame(i,3) > max_H
%             max_H = in_frame(i,3);
%         end
%         if in_frame(i,3) < min_H
%             min_H = in_frame(i,3);
%         end
%     end
    divH = abs(max(in_frame(:,3) - min(in_frame(:,3)))); 
    divL = abs(max(in_frame(:,2) - min(in_frame(:,2))));
    divW = abs(max(in_frame(:,1) - min(in_frame(:,1))));
    
    pts = [in_frame(:,1).*W./divW in_frame(:,2).*L./divL in_frame(:,3).*H./divH];
    
    for i = 1:5
    vectors(i,:) = reshape((diag([W/divW,L/divL,H/divH])*reshape(vectors1(i,:),3,14)),1,42);
    end
    
end