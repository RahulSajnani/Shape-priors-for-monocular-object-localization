function out =  Write_input(imagePts, carPts,carcent, H, W, L, K, lambda, eigVectors, ry, kplookup)
    
    out = [];
    numViews = 1;
    numPts = 14;
    numObs = 14;
    xm = (sum(carPts(:,1))./size(carPts,1));
    ym = (sum(carPts(:,2))./size(carPts,1));
    zm = (sum(carPts(:,3))./size(carPts,1));
    
    filename = 'ceres_input_singleViewPoseAdjuster.txt';
    f = fopen(filename, 'w');
    fprintf(f, '%d %d %d\n',numViews, numPts, numObs);
    fprintf(f,'\n');
    
    fprintf(f, '%f %f %f\n', carcent(1), carcent(2), carcent(3));
%     fprintf(f, '%f %f %f\n', xm, ym, zm);
    fprintf(f,'\n');
    
    fprintf(f, '%f %f %f\n', H, W, L);
    fprintf(f,'\n');    
    
    for i = 1:3
        for j = 1:3
            fprintf(f, '%f ',K(i,j));
        end
        fprintf(f,'\n');
    end
    fprintf(f,'\n');
    
    %observations
    m = size(imagePts,1);
    for i = 1:m
        fprintf(f, '%f %f\n', imagePts(i,1), imagePts(i,2));
    end
    fprintf(f,'\n');
    
    angle = round(rad2deg(ry));
    if angle <= 0
        angle = angle + 360;
    end
    
    divider = sum(kplookup(angle,:));
    
    %weights
    for i = 1:m
%         if (0.8*imagePts(i,3) + 0.2*kplookup(angle,i)./divider) < 0.1
%             fprintf(f, '%f ', 0.1);
%         else
            fprintf(f, '%f ', (0.3*imagePts(i,3) + 0.7*kplookup(angle,i)./divider));
            out = [out;(0.3*imagePts(i,3) + 0.7*kplookup(angle,i)./divider)];
%         end
    end
    
    fprintf(f,'\n');
    fprintf(f,'\n');
    
    %mean locations
    for i = 1:numObs
        fprintf(f,'%f %f %f\n', carPts(i,1), carPts(i,2), carPts(i,3));
    end
    fprintf(f,'\n');
    
    %vectors
    for i = 1:5
        for j = 1:3*numPts
            fprintf(f, '%f ',eigVectors(i,j));
        end
        fprintf(f,'\n');
    end
    fprintf(f,'\n');
    
    %lambdas
    for i = 1:5
        fprintf(f, '%f ',lambda(i));
    end
    fclose(f);
    %Write_input(NetPts', d3plotPts, avgCarHeight, avgCarWidth, avgCarLength, K, lambda, V);
end