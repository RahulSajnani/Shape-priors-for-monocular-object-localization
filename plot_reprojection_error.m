function [Einit, Epose, Eshape] = plot_reprojection_error(imagePts,initPts, posePts, shapePts, Show)

    m = size(imagePts,1);
    errorinit = sqrt((imagePts(:,1) - initPts(:,1)).^2 + (imagePts(:,2) - initPts(:,2)).^2);
    errorpose = sqrt((imagePts(:,1) - posePts(:,1)).^2 + (imagePts(:,2) - posePts(:,2)).^2);
    errorshape = sqrt((imagePts(:,1) - shapePts(:,1)).^2 + (imagePts(:,2) - shapePts(:,2)).^2);
    
    if Show == 1
        i = 1:m;
        figure;
    %subplot(1,3,1);
        plot(i,errorinit);
        hold on 
        plot(i,errorpose);
        plot(i,errorshape);
    
        xlabel('keypoints');
        ylabel('reprojection error');
        legend('initial error','error after pose adjustment','error after shape adjustment');
        hold off
    end
    Einit = sum(sqrt((imagePts(:,1) - initPts(:,1)).^2 + (imagePts(:,2) - initPts(:,2)).^2));
    Epose = sum(sqrt((imagePts(:,1) - posePts(:,1)).^2 + (imagePts(:,2) - posePts(:,2)).^2));
    Eshape = sum(sqrt((imagePts(:,1) - shapePts(:,1)).^2 + (imagePts(:,2) - shapePts(:,2)).^2));    
end
    
   