function WireframeToImage(wireframe, seqs, frames, ids, height_camera, avgL, avgW, avgH, K, networkOutput, lambda, eigVectors1, kplookup, numViews,seqId,CarId,startFrame,endFrame,totalFrames)
    

    avgCarHeight = avgH;
    avgCarLength = avgL;
    avgCarWidth = avgW;
    terrortot = 0;
    plot_data = [];
    !rm lambdas.txt
    %scaling wireframe to avg height, width and length
    [frame2 eigVectors2] = ScaleAvg(wireframe, avgH, avgW, avgL, eigVectors1);
    % getting the tracklets
    tracklets = storeVals(seqs, frames, ids);
    % getting the hourglass network output
    Pts = networkOutput;
    % changing network output to 2d points on the image
    Pts = HNtoImage(Pts, tracklets);
    Etrinit = [];
    EtrPose = [];
    ERepinit = [];
    EReppose = [];
    ERepshape = [];
    Figplot = [];
    DegError = [];
    CENTER = {};
    KPS = {};
    SHAPEbeforesingleView = {};
    VECTORS = {};
    LaftersingleView = {};
    POSEafterPnP = {};
    T = [1 0 0;0 cos(pi/2) sin(pi/2);0 -sin(pi/2) cos(pi/2)];
    r = [cos(pi) sin(pi) 0; -   sin(pi) cos(pi) 0;0 0 1];
    T2 = [cos(pi/2) 0 sin(pi/2); 0, 1, 0; -sin(pi/2), 0, cos(pi/2)];
    % transformation to align world with KITTI dataset camera
    r = r*T;
    
    iter = size(tracklets,1);
    for i = 1:iter
        
        %change path according to your system
        Image = fullfile('~/Robotics Research Centre/KITTI dataset/data_tracking_image_2/training/image_02/', sprintf('%04d/%06d.png', table2array(tracklets(i, {'sequence'})), table2array(tracklets(i, {'frame'}))));        
        NetPts = reshape(Pts(i,:), 3, 14);%pts from the Hourglass network
        
        % x and y points to estimate translation
        xp = (table2array(tracklets(i,{'x2'})) + table2array(tracklets(i,{'x1'})))/2;
        yp = table2array(tracklets(i,{'y2'}));
        %yp = (table2array(tracklets(i,{'y1'})) + table2array(tracklets(i,{'y2'})))/2;
        
        rycorr = table2array(tracklets(i,{'ry'}));
        Rerror = [cos(rycorr) 0 sin(rycorr); 0 1 0; -sin(rycorr) 0 cos(rycorr)];
        Rerrorx = [1;0;0];
        Rcorrect = Rerror*Rerrorx;
        Rcorrect(2) = 0;
        corrmag = sqrt(sum(Rcorrect.^2));
        
        
        %azimuthal angle with some noise
        ry = table2array(tracklets(i,{'ry'})) + normrnd(0,15*pi/180);
        
        %calculating translation vector
        Trans = TransFromPoint(xp, yp, height_camera, K);
        %tweaking translation
        Trans3dcent = [0;-avgH/2;avgW/2];
        % rotation matrix to rotate for azimuthal angle
        T3 = [cos(ry) 0 sin(ry); 0 1 0; -sin(ry) 0 cos(ry)];
        %d3PlotPts = (T3)*(T2)*(inv(r)*(frame2') + Trans3dcent) + Trans ;
        
        %3d world points
        d3PlotPtsWorld3d = (T3)*(T2)*(inv(r)*(frame2')) + Trans + Trans3dcent;
        
        SHAPEbeforesingleView{i} = d3PlotPtsWorld3d';
        CENTER{i} = mean(d3PlotPtsWorld3d');
        
        for j=1:5
             eigVectors(j,:) =reshape(((T3)*(T2)*(inv(r))*reshape(eigVectors2(j,:),3,14)),1,42);
        end
        
        VECTORS{i} = eigVectors;
        %writing input for pose adjustment
        kpout = Write_input(NetPts', d3PlotPtsWorld3d',Trans + Trans3dcent, avgH, avgW, avgL, K, lambda, eigVectors, ry, kplookup);
        !./singleViewPoseAdjuster
        % reading input after pose adjustment
         [Rotcorr Tcorr] = read_output();
%         
%         Rotcorr = eye(3,3);
%         Tcorr = zeros(3,1);
        POSEafterPnP{i} = [Rotcorr(:)' Tcorr'];
        
        Rnew = Rotcorr*T3*Rerrorx;
        inR = Rotcorr*T3;
        A = rotm2axang(inR);
        B = rotm2axang(Rerror);
        cross(A(1:3)',B(1:3)');
        DegError = [DegError; (abs((A(4) - B(4))*180/pi))]; 
        Rnew(2) = 0;
        incorrectRmag = norm(Rnew);
        thet = Rcorrect'*Rnew./(incorrectRmag*corrmag);
        thet = acos(thet)*180./pi;
        
        %DegError = [DegError;thet];
%       writing input for shape adjustment
        Write_input_Shape(NetPts', d3PlotPtsWorld3d',((Trans + Trans3dcent)), avgH, avgW, avgL, K, lambda, eigVectors, ry, kplookup, Rotcorr, Tcorr);        
%         Write_input_Shape(NetPts', d3PlotPtsWorld',Trans + Trans3dcent, avgH, avgW, avgL, K, lambda, eigVectors, ry, kplookup, eye(3,3), zeros(3,1));

        !./singleViewShapeAdjuster
        % reading 3D points after shape adjustment
        d3PlotShape3D = read_output_Shape();
        
        d3PlotShape = K*d3PlotShape3D;
        d3PlotShape = d3PlotShape./d3PlotShape(3,:);            
        
        d3PlotPts = K*d3PlotPtsWorld3d;
        d3PlotPts = d3PlotPts./d3PlotPts(3,:);
        NetPts = reshape(Pts(i,:), 3, 14);
        tmpkps = NetPts';
        KPS{i} = [tmpkps(:,1:2) kpout];
        h = figure;
        subplot(2,2,1)
        hold on
        imshow(Image)
        plot(NetPts(1,:), NetPts(2,:), 'o', 'MarkerFaceColor', 'r');
        hold off
        title('Image with keypoints from the Hourglass network');
        
        subplot(2,2,2)
        visualizeWireframe2D(Image, d3PlotPts(1:2,:));
        title('Mean wireframe without pose adjustment');    
        %       hold on
%         %plot(NetPts(1,:), NetPts(2,:), 'o', 'MarkerFaceColor', 'r');
%         hold off
        
        
        d3PlotPtsWorld3D = Rotcorr*d3PlotPtsWorld3d + Tcorr;
        
        %figure;
        %d3PlotPts = visualizeWireframe3D(frame2', ry, Trans);
        %visualize3dscene(d3PlotPtsWorld,height_camera);
        d3PlotPts2 = K*d3PlotPtsWorld3D;
        d3PlotPts2 = d3PlotPts2./d3PlotPts2(3,:);
        NetPts = reshape(Pts(i,:), 3, 14);
        
        subplot(2,2,3);
        visualizeWireframe2D(Image, d3PlotPts2(1:2,:));
        title('Wireframe after Pose adjustments');
        hold on
        %plot(NetPts(1,:), NetPts(2,:), 'o', 'MarkerFaceColor', 'r');
        hold off
        
        subplot(2,2,4);
        visualizeWireframe2D(Image, d3PlotShape(1:2,:));
        title('After shape and pose adjustments');
        close(h);
        out = read_plot();
        %plot_data = [plot_data ;i out];
        [tempinit, temppose, tempshape] = plot_reprojection_error(NetPts',d3PlotPts(1:2,:)',d3PlotPts2(1:2,:)',d3PlotShape(1:2,:)',0);
        ERepinit = [ERepinit;tempinit];
        EReppose = [EReppose;temppose];
        ERepshape = [ERepshape;tempshape];
%         figure;
%         subplot(1,3,1);
%         visualize3dscene(d3PlotPtsWorld3d,height_camera);
%         title('before pose adjustment');
%         subplot(1,3,2);
%         visualize3dscene(d3PlotPtsWorld3D,height_camera);
%         title('after pose adjustment');
%         subplot(1,3,3);
%         visualize3dscene(d3PlotShape3D,height_camera);
%         title('after shape adjustment');
        
        tactual = table2array(tracklets(i,{'t'}));
        denom = sqrt(sum(tactual.^2));
        tcalc = Rotcorr*(Trans) + Tcorr;
        num = sqrt(sum((tactual' - tcalc).^2));
        EtrPose = [EtrPose; num];
        num_initial = sqrt(sum((tactual' - Trans).^2));
        Etrinit = [Etrinit;num_initial];
        Terrorpercent = num.*100./denom;
        terrortot = terrortot + Terrorpercent;
        Figplot = [Figplot;i];
        pause(1.0);
        %break;
    end
    figuresh = figure;
    plot(Figplot,ERepinit);
    hold on
    plot(Figplot,EReppose);
    plot(Figplot,ERepshape);
    xlabel('Figures');
    ylabel('Reprojection Error');
    legend('Initial error','Error after pose adjustment','After shape adjustment');
    hold off
    title('Reprojection Error');
    saveas(figuresh,sprintf('../Seq%d_%d_%d_%d/ReprojectionError.jpg',seqId,startFrame,endFrame,CarId));
    
    figuresl = figure;
    plot(Figplot,Etrinit);
    hold on
    plot(Figplot,EtrPose);
    xlabel('Figures');
    ylabel('Translation Error');
    legend('Initial error','Error after pose adjustment');    
    title('Translation error');
    saveas(figuresl,sprintf('../Seq%d_%d_%d_%d/TranslationError.jpg',seqId,startFrame,endFrame,CarId));
    figuresr = figure;
    plot(Figplot,DegError);
    hold on
    xlabel('Figures');
    ylabel('Rotation Error in degrees');    
    title('Rotation error');
    saveas(figuresr,sprintf('../Seq%d_%d_%d_%d/RotationError.jpg',seqId,startFrame,endFrame,CarId));
    
    pause(1.0);
%    terrortot./iter;
    LaftersingleView = read_lambdas(numViews);
    write_inpFile_multiviewadjuster;
    !multicpp/multiViewShapeandPoseAdjuster
    plot_multi_image(K,tracklets,seqId,CarId,startFrame,endFrame);
end

        
    