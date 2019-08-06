function visualize3dscene(CarPts,camH)

    visualizeWireframe3D(CarPts,eye(3,3),zeros(3,1));
    x = meshgrid(-10:0.5:10);
    y = camH*ones(41,41);
    z = meshgrid(0:1:40)';
    hold on 
    surf(x,y,z);
    colormap(gray);
    R = [1 0 0 ; 0 1 0; 0 0 1];
    cam = plotCamera('Location',[0 0 0],'Orientation',R,'Opacity',0);
    hold off
    shading interp;
    xlim([-20 20]);
    ylim([-20 20]);
end