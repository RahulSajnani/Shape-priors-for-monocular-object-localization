function RotAndPlot(frame, ry, H, W, L)
    
    frame2 = ScaleAvg(frame, H, W, L);
    visualizeWireframe3D(frame2', ry);
end
    
    