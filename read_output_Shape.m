function d2plotpts = read_output_Shape()

    
    filename = 'ceres_output_singleViewShapeAdjuster.txt';
    f = fopen(filename, 'r');
    
    k = fscanf(f,'%lf');
    d2plotpts = reshape(k,3,14);
    fclose(f);
end
