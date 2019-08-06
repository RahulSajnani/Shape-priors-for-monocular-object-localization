function out = read_plot()

    filep = 'pose_cost.txt';
    files = 'shape_cost.txt';
    
    fp = fopen(filep , 'r');
    fs = fopen(files , 'r');
    
    tmp = fscanf(fp,'%lf');
    tms = fscanf(fs,'%lf');
    
    out = [tmp' tms];
end
