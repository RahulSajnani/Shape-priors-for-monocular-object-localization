function [rot trans] = read_output()
    temprot = zeros(3,3);
    temptrans = zeros(3,1);
    filename = 'ceres_output_singleViewPoseAdjuster.txt';
    f = fopen(filename, 'r');
    tempArray = fscanf(f,'%lf');
    for i = 1:3
        for j = 1:3
            temprot(j,i) = tempArray(3*(i - 1) + j);
        end
    end
    for i = 1:3
        temptrans(i,1) = tempArray(9 + i);
    end
    fclose(f);
    rot = temprot;
    trans = temptrans;
end