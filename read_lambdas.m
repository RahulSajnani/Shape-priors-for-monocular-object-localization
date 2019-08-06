function l = read_lambdas(numViews)
    
    filename = 'lambdas.txt';
    f = fopen(filename, 'r');
    
    k = fscanf(f,'%lf');
    m = reshape(k,5,numViews)';
    for i = 1:numViews
        l{i} = m(1,:);
    end
    fclose(f);
end