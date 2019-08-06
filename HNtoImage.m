function y = HNtoImage(Pts, tableVals)
    
    y = [];
    m = size(Pts, 1);
    for i = 1:m
        
        
        d2plot = reshape(Pts(i,:),3,14)';
        xp1 = table2array(tableVals(i, {'x1'}));
        yp1 = table2array(tableVals(i, {'y1'}));
        xp2 = table2array(tableVals(i, {'x2'}));
        yp2 = table2array(tableVals(i, {'y2'}));
        
        xtot = xp2 - xp1 + 1;
        ytot = yp2 - yp1 + 1;
        
        d2plot = [(d2plot(:,1).*xtot./64) d2plot(:,2).*ytot./64 d2plot(:,3)];
        d2plot = [(d2plot(:,1) + xp1 - 1) (d2plot(:,2) + yp1 - 1) d2plot(:,3)];
        d2plot = [(d2plot(:,1:2)) d2plot(:,3)];
        
        temp = d2plot';
        temp = temp(:)';
        y = [y;temp];
    end
  
end