function T = TransFromPoint(x, y, H, K)
    xp = [x;y;1];
    Ray = inv(K)*xp;
    n = [0;-1;0];%ground normal
    T = -H*Ray/(n'*Ray);
end