function [y] = measureUKF(x, an, mn)
    q = x(1:4);
    
    ab_hat = quatrotate(q,an');
    mb_hat = quatrotate(q,mn');
    
    y = [ab_hat mb_hat];
end