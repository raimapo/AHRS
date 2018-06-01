function [x] = systemUKF(x, wb, dT)
    q = x(1:4)';
    dw = x(5:7);
    %Correct angular rate
    w_hat = (wb-dw);
    %calculate quaternion
    q = Qrot(q,w_hat,dT');
    q = quatnormalize(q);
    x(1:4) = q;
end
