function [u1] = utSystem(X,Wm,Wc,n,Q,Wb,dT)
  %Unscented Transformation
  %Input:
  %        X: sigma points
  %       Wm: weights for mean
  %       Wc: weights for covraiance
  %        n: number of outputs of f
  %        R: additive covariance
  %Output:
  %        y: transformed mean
  %
  %        P: transformed covariance
  %       Y1: transformed deviations
  
  L = size(X,2);
  x = zeros(n,1);
  Y = zeros(n,L);
  
  for (i=1:L)
    Y(:,i) = systemUKF(X(:,i)', Wb, dT);
    x = x + Wm(i)*Y(:,i);
  end
  
  Y1 = Y - repmat(x,size(x,2),L);
  P = Y1 * diag(Wc) * Y1' + Q;
  
  u1(1).x = x;
  u1(1).Y = Y;
  u1(1).P = P;
  u1(1).Y1 = Y1;
  
end