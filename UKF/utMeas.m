function [u2] = utMeas(X,Wm,Wc,n,R,An,Mn)
  %Unscented Transformation
  %Input:
  %        X: sigma points
  %       Wm: weights for mean
  %       Wc: weights for covraiance
  %        n: number of outputs of f
  %        R: additive covariance
  %Output:
  %        y: transformed mean
  %        Y: transformed sampling points
  %        P: transformed covariance
  %        Y1: transformed deviations
  
  L = size(X,2);
  y = zeros(n,1);
  Y = zeros(n,L);
  
  for (i=1:L)
    Y(:,i) = measureUKF(X(:,i)', An, Mn);
    y = y + Wm(i)*Y(:,i);
  end
  
  Y1 = Y - repmat(y,size(y,2),L);
  P = Y1 * diag(Wc) * Y1' + R;
  
  u2(1).y = y;
  u2(1).Y = Y;
  u2(1).P = P;
  u2(1).Y1 = Y1;    
end