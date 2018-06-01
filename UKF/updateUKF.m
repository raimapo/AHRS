function [Filter] = updateUKF(Filter, z, Wb, An, Mn, dT)
  % UKF   Unscented Kalman Filter for nonlinear dynamic systems
  
  % Reference: Julier, SJ. and Uhlmann, J.K., Unscented Filtering and
  % Nonlinear Estimation, Proceedings of the IEEE, Vol. 92, No. 3,
  % pp.401-422, 2004.
  
  P = Filter.P;
  x = Filter.x;
  Q = Filter.Q;
  R = Filter.R;
  
  %number of states
  L = length(x);
  %number of measurements
  m = length(z);
  %default, tunable
  ki = 0;
  %default, tunable
  alpha = 1;
  %default, tunable
  beta = 2;
  %Scaling factor
  lambda = (alpha^2)*(L+ki)-L;
  %Scaling factor
  c = L+lambda;
  %weights for means 
  Wm = [lambda/c repmat(0.5/c, 1, 2*L)];
  Wc = Wm;
  %weights for covariance
  Wc(1) = Wc(1)+(1-alpha^2+beta);
  c = sqrt(c);
  
  %Sigma points around x
  X = Sigmas(x, P, c);

  %Predict
  %unscented transformation of process
  u1 = utSystem(X,Wm,Wc,L,Q,Wb,dT);
  x1 = u1(1).x;
  X1 = u1(1).Y;
  P1 = u1(1).P;
  X2 = u1(1).Y1;
  
  %Update
  %unscented transformation of measurments
  u2 = utMeas(X1,Wm,Wc,m,R,An,Mn);
  z1 = u2(1).y;
  P2 = u2(1).P;
  Z2 = u2(1).Y1;
  
  %transformed cross-covariance
  P12 = X2 * diag(Wc) * Z2';
  K = mrdivide(P12, P2);
  %State update
  x = x1+K * (z-z1);
  %Covariance update
  P = P1-K * P12';
  
  %Assign new filter state
  Filter(1).P = P;
  Filter(1).x = x;
end