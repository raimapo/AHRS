function [X] = Sigmas(x, P, c)
  %Sigma points around reference point
  %Inputs:
  %       x: reference point
  %       P: covariance
  %       c: coefficient
  %Output:
  %       X: Sigma points
  A = c*(chol(P))';
  Y = repmat (x, 1, size(x, 1));
  X = horzcat(x, Y+A, Y-A);
end