function [phi theta] = EulerAccel(ax, ay, az, mx, my, mz)
%
%
%g = 9.8;
%{
%supported data without free acceleration, changed ax -> -ax and -ay -> ay,
%which depends on rotation matrix (below), added psi angle calculation

g = 1;

theta = asin(  -ax / g );
phi   = asin( ay / (g*cos(theta)) );
%}
% previous equaations sometime give complex numbers. It's impossible to use
% them to convert to quaternions

theta   = atan2((-ax), sqrt(ay * ay + az * az));
phi = atan2(ay, az);
psi   = atan2(-my, mx);

%{
syms phi theta psi


Rz = [cos(psi)  sin(psi) 0;
      -sin(psi) cos(psi) 0;
      0         0        1]
Ry = [cos(theta)  0   -sin(theta);
      0           1   0;
      sin(theta)  0   cos(theta)]
Rx = [1     0           0;
      0     cos(phi)    sin(phi);
      0     -sin(phi)   cos(phi) ]

R=Rx*Ry*Rz
  
A = R*[0 0 1]'

Pitch = A(1)'
Roll  = A(2)'
%}