function [phi theta psi] = EulerGyro(p, q, r, dt)
%
%
persistent prevPhi prevTheta prevPsi


if isempty(prevPhi)
  prevPhi   = 0;
  prevTheta = 0;
  prevPsi   = 0;
end

sinPhi   = sin(prevPhi);    cosPhi   = cos(prevPhi);
cosTheta = cos(prevTheta);  tanTheta = tan(prevTheta);

phi   = prevPhi   + dt*( p + q*sinPhi*tanTheta + r*cosPhi*tanTheta );
theta = prevTheta + dt*(     q*cosPhi          - r*sinPhi );
psi   = prevPsi   + dt*(     q*sinPhi/cosTheta + r*cosPhi/cosTheta );

%chanege rotation sequance from 'XYZ' to 'ZYX'
eul = [phi theta psi];
rotmZYX = eul2rotm(eul);
eul = rotm2eul(rotmZYX);
phi = eul(1);
theta = eul(2);
psi = eul(3);

prevPhi   = phi;
prevTheta = theta;
prevPsi   = psi; 