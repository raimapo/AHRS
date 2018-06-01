function Q = Qrot(Q, w, dt)
  % Updates current attitude quaternion q
  % output - current attitude quaternion
  % input - initial quaternion value
  % input - wx, wy, wz - angular rate values
  % input - dT - inverse of update rate

N=size(Q,1);
Qr=zeros(4,1);

Fx=w(1,1)*dt;
Fy=w(1,2)*dt;
Fz=w(1,3)*dt;

Fm = sqrt(Fx*Fx+Fy*Fy+Fz*Fz);
sinfm2=sin(Fm/2);
cosfm2=cos(Fm/2);

if (Fm~=0)
    Qr(1)=cosfm2;
    Qr(2)=(Fx/Fm)*sinfm2;
    Qr(3)=(Fy/Fm)*sinfm2;
    Qr(4)=(Fz/Fm)*sinfm2;
    
else
    Qr(1)=1;
    Qr(2)=0;
    Qr(3)=0;
    Qr(4)=0;
end
Qr=Qr';
Q=Q';
Q=quatmultiply(Q,Qr);
%Q(1)=1;