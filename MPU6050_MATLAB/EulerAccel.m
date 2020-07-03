function [phi, theta] = EulerAccel(ax,ay)

g = 9.81;

theta =  asin(ax/g);
%-atan2(az, sqrt(ay^2 + az^2));
%asin(ax/g);
phi = asin(-ay/(g*cos(theta)));
%atan2(ay, az);
%asin(-ay/(g*cos(phi)));



end