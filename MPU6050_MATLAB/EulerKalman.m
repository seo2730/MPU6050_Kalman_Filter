function [phi, theta, psi] = EulerKalman(A,z)

persistent H Q R
persistent x P
persistent firstRun

if isempty(firstRun)
    H = eye(4);
    
    Q = 5*eye(4); % 공분산
    R = 0.5*eye(4);   % 공분산 
    
    x = [1 0 0 0]';
    P = 1*eye(4);
    
    firstRun = 1;
end

xp = A*x;
Pp = A*P*A' + Q;

K = Pp*H'*(H*Pp*H'+R)^-1;

x = xp + K*(z - H*xp); % x = [q1, q2, q3, q4]
P = Pp - K*H*Pp;

% 이거 식 복습할 것.
phi = atan2( 2*(x(3)*x(4) + x(1)*x(2)) , 1-2*(x(2)^2 + x(3)^2) );
theta = -asin( 2*(x(2)*x(4) - x(1)*x(3)) );
psi = atan2( 2*(x(2)*x(3) + x(1)*x(4)), 1-2*(x(3)^2 + x(4)^2) );

end