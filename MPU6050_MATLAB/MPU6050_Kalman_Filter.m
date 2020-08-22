%% 실시간 측정 및 칼만 필터
clc
clear

Nsamples = 500;

dt = 0.1;

clear s
n=1;

while n<=Nsamples
    s = serialport("COM3",115200,"Timeout",60);
    configureTerminator(s,"CR");
    data = readline(s)
    split_data = strsplit(data,', ');
    
    a = str2double(split_data(1,1:3));
    p = str2double(split_data(1,4));
    q = str2double(split_data(1,5));
    r = str2double(split_data(1,6));
    
    A = eye(4) + dt*1/2*[0, -p, -q, -r;
                         p,  0,  r, -q;
                         q, -r,  0,  p;
                         r,  q, -p,  0];
                    
    [phi1, theta1] = EulerAccel(a(1,1),a(1,2));
    z = EulerToQuaternion(phi1,theta1,0);
                    
    [phi, theta, psi] = EulerKalman(A,z);
    
    AccelSaved(n,:) = [phi1, theta1];
    EulerSaved(n,:) = [phi, theta, psi];
    
    n = n+1;
    clear s
end

PhiSaved1 = AccelSaved(:,1)*180/pi;
ThetaSaved1 = AccelSaved(:,2)*180/pi;

PhiSaved = EulerSaved(:,1)*180/pi;
ThetaSaved = EulerSaved(:,2)*180/pi;
PsiSaved = EulerSaved(:,3)*180/pi;

t = 0:dt:Nsamples*dt-dt;

figure
plot(t,PhiSaved,'r')
hold on
plot(t,PhiSaved1,'b--')
hold off

figure
plot(t,ThetaSaved,'r')
hold on
plot(t,ThetaSaved1,'b--')
hold off

figure
plot(t,PsiSaved)




