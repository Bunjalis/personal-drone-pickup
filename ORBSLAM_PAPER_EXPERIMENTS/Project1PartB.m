clear; clc;
startup_rvc;

% %%
% theta2 = deg2rad([0, -75, 90, -105, 90, 0]);
% d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996];
% a = [0, -0.425, -0.3922, 0, 0, 0];
% alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];
% 
% K(1) = Link('revolute', 'd', d(1), 'a', a(1), 'alpha', alpha(1), 'offset', 0);
% K(2) = Link('revolute', 'd', d(2), 'a', a(2), 'alpha', alpha(2), 'offset', 0);
% K(3) = Link('revolute', 'd', d(3), 'a', a(3), 'alpha', alpha(3), 'offset', 0);
% K(4) = Link('revolute', 'd', d(4), 'a', a(4), 'alpha', alpha(4), 'offset', 0);
% K(5) = Link('revolute', 'd', d(5), 'a', a(5), 'alpha', alpha(5), 'offset', 0);
% K(6) = Link('revolute', 'd', d(6), 'a', a(6), 'alpha', alpha(6), 'offset', 0);
% 
% 
% robot= SerialLink(K, 'name', 'six link');
% robot.plot(theta2);

%%

%theta = deg2rad([0, (-75 + 132.7015 +90), (-105 + 47.2985 +180), 90, 0]);
%theta = deg2rad([0, (-75 -137.2985), (-105 -132.7015), 90, 0]);
theta = deg2rad([0, (-90 + 42.2985), (0 + 47.27015), 0, 0]);
%theta = deg2rad([0, (0), (0), 0, 0]);


d = [0.1625, 0, 0.1333, 0.0997, 0.0996];
a = [0, 0.57831, 0, 0, 0];
alpha = [pi/2, 0, pi/2, -pi/2, 0];

L(1) = Link('revolute', 'd', d(1), 'a', a(1), 'alpha', alpha(1), 'offset', 0);
L(2) = Link('revolute', 'd', d(2), 'a', a(2), 'alpha', alpha(2), 'offset', 0);
L(3) = Link('revolute', 'd', d(3), 'a', a(3), 'alpha', alpha(3), 'offset', 0);
L(4) = Link('revolute', 'd', d(4), 'a', a(4), 'alpha', alpha(4), 'offset', 0);
L(5) = Link('revolute', 'd', d(5), 'a', a(5), 'alpha', alpha(5), 'offset', 0);

robot = SerialLink(L, 'name', 'five link');

T = zeros(4,4,5);

for i = 1:5
    thetaCurr = theta(i);
    alphaCurr = alpha(i);
    aCurr = a(i);
    dCurr = d(i);

    T(:,:,i) = [cos(thetaCurr), -sin(thetaCurr)*cos(alphaCurr), sin(thetaCurr)*sin(alphaCurr), aCurr*cos(thetaCurr);
        sin(thetaCurr), cos(thetaCurr)*cos(alphaCurr), -cos(thetaCurr)*sin(alphaCurr), aCurr*sin(thetaCurr);
        0, sin(alphaCurr), cos(alphaCurr), dCurr;
        0,0,0,1];
end

Tfinal = T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5);
disp(Tfinal);