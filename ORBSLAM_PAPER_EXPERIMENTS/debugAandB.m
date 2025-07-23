clc; clear;

thetaFive = deg2rad([0, (-75 -137.2985), (-105 -132.7015), 90, 0]);
dFive = [0.1625, 0, 0.1333, 0.0997, 0.0996];
aFive = [0, 0.57831, 0, 0, 0];
alphaFive = [pi/2, 0, pi/2, -pi/2, 0];

LFive(1) = Link('revolute', 'd', dFive(1), 'a', aFive(1), 'alpha', alphaFive(1), 'offset', 0);
LFive(2) = Link('revolute', 'd', dFive(2), 'a', aFive(2), 'alpha', alphaFive(2), 'offset', 0);
LFive(3) = Link('revolute', 'd', dFive(3), 'a', aFive(3), 'alpha', alphaFive(3), 'offset', 0);
LFive(4) = Link('revolute', 'd', dFive(4), 'a', aFive(4), 'alpha', alphaFive(4), 'offset', 0);
LFive(5) = Link('revolute', 'd', dFive(5), 'a', aFive(5), 'alpha', alphaFive(5), 'offset', 0);


thetaFiveNegA = deg2rad([0, (-75 +42.7015), (-105 -132.7015), 90, 0]);
dFiveNegA = [0.1625, 0, 0.1333, 0.0997, 0.0996];
aFiveNegA = [0, -0.57831, 0, 0, 0];
alphaFiveNegA = [pi/2, 0, pi/2, -pi/2, 0];

LFive(1) = Link('revolute', 'd', dFiveNegA(1), 'a', aFiveNegA(1), 'alpha', alphaFiveNegA(1), 'offset', 0);
LFive(2) = Link('revolute', 'd', dFiveNegA(2), 'a', aFiveNegA(2), 'alpha', alphaFiveNegA(2), 'offset', 0);
LFive(3) = Link('revolute', 'd', dFiveNegA(3), 'a', aFiveNegA(3), 'alpha', alphaFiveNegA(3), 'offset', 0);
LFive(4) = Link('revolute', 'd', dFiveNegA(4), 'a', aFiveNegA(4), 'alpha', alphaFiveNegA(4), 'offset', 0);
LFive(5) = Link('revolute', 'd', dFiveNegA(5), 'a', aFiveNegA(5), 'alpha', alphaFiveNegA(5), 'offset', 0);




thetaFiveMarkingGuide = deg2rad([0, (-75 +42.7015), (-105 -132.7015), 90, 0]);
dFiveMarkingGuide = [0.1625, 0, 0.1333, 0.0997, 0.0996];
aFiveMarkingGuide = [0, 0.57831, 0, 0, 0];
alphaFiveMarkingGuide = [pi/2, 0, pi/2, -pi/2, 0];

LFive(1) = Link('revolute', 'd', dFiveMarkingGuide(1), 'a', aFiveMarkingGuide(1), 'alpha', alphaFiveMarkingGuide(1), 'offset', 0);
LFive(2) = Link('revolute', 'd', dFiveMarkingGuide(2), 'a', aFiveMarkingGuide(2), 'alpha', alphaFiveMarkingGuide(2), 'offset', 0);
LFive(3) = Link('revolute', 'd', dFiveMarkingGuide(3), 'a', aFiveMarkingGuide(3), 'alpha', alphaFiveMarkingGuide(3), 'offset', 0);
LFive(4) = Link('revolute', 'd', dFiveMarkingGuide(4), 'a', aFiveMarkingGuide(4), 'alpha', alphaFiveMarkingGuide(4), 'offset', 0);
LFive(5) = Link('revolute', 'd', dFiveMarkingGuide(5), 'a', aFiveMarkingGuide(5), 'alpha', alphaFiveMarkingGuide(5), 'offset', 0);



theta = deg2rad([0,-75, 90, -105, 90, 0]);
d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996];
a = [0, -0.425, -0.3922, 0, 0, 0];
alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];

L(1) = Link('revolute', 'd', d(1), 'a', a(1), 'alpha', alpha(1), 'offset', 0);
L(2) = Link('revolute', 'd', d(2), 'a', a(2), 'alpha', alpha(2), 'offset', 0);
L(3) = Link('revolute', 'd', d(3), 'a', a(3), 'alpha', alpha(3), 'offset', 0);
L(4) = Link('revolute', 'd', d(4), 'a', a(4), 'alpha', alpha(4), 'offset', 0);
L(5) = Link('revolute', 'd', d(5), 'a', a(5), 'alpha', alpha(5), 'offset', 0);
L(6) = Link('revolute', 'd', d(6), 'a', a(6), 'alpha', alpha(6), 'offset', 0);

robot= SerialLink(L, 'name', 'six link');
robotFive = SerialLink(LFive, 'name', 'five link');

%robot.plot(theta);
%robot.plot(thetaFive);

TSix = zeros(4,4,6);
TFive = zeros(4,4,6);
TFiveNegA = zeros(4,4,6);
TFiveMarkingGuide = zeros(4,4,6);

for i = 1:6
    thetaCurr = theta(i);
    alphaCurr = alpha(i);
    aCurr = a(i);
    dCurr = d(i);

    TSix(:,:,i) = [cos(thetaCurr), -sin(thetaCurr)*cos(alphaCurr), sin(thetaCurr)*sin(alphaCurr), aCurr*cos(thetaCurr);
        sin(thetaCurr), cos(thetaCurr)*cos(alphaCurr), -cos(thetaCurr)*sin(alphaCurr), aCurr*sin(thetaCurr);
        0, sin(alphaCurr), cos(alphaCurr), dCurr;
        0,0,0,1];
    

    if (i<6)
        thetaCurr = thetaFive(i);
        alphaCurr = alphaFive(i);
        aCurr = aFive(i);
        dCurr = dFive(i);

        TFive(:,:,i) = [cos(thetaCurr), -sin(thetaCurr)*cos(alphaCurr), sin(thetaCurr)*sin(alphaCurr), aCurr*cos(thetaCurr);
            sin(thetaCurr), cos(thetaCurr)*cos(alphaCurr), -cos(thetaCurr)*sin(alphaCurr), aCurr*sin(thetaCurr);
            0, sin(alphaCurr), cos(alphaCurr), dCurr;
            0,0,0,1];


        thetaCurr = thetaFiveNegA(i);
        alphaCurr = alphaFiveNegA(i);
        aCurr = aFiveNegA(i);
        dCurr = dFiveNegA(i);

        TFiveNegA(:,:,i) = [cos(thetaCurr), -sin(thetaCurr)*cos(alphaCurr), sin(thetaCurr)*sin(alphaCurr), aCurr*cos(thetaCurr);
            sin(thetaCurr), cos(thetaCurr)*cos(alphaCurr), -cos(thetaCurr)*sin(alphaCurr), aCurr*sin(thetaCurr);
            0, sin(alphaCurr), cos(alphaCurr), dCurr;
            0,0,0,1];


        thetaCurr = thetaFiveMarkingGuide(i);
        alphaCurr = alphaFiveMarkingGuide(i);
        aCurr = aFiveMarkingGuide(i);
        dCurr = dFiveMarkingGuide(i);

        TFiveMarkingGuide(:,:,i) = [cos(thetaCurr), -sin(thetaCurr)*cos(alphaCurr), sin(thetaCurr)*sin(alphaCurr), aCurr*cos(thetaCurr);
            sin(thetaCurr), cos(thetaCurr)*cos(alphaCurr), -cos(thetaCurr)*sin(alphaCurr), aCurr*sin(thetaCurr);
            0, sin(alphaCurr), cos(alphaCurr), dCurr;
            0,0,0,1];
    else
    end
    
  
end

%T61 = TSix(:,:,1)*TSix(:,:,2)*TSix(:,:,3);
%fprintf('6DOF:\n'); disp(T61);

T51 = TFive(:,:,1)*TFive(:,:,2);
fprintf('5DOF -132 offset:\n'); disp(T51);

T51NegA = TFiveNegA(:,:,1)*TFiveNegA(:,:,2);
fprintf('5DOF +42 offset, negative a :\n'); disp(T51NegA);

T51MarkingGuide = TFiveMarkingGuide(:,:,1)*TFiveMarkingGuide(:,:,2);
fprintf('5DOFMarkingGuide:\n'); disp(T51MarkingGuide);




T01 = trotz(deg2rad(0)) * transl(0,0, 0.1625) * transl(0)* trotx((pi/2));
T12 = trotz(deg2rad(-75 + 42.7015)) * transl(0) * transl(-sqrt(0.425^2 + 0.3922^2), 0,0) * trotx(0);
fprintf('5DOFJasper:\n'); disp(T01*T12);

T01 = trotz(deg2rad(0)) * transl(0,0, 0.1625) * transl(0)* trotx((pi/2));
T12 = trotz(deg2rad(-75 + 42.7015)) * transl(0) * transl(sqrt(0.425^2 + 0.3922^2), 0,0) * trotx(0);
fprintf('5DOFJasper (no negative):\n'); disp(T01*T12);



%Tfinal = T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5);

%disp(Tfinal);