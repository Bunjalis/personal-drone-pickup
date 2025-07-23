% Project1PartA.m
% MTRN4230 Project 1 25T2
% Name: Hendrik Arendsen
% Zid:  z5260082

clear; clc;
startup_rvc;

host = '127.0.0.1'; % THIS IP ADDRESS MUST BE USED FOR THE VIRTUAL BOX VM
%host = '192.168.0.100'; % THIS IP ADDRESS MUST BE USED FOR THE REAL ROBOT
port = 30003;
rtde = rtde(host, port);

disp("Your elbow joint is stuck at 90 degrees!")

disp("Enter the pickup position")
pickupJointConfiguration = readConfiguration();

disp("Set robot to remote control mode then click enter")
input('');

clc;
pickupPose = convertJointToPose(pickupJointConfiguration);


jointAngles = [pickupJointConfiguration(1:2); 90; pickupJointConfiguration(3:end)];
rtde.movej(deg2rad(jointAngles)', 'joint');
rtde.movej(deg2rad(jointAngles)', 'joint');

clc;
% get real robot pose
realPose = rtde.actualPosePositions();


% RTDE says it's taking in [x,y,z,r,p,y] but 
% its actually taking in [x,y,z,(axis-angle rotation vector)]
% The below four lines converts students rpy pose, into one with a rotation
% vector, this requires the computer vision toolbox
Tp = rpy2tr(pickupPose(4:6));
pickupPose_converted = [pickupPose(1:3), rotmat2vec3d(Tp(1:3, 1:3))];

% Check student and real pose
checkSolution(realPose, pickupPose_converted)


% Function to convert user input to array
function configuration = readConfiguration()
    configuration = [];

    in = input('Enter joint configuration exactly in the form "j1,j2,j4,j5,j6": ', 's');
    joints = split(in, ",");

    for joint = joints
        configuration = [configuration, str2double(joint)];
    end
end

% Function for checking if solutions are correct. Do not modify!!!
% Look out for cases were -2.2214   -2.2214 == 2.2214    2.2214 for R and P
% values.
function correct = checkSolution(realPose, calucatedPose_converted)
    if realPose == calucatedPose_converted
        disp("Correct solution!")
    else
        i = 1;
        while i <= size(realPose,2)
            if abs(realPose(i) - calucatedPose_converted(i)) > 0.01
                errorString = "Mismatch between calculated an real solution at Pose value #" + i;
                realString = "The real value is " + realPose(i);
                yourString = "Your value is " + calucatedPose_converted(i);
                disp(errorString)
                disp(realString)
                disp(yourString)
            else
                disp("Correct solution, within tolerance!")
            end
            i = i+1;
        end
    end
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% You must implement the following function
% Output pose should be in the form [x, y, z, r, p, y] with rpy in radians.
% Remember to view pose using 'base' view in the simulator.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function outputPose = convertJointToPose(jointConfiguration)
    
    % You must not use RTDE to directly calculate forward kinematics (it
    % must be done from first principles). So functions like fkine() cannot
    % be used. 
    % However, functions for conversions can be used, such as rpy2tr().
    
    theta = deg2rad([jointConfiguration(1),jointConfiguration(2),90, ...
        jointConfiguration(3), jointConfiguration(4), jointConfiguration(5)]);
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

    T = zeros(4,4,6);

    for i = 1:6
        thetaCurr = theta(i);
        alphaCurr = alpha(i);
        aCurr = a(i);
        dCurr = d(i);

        T(:,:,i) = [cos(thetaCurr), -sin(thetaCurr)*cos(alphaCurr), sin(thetaCurr)*sin(alphaCurr), aCurr*cos(thetaCurr);
            sin(thetaCurr), cos(thetaCurr)*cos(alphaCurr), -cos(thetaCurr)*sin(alphaCurr), aCurr*sin(thetaCurr);
            0, sin(alphaCurr), cos(alphaCurr), dCurr;
            0,0,0,1];
    end

    Tfinal = T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*T(:,:,6);
    disp(Tfinal);

    % Tcheck = robot.fkine(theta);
    % fprintf('The check result is:\n');
    %display(Tcheck);
    robot.plot(theta);
    
    R = [Tfinal(1:3,1:3)];

    eulZYX = rotm2eul(R);  % ZYX order: yaw (Z), pitch (Y), roll (X)

    

    outputPose = [Tfinal(1,4), Tfinal(2,4), Tfinal(3,4), eulZYX(3),  eulZYX(2),  eulZYX(1)];

    %%
end
