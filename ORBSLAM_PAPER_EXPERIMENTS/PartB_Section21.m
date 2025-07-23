startup_rvc;

%%%% DH parameters (angle) in degree unit
angles= [0.00, -75.00, -105.00, 90.00, 0.00];

joint1 = [0 0.1625 0 (pi/2)];
joint2 = [0 0 sqrt(0.425^2 + 0.3922^2) 0];
joint3 = [0 0.1333 0 (pi/2)];
joint4 = [0 0.0997 0 -(pi/2)];
joint5 = [0 0.0996 0 0];

N(1) = Link(joint1);
N(2) = Link(joint2);
N(3) = Link(joint3);
N(4) = Link(joint4);
N(5) = Link(joint5);

q2_armN = SerialLink(N, 'name', 'Articulated');

% We are just manually adding the theta offset here, but students can use
% the offset option in when defining each Link.
theta = [angles(1), angles(2) + 42.7015 - 180, angles(3) + 47.2985 - 180, angles(4:end)]
theta = deg2rad(theta)
figure;
q2_armN.plot(theta);  % Show the robot
hold on;

% Loop through each joint and draw the axes
for i = 1:2
    T = q2_armN.A(1:i, theta);  % Get the transform up to joint i
    trplot(T, 'frame', num2str(i), 'length', 0.5, 'rgb');  % draw axes
end

% Draw the tool/end-effector frame as well
%T_tool = q2_armN.fkine(theta);
%trplot(T_tool, 'frame', 'EE', 'length', 0.5, 'rgb');

jointB_fKineNew = q2_armN.fkine(theta)

rpy = tr2rpy(jointB_fKineNew);
xyz = transl(jointB_fKineNew)*1000