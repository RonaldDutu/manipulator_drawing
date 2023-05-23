%%%%%%%%%%%%%%%%%%%%%%%%%%   ROBOT DEFINITION   %%%%%%%%%%%%%%%%%%%%%%%%%%%
% In this file we set all parameters of a three link manipulator, saving
% them in the DH.mat and the three_link.mat files. 
% They will be used in the other functions.
% three_link.mat is a SerialLink file, defining the robot.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all, clear all, clc

%% DH Parameters

DH.a1 = 0; %Link length
DH.a2 = 0.085;                      % 8.5 cm
DH.a3 = 0.154;                      % 15.4 cm
DH.d1 = 0.058; %Link offset         % 5.8 cm
DH.d2 = 0;
DH.d3 = 0;
DH.alpha1 = pi/2;
DH.alpha2 = 0;
DH.alpha3 = 0;

%% Links definition

L(1) = Revolute('d', DH.d1, 'a', DH.a1, 'alpha', DH.alpha1);
L(2) = Revolute('d', DH.d2, 'a', DH.a2, 'alpha', DH.alpha2);
L(3) = Revolute('d', DH.d3, 'a', DH.a3, 'alpha', DH.alpha3);

L(1).qlim = [0 pi]; %Joint variable limits [min max]
L(2).qlim = [0 pi];
L(3).qlim = [0 pi];

%% Robot Definition

three_link = SerialLink(L); % to connect links together
three_link.name = 'Manipolatore';

save three_link three_link
save DH DH



%% Peter Corke Toolbox

% 1 - Forward Kinematics
% Tbe = three_link.fkine(q1);
% p_ee1 = Tbe.t;
% J = three_link.jacob0(q1);
% Jp = J(1:3,1:3);
% det(Jp)
% 
% 2 - Inverse Kinematics
% q1 = [0 0 pi/3];
% figure,three_link.plot(q1)
% figure,three_link.plot(q1)
% q2 = three_link.ikine(Tbe, 'mask', [1 1 1 0 0 0]);

% T = SE3(0.01,0,0)*SE3.rpy(0,0,0);
% q = 
% figure,three_link.plot(q)
% figure,three_link.plot(q2)


% Trajectory

T1 = SE3(0, 0, 0.10);
T2 = SE3(0, 0, 0.15);

q1 = three_link.ikine(T1, 'mask', [1 1 1 0 0 0]);
q2 = three_link.ikine(T2, 'mask', [1 1 1 0 0 0]);
t = [0:0.1:5]';
q = jtraj(q1, q2, t);
figure,three_link.plot(q)





