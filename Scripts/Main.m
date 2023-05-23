%%%%%%%%%%%%%%%%%%%%%%%%%%   ROBOT DEFINITION   %%%%%%%%%%%%%%%%%%%%%%%%%%%
% In this file we set all parameters of a three link manipulator, saving
% them in the DH.mat and the three_link.mat files. 
% They will be used in the other functions.
% three_link.mat is a SerialLink file, defining the robot.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clear  
clc

%% DH Parameters

DH.a1 = 0; 
DH.a2 = 0.085;                      % 8.5 cm = 0.085 m
DH.a3 = 0.154;                      % 15.4 cm = 0.154 m
DH.d1 = 0.058;                      % 5.8 cm = 0.058 m
DH.d2 = 0;
DH.d3 = 0;
DH.alpha1 = pi/2;
DH.alpha2 = 0;
DH.alpha3 = 0;

%% Links definition

L(1) = Revolute('d', DH.d1, 'a', DH.a1, 'alpha', DH.alpha1);
L(2) = Revolute('d', DH.d2, 'a', DH.a2, 'alpha', DH.alpha2);
L(3) = Revolute('d', DH.d3, 'a', DH.a3, 'alpha', DH.alpha3);

% L(1).qlim = [-pi/2 pi/2]; %Joint variable limits [min max]
% L(2).qlim = [-pi pi/2];
% L(3).qlim = [-pi pi/2];

%% Robot Definition

three_link = SerialLink(L);
three_link.name = 'Manipolatore';

save three_link three_link
save DH DH

q1 = 0;
q2 = 0.425; %singolarità a 0.425
q3 = acos(-DH.a2/DH.a3*cos(q2))-q2;
qv1 = [q1 q2 q3];


figure,three_link.plot(qv1)

%% Peter Corke Toolbox

% 1 - Forward Kinematics
Tbe = three_link.fkine(qv1);
p_ee1 = Tbe.t;
J = three_link.jacob0(qv1);
Jp = J(1:3,1:3);
det(Jp)
jsingu(Jp)

% qv2 = three_link.ikine(Tbe, 'mask', [1 1 1 0 0 0]);

% 2 - Inverse Kinematics
% q2 = three_link.ikine(Tbe, 'mask', [1 1 1 0 0 0]);
% 
% figure,three_link.plot(q2)
% qd = inv(Jp)*[0.1 0.2 0.3]';


% Trajectory

% qi = [0 pi/2 0];
% qf = [0 3/4*pi pi];
% 
% 
% Ti = three_link.fkine(qi);
% Tf = three_link.fkine(qf);
% 
% qi = three_link.ikine(Ti, 'mask', [1 1 1 0 0 0]);
% qf = three_link.ikine(Tf, 'mask', [1 1 1 0 0 0]);
% t = (0:0.05:2)';
% [q,qd,qdd] = jtraj(qi, qf, t);
% 
% figure,three_link.plot(q)
% 
% [q,qd,qdd] = jtraj(qf, qi, t);
% three_link.plot(q)

% load hershey
% 
% P = hershey{'P'};
% path = [P.stroke; zeros(1,numcols(P.stroke))];
% k = find(isnan(path(1,:)));
% path(:,k) = path(:,k-1); path(3,k) = 0.2;
% traj = mstraj(path(:,2:end)', [1 1 1], [], path(:,1)',0.2, 1);
% about(traj)
% plot3(traj(:,1), traj(:,2), traj(:,3))
% Tp = SE3(0.6, 0, 0) * SE3(traj) * SE3.oa( [0 1 0], [0 0 -1]);
% q = three_link.ikine(Tp, 'mask', [1 1 1 0 0 0]);
% figure,three_link.plot(q)




% Custom toolbox
% [p_ee2,A03] = position_ee_project(q2,DH); % computes the position of the end effector
% [Jg,Ja] = f_jcb_tl(q,DH); sbagliato, da rifare
% rank(Jg)
% jsingu(Jg)
% det = det(Jg(1:3,1:3));


