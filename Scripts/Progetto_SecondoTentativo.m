%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Anthropomorphic robotic arm for drawing applications - Robotics Project     %  
% Academic Year 2020/2021                                                     %  
% Code made by:                                                               %  
% Federica Parisi – 289819                                                    %  
% Martina Bonaffini – 289563                                                  %  
% Nicola Occhipinti – 289648                                                  %  
% Ronald Cristian Dutu – 290185                                               %  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


close all, clear all, clc

%% DH Parameters

DH.a1 = 0; 
DH.a2 = 0.085;                      
DH.a3 = 0.154;                      
DH.d1 = 0.058;        
DH.d2 = 0;
DH.d3 = 0;
DH.alpha1 = pi/2;
DH.alpha2 = 0;
DH.alpha3 = 0;

%% Links definition

L(1) = Revolute('d', DH.d1, 'a', DH.a1, 'alpha', DH.alpha1);
L(2) = Revolute('d', DH.d2, 'a', DH.a2, 'alpha', DH.alpha2);
L(3) = Revolute('d', DH.d3, 'a', DH.a3, 'alpha', DH.alpha3);

% L(2).offset = 0.3;
% L(3).offset = 0.5; 

L(1).qlim = [0 pi]; 
L(2).qlim = [0 pi];
L(3).qlim = [0 pi];

%% Robot Definition

four_link = SerialLink(L); % to connect links together
four_link.name = 'Manipolatore';

save four_link four_link
save DH DH


%% 1 - Forward Kinematics
q1 = [0,pi/2,pi/2];
Tbe = four_link.fkine(q1);
pe = Tbe.t;
figure,four_link.plot(q1)

%% 2 - Inverse Kinematics
q2 = four_link.ikine(Tbe, 'mask', [1 1 1 0 0 0]);
figure,four_link.plot(q2)


%% 3 - Singularity
% Elbow totally extended
q3 = [0,0,0];
J3 = four_link.jacob0(q3);
Jp3 = J3(1:3,1:3);
det(Jp3)
figure,four_link.plot(q3)

% Elbow totally retracted
q4 = [0,0,pi];
J4 = four_link.jacob0(q4);
Jp4 = J4(1:3,1:3);
det(Jp4)
figure,four_link.plot(q4)

% Shoulder singularity
theta2 = 0.425;
theta3 = acos(-DH.a2/DH.a3*cos(theta2))-theta2;
q5 = [0,theta2,theta3];
J5 = four_link.jacob0(q5);
Jp5 = J5(1:3,1:3);
det(Jp5)
figure,four_link.plot(q5)

return

%% 4 - Trajectory

T1 = SE3(0,0,0.15);
T2 = SE3(0,0,0.2);

% the following code computes the vertical part of the trajectory
q1 = four_link.ikine(T1, 'mask', [1 1 1 0 0 0]);
q2 = four_link.ikine(T2, 'mask', [1 1 1 0 0 0]);
t = [0:0.1:8]';
q = jtraj(q1, q2, t);
figure(2),four_link.plot(q)

a=arduino('COM11', 'Uno')
s=servo(a,'D9')
s1=servo(a,'D6')
s2=servo(a,'D5')
writePosition(s,0)
readPosition(s)
writePosition(s1,0)
readPosition(s1)
writePosition(s2,0)
readPosition(s2)

pause(2)

for i=1:81
    
    writePosition(s,0.2+q(i,1)/(pi))
    readPosition(s)
    writePosition(s1,0.2 - q(i,2)/(pi))
    readPosition(s1)
    writePosition(s2,1-q(i,3)/(2*pi))
    readPosition(s2)
  
end

% the following code computes the horizontal part of the L letter

writePosition(s,0.18)
readPosition(s)
writePosition(s1,0.2 - 0.1527/(pi))
readPosition(s1)
writePosition(s2,1-1.4951/(2*pi))
readPosition(s2)
pause(0.5)
writePosition(s,0.14)
readPosition(s)
pause(0.5)
writePosition(s,0.10)
readPosition(s)
pause(0.5)
writePosition(s,0.08)
readPosition(s)
    
    
