function [p_ee,A03] = position_ee_project(q,DH)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%This function computes the position of the end effector for a three link
%manipulator, given the joint positions and the Denavit Hartemberg 
%parameters.
% 
%p_ee = f_ee_position_tl(q,DH)
% 
%INPUTS
%q = [q1 q2 q3] Nx3 matrix of joints positions, where q1(Nx1) is an array
%of given joint 1 positions.
%DH struct with Denavit Hartenberg parameters.
% 
%OUTPUTS
%p_ee = [x_ee y_ee z_ee]  Nx3 matrix of end effector positions.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

a1 = DH.a1;
a2 = DH.a2;
a3 = DH.a3;

d1 = DH.d1;
d2 = DH.d2;
d3 = DH.d3;

alpha1 = DH.alpha1;
alpha2 = DH.alpha2;
alpha3 = DH.alpha3;

q1 = q(:,1);
q2 = q(:,2);
q3 = q(:,3);

p_ee = zeros(length(q1),3);

for i=1:length(q1)
    
%% Transformation matrix A03

A01 = [cos(q1(i)) -sin(q1(i))*cos(alpha1) sin(q1(i))*sin(alpha1) a1*cos(q1(i));
      sin(q1(i)) cos(q1(i))*cos(alpha1) -cos(q1(i))*sin(alpha1) a1*sin(q1(i));
      0 sin(alpha1) cos(alpha1) d1;
      0 0 0 1];
A12 = [cos(q2(i)) -sin(q2(i))*cos(alpha2) sin(q2(i))*sin(alpha2) a2*cos(q2(i));
      sin(q2(i)) cos(q2(i))*cos(alpha2) -cos(q2(i))*sin(alpha2) a2*sin(q2(i));
      0 sin(alpha2) cos(alpha2) d2;
      0 0 0 1];
A23 = [cos(q3(i)) -sin(q3(i))*cos(alpha3) sin(q3(i))*sin(alpha3) a3*cos(q3(i));
      sin(q3(i)) cos(q3(i))*cos(alpha3) -cos(q3(i))*sin(alpha3) a3*sin(q3(i));
      0 sin(alpha3) cos(alpha3) d3;
      0 0 0 1];
  

  
A02 = A01*A12;
A03 = A01*A12*A23; 

%% Geometric Jacobian

p0_4 = [0 0 0 1]'; 
p1_4 = A01*[0 0 0 1]'; 
p2_4 = A02*[0 0 0 1]';
p3_4 = A03*[0 0 0 1]';
z0_4 = [0 0 1 0]';
z1_4 = A01*[0 0 1 0]';
z2_4 = A02*[0 0 1 0]';
z3_4 = A03*[0 0 1 0]';

p0 = p0_4(1:3);
p1 = p1_4(1:3);
p2 = p2_4(1:3);
p3 = p3_4(1:3);

p_ee(i,:) = p3';








end

    