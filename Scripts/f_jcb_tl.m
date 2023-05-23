function [Jg,Ja] = f_jcb_tl(q,DH)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function computes the jacobian matrix for a three link planar
% manipulator, given the joint positions and the Denavit Hartemberg
% parameters.
% 
% J = f_jcb_tl(q,DH)
% 
% INPUTS
% q = [q1 q2 q3] 1x3 vector of joints position.
% DH: struct with Denavit-Hartenberg parameters
% 
% OUTPUTS
% J: Jacobian matrix 6x3
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% internal robot parameters definition

a1 = DH.a1;
a2 = DH.a2;
a3 = DH.a3;
d1 = DH.d1;
d2 = DH.d2;
d3 = DH.d3;
q1 = q(1);
q2 = q(2);
q3 = q(3);

%% Transformation matrix A03

A01 = [cos(q1) -sin(q1) 0 a1*cos(q1);
      sin(q1) cos(q1) 0 a1*sin(q1);
      0 0 1 d1;
      0 0 0 1];
A12 = [cos(q2) -sin(q2) 0 a2*cos(q2);
      sin(q2) cos(q2) 0 a2*sin(q2);
      0 0 1 d2;
      0 0 0 1];
A23 = [cos(q3) -sin(q3) 0 a3*cos(q3);
      sin(q3) cos(q3) 0 a3*sin(q3);
      0 0 1 d3;
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
z0 = z0_4(1:3);
z1 = z1_4(1:3);
z2 = z2_4(1:3);
z3 = z3_4(1:3);

J11 = cross(z0,p3-p0);
J12 = cross(z1,p3-p1);
J13 = cross(z2,p3-p2);
J21 = z0;
J22 = z1;
J23 = z2;
Jg = [J11 J12 J13; 
     J21 J22 J23];
 

%% Analytical Jacobian

phi = pi;
theta = pi/2;
psi = pi;

A_gamma = eul2jac(phi, theta, psi);

Ja = [eye(3) zeros(3); zeros(3) inv(A_gamma)]*Jg;

end