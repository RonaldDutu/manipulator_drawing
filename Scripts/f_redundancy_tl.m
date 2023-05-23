function [q_n,qdot_n,v_r] = f_redundancy_tl(qi,qdot,t,DH)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The purpose of this function is to show the redundant behavior of the
% three-link manipulator. Given a joint velocity trajectory to be mapped in
% the null space of J and an initial joint position, the function computes
% a joint velocity trajectory belonging to the null space of J. It is shown
% how joint move, but the end effector remains still.
%
% [q_n,qdot_n,v_r] = f_redundancy_tl(qi,qdot,t,DH)
%
% INPUTS
% qi: [qi1 qi2 qi3] (1x3) initial joint positions
% qdot: [qdot1 qdot2 qdot3] (1x3) joint velocity trajectory to be mapped in
% the null space
% t: time vector
% DH: Denavit Hartemberg parameters
%
% OUTPUTS
% qdot_n: [qdot_n1 qdot_n2 qdot_n3] (Nx3) joint velocities belonging to the
% null space of J
% q_n: [q_n1 q_n2 q_n3] (Nx3) joint positions obtained integrating qdot_n
% v_r: (6xN) velocity of the end effector when joints move with velocity
% qdot_n.
% It should be 0.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

N = length(t);
ti = t(1);
tf = t(N);
dt = (tf-ti)/(N-1);
q_n(1,:) = qi;

for i=1:N-1
J = f_jcb_tl(q_n(i,:),DH);
qdot_n(:,i) = (eye(3)-pinv(J)*J)*(qdot(i,:))';
v_r(:,i) = J*qdot_n(:,i);
q_n(i+1,:) = q_n(i,:)+qdot_n(:,i)'*dt;
end

figure
subplot(3,1,1)
plot(q_n(:,1),'r')
grid on
title('q1 [rad]')
subplot(3,1,2)
plot(q_n(:,2),'b')
grid on
title('q2 [rad]')
subplot(3,1,3)
plot(q_n(:,3),'g')
grid on
title('q3 [rad]')
xlabel('t [s]')
end