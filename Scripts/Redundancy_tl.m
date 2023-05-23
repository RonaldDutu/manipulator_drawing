clc
clear all
close all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% REDUNDANCY %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% In this script the user can define the initial positions of the joints
% and a trajectory to be mapped in the null space of the Jacobian.
% In this way it is possible to make the joints move, while keeping the
% end effector fixed. This is possible exploiting the redundancy of the
% three link manipulator.
%
% The animation of the robot moving is plotted.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

load three_link
load DH

%% Trajectory
ti = 0; %Initial time instant
tf = 1; %Final time instant
N = 100; %Number of points in the time interval
t = linspace(ti,tf,N)';
qi = [0 pi pi]; %Initial joints positions

qdot = 10*[ones(N,1) zeros(N,1) zeros(N,1)]; %Joints trajectory to be
% mapped in the null space of J

%% Redundancy
[q_n,qdot_n,v_r] = f_redundancy_tl(qi,qdot,t,DH);

%% Animation
% f_animation_tl(three_link,q_n,DH)