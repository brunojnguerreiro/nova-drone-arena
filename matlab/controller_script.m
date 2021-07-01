clear all;
clc;

%% Script with needed variables for the controller

zw = [0;0;1];
g = 9.81;


% Best gains for step references
% Kr = 22*eye(3);   %20
% Kw = 2*eye(3);    %2
% Kp = 15*eye(3);   %15
% Kv = 10*eye(3);   %10
I = [0.06 0.003 0.0006;0.003 0.05 0.002;0.002 0.003 0.1];
m = 1.5;

% Best gains for curve trajectories
Kr = 20*eye(3);   %20
Kw = 2*eye(3);    %2 
Kp = 15*eye(3);   %15
Kv = 10*eye(3);   %10

%%%%%%%%%%%%%%%%%%%%%%%%%% PX4 %%%%%%%%%%%%%%%%%%%%%%%%%%
% GAINS FOR PX4
% Kr = 6*eye(3);      %6
% Kw = 2*eye(3);      %2
% Kp = 1*eye(3);      %1
% Kv = 1.5*eye(3);    %1
% I = [0.029125 0 0; 0 0.029125 0; 0 0 0.055225];
% m = 1.9; %1.3
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


time = .1*(0:300)';

load('pos_ref.mat')
% Para fixar o tempo e o test n demorar muito tempo
for i=1:length(position_ref)
    position_ref.Time = position_ref.Time/5
end
%rosinit('http://192.168.1.222:11311');