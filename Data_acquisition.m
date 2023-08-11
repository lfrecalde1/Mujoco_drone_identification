%% Syten Data Adquisition Mujoco

%% Clear Variables
clc, clear all, close all;

%% Load Values of the desired Signals
load('Signals.mat')

%% Reference Signals
ul_ref = Signals(1,:);
um_ref = Signals(2,:);
un_ref = Signals(3,:);
w_ref = Signals(4,:);

%% Definition Sample Time
ts = 0.05;
t = time_simulation(ul_ref, ts);

%% Ros Configuration
rosshutdown
rosinit('192.168.1.106', 'NodeHost', '192.168.1.106', 'Nodename', '/Matlab');

%% Ros topics names
robot_references = rospublisher('/cmd_vel');
velmsg = rosmessage(robot_references);
odom = rossubscriber('/odom');
inputs = rossubscriber('/input_ref');
%% Initial Conditions System
h = zeros(7, length(t)+1);
hp = zeros(6, length(t)+1);
T = zeros(3, length(t));
F = zeros(3, length(t));
%% Get data system
[h(:, 1), hp(:, 1)] = odometry(odom);

for k=1:1:length(t)
    tic; 
    %% SEND VALUES OF CONTROL ROBOT
    send_velocities(robot_references, velmsg, [ul_ref(k), um_ref(k), un_ref(k), 0, 0 , w_ref(k)]);
    [F(:, k), T(:, k)] =  inputs_system(inputs);
    
    %% GET VALUES OF DRONE
    [h(:, k+1), hp(:,k+1)] = odometry(odom);
    
    while(toc<ts)
    end
    toc;
end
send_velocities(robot_references, velmsg, [0, 0, 0, 0, 0 , 0])
rosshutdown;

%% Save Data System
save("Data_identification.mat", "ts", "t", "ul_ref", "um_ref", "un_ref", "w_ref", "h", "hp", "T", "F")