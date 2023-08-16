%% Data Figure

clc, clear all, close all;

%% Load information
%% Load information
load("h.mat");
load("hp.mat");
load("u_ref.mat");
load("t.mat");
des =1;

%% Load Data System Pose
h = h(:, des:end-1);

%% Load Data Velocities
hp = hp(:, des:end-1);
p = hp(4, :);
q = hp(5, :);
r = hp(6, :);

%% Load Time
t = t(:,des:end);

%% Length Simulation
N = length(t);

%% Reference Angles
phi_ref = u_ref(2, :);
theta_ref = u_ref(3, :);
euler_ref = [phi_ref;...
             theta_ref];
w_ref = u_ref(4, :);
%% Real Angles System
phi = h(8, :);
theta = h(9,:);
psi = h(10, :);

euler = [phi;...
         theta;...
          psi];

%% Angles velocities
for k =1:length(hp)
[euler_p(:, k)] = Euler_p(hp(4:6, k),h(8:10, k));
end
%% Angular Velocities Body euler
phi_p = euler_p(1, :);
theta_p = euler_p(2, :);
psi_p = euler_p(3, :);

%% generalized Data system
X = [euler(1:3,:);...
     euler_p(1:3,:)];

%% Control Signal
U_ref = [phi_ref;...
        theta_ref;...
         w_ref];

%% Rearrange data in order to develp DMD ext

X1 = [X(:,2:end-1);...
      X(:,1:end-2)];
  
X2 = [X(:,3:end);...
      X(:,2:end-1)];
  
Gamma = U_ref(:,2:end-1);

%% Parameter matrices
alpha = 0.5;
 %% Identificacion del TOrque
%% Parametros del optimizador
options = optimset('Display','iter',...
    'TolFun', 1e-8,...
    'MaxIter', 60000,...
    'Algorithm', 'active-set',...
    'FinDiffType', 'forward',...
    'RelLineSrchBnd', [],...
    'RelLineSrchBndDuration', 1,...
    'TolConSQP', 2e-8);

%% Initial Condition Optimization problem
x0=ones(1,180).*rand(1,180);
f_obj1 = @(x)  funcion_costo__DMD_koop(x, N, X1, X2, Gamma, alpha);
tic
%% Optimization Problem
x = fmincon(f_obj1,x0,[],[],[],[],[],[],[],options);
chi = x;
toc
%% Model Of the system
A = [ x(1), x(2),  x(3),  x(4),  x(5),  x(6),  x(7),  x(8),  x(9),  x(10), x(11), x(12);...
     x(13), x(14), x(15), x(16), x(17), x(18), x(19), x(20), x(21), x(22), x(23), x(24);...
     x(25), x(26), x(27), x(28), x(29), x(30), x(31), x(32), x(33), x(34), x(35), x(36);...
     x(37), x(38), x(39), x(40), x(41), x(42), x(43), x(44), x(45), x(46), x(47), x(48);...
     x(49), x(50), x(51), x(52), x(53), x(54), x(55), x(56), x(57), x(58), x(59), x(60);...
     x(61), x(62), x(63), x(64), x(65), x(66), x(67), x(68), x(69), x(70), x(71), x(72);...
     x(73), x(74), x(75), x(76), x(77), x(78), x(79), x(80), x(81), x(82), x(83), x(84);...
     x(85), x(86), x(87), x(88), x(89), x(90), x(91), x(92), x(93), x(94), x(95), x(96);...
     x(97), x(98), x(99), x(100), x(101), x(102), x(103), x(104), x(105), x(106), x(107), x(108);...
     x(109), x(110), x(111), x(112), x(113), x(114), x(115), x(116), x(117), x(118), x(119), x(120);...
     x(121), x(122), x(123), x(124), x(125), x(126), x(127), x(128), x(129), x(130), x(131), x(132);...
     x(133), x(134), x(135), x(136), x(137), x(138), x(139), x(140), x(141), x(142), x(143), x(144);...
     ];
 
B = [x(145), x(146), x(147);...
     x(148), x(149), x(150);...
     x(151), x(152), x(153);...
     x(154), x(155), x(156);...
     x(157), x(158), x(159);...
     x(160), x(161), x(162);...
     x(163), x(164), x(165);...
     x(166), x(167), x(168);...
     x(169), x(170), x(171);...
     x(172), x(173), x(174);...
     x(175), x(176), x(177);...
     x(178), x(179), x(180);...
     ];

C = [eye(6,6), zeros(6,6);...
     zeros(6,6), zeros(6,6)];

v_estimate(:, 1) = X1(:, 1);
for k= 1:length(X1)
    %% Output of the system
    salida_es(:, k) = C*v_estimate(:, k);
    salida_real(:, k) = C*X1(:, k);
    
    %% Error of the estimation
    error(:, k) = salida_real(:,k) - salida_es(:, k);
    norm_error(k) = norm(error(:, k), 2);
    
    
    %% Evolution of the system
    v_estimate(:, k+1) = A*v_estimate(:, k) + B*U_ref(:,k);
    
end

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(3,1,1)
plot(t(1:length(X2)),X1(1,1:length(X2)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
plot(t(1:length(X2)),salida_es(1,1:length(X2)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
grid on;
legend({'${{\psi}}$','$\hat{\psi}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Angles estimation}$','Interpreter','latex','FontSize',9);
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

subplot(3,1,2)
plot(t(1:length(X2)),X1(2,1:length(X2)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(t(1:length(X2)),salida_es(2,1:length(X2)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
legend({'${\theta}$','$\hat{\theta}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])
set(gcf, 'Color', 'w'); % Sets axes background

subplot(3,1,3)
plot(t(1:length(X2)),X1(3,1:length(X2)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(t(1:length(X2)),salida_es(3,1:length(X2)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
legend({'${\theta}$','$\hat{\theta}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

set(gcf, 'Color', 'w'); % Sets axes background
export_fig angles_estimation_koopman.pdf -q101

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(3,1,1)
plot(t(1:length(X2)),X1(4,1:length(X2)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
plot(t(1:length(X2)),v_estimate(4,1:length(X2)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
grid on;
legend({'$\dot{{\psi}}$','$\dot{\hat{\psi}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Angular velocity estimation}$','Interpreter','latex','FontSize',9);
ylabel('$[rad/s]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

subplot(3,1,2)
plot(t(1:length(X2)),X1(5,1:length(X2)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(t(1:length(X2)),v_estimate(5,1:length(X2)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
legend({'$\dot{\theta}$','$\dot{\hat{\theta}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad/s]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])
set(gcf, 'Color', 'w'); % Sets axes background

subplot(3,1,3)
plot(t(1:length(X2)),X1(6,1:length(X2)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(t(1:length(X2)),v_estimate(6,1:length(X2)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
legend({'${\dot{\theta}}$','$\dot{\hat{\theta}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[rad/s]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

set(gcf, 'Color', 'w'); % Sets axes background
export_fig omega_estimation_koopman.pdf -q101

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(1,1,1)
plot(t(1:length(X2)),norm_error(1,1:length(X2)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
legend({'$||e_{estimation}||$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Error estimation}$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])
set(gcf, 'Color', 'w'); % Sets axes background
export_fig nrom_estimation_koopman.pdf -q101