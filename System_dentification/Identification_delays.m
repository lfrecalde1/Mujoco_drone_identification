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

X1 = X(:,1:end-1);
  
X2 = X(:,2:end);
Gamma = U_ref(:,1:end-1);

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
x0=ones(1,54).*rand(1,54);
f_obj1 = @(x)  funcion_costo__DMD_extend_delay(x, N, X1, X2, Gamma, alpha);
tic
%% Optimization Problem
x = fmincon(f_obj1,x0,[],[],[],[],[],[],[],options);
chi = x;
toc
%% Model Of the system
A = [ x(1), x(2),  x(3),  x(4),  x(5),  x(6);...
     x(7),  x(8),  x(9),  x(10), x(11), x(12);...
     x(13), x(14), x(15), x(16), x(17), x(18);...
     x(19), x(20), x(21), x(22), x(23), x(24);...
     x(25), x(26), x(27), x(28), x(29), x(30);...
     x(31), x(32), x(33), x(34), x(35), x(36);...
     ];
 
B = [x(37), x(38), x(39);...
     x(40), x(41), x(42);...
     x(43), x(44), x(45);...
     x(46), x(47), x(48);...
     x(49), x(50), x(51);...
     x(52), x(53), x(54)];
 
C = eye(6,6);
%% Initial conditions System
v_estimate(:, 1) = X1(:, 1);
for k= 1:length(X2)
    %% Output of the system
    salida_es(:, k) = C*v_estimate(:, k);
    salida_real(:, k) = C*X1(:, k);
    
    %% Error of the estimation
    error(:, k) = salida_real(:,k) - salida_es(:, k);
    norm_error(k) = norm(error(:, k), 2);
    %% System evolution
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
export_fig angles_estimation_simple.pdf -q101

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
export_fig omega_estimation_simple.pdf -q101

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
export_fig nrom_estimation_simple.pdf -q101