%% Data Figure

clc, clear all, close all;

%% Load information
load("full_ident_M100_16_b.mat");
des =1;

%% Load Data System Pose
h = [h(:, des:end-1):...
     quat(:, des:end-1)];

%% Load Data Velocities
hp = [v(:, des:end-1);...
      omega(:, des:end-1)];
  
p = hp(4, :);
q = hp(5, :);
r = hp(6, :);

%% Reference Angles
phi_ref = u_ref(2, :);
theta_ref = u_ref(3, :);
euler_ref = [phi_ref;...
             theta_ref];
         
%% Real Angles System
phi = euler(1, des:end-1);
theta = euler(2, des:end-1);
psi = euler(3, des:end-1);
euler_data = [phi;...
              theta;...
              psi];

%% Angular Velocities Body euler
phi_p = euler_p(1, des:end-1);
theta_p = euler_p(2, des:end-1);
psi_p = euler_p(3, des:end-1);

%% generalized Data system
X = [euler(1:3,:);...
     euler_p(1:3,:)];

%% Control Signal
U_ref = [phi_ref;...
        theta_ref;...
         u_ref(4,:)];

%% Rearrange data in order to develp DMD ext

X1 = [X(:,2:end-1);...
      X(:,1:end-2)];
  
X2 = X(:,3:end);
Gamma = U_ref(:,2:end-1);

%% Parameter matrices
alpha = 0.01;
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
%% Parametros del optimizador
options = optimset('Display','iter',...
    'TolFun', 1e-8,...
    'MaxIter', 60000,...
    'Algorithm', 'active-set',...
    'FinDiffType', 'forward',...
    'RelLineSrchBnd', [],...
    'RelLineSrchBndDuration', 1,...
    'TolConSQP', 2e-8);
%% Initial Parameterts
x0=ones(1,89).*rand(1,89);
f_obj1 = @(x)  funcion_costo__DMD_extend_delay(x, length(X2), X1, X2, Gamma, alpha);
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
     x(61), x(62), x(63), x(64), x(65), x(66), x(67), x(68), x(69), x(70), x(71), x(71);...
     ];
 
B = [x(72), x(73), x(74);...
     x(75), x(76), x(77);...
     x(78), x(79), x(80);...
     x(81), x(82), x(83);...
     x(84), x(85), x(86);...
     x(87), x(88), x(89)];


v_estimate_1 = X1(:, 1);
for k= 1:length(X2)
    v_estimate(:, k) = A*v_estimate_1 + B*U_ref(:,k);
    v_estimate_1 =  [ v_estimate(:, k); v_estimate_1(1:6)];
end

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(3,1,1)
plot(t(1:length(X2)),X2(1,1:length(X2)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
plot(t(1:length(X2)),v_estimate(1,1:length(X2)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
grid on;
legend({'${{\psi}}$','$\hat{\psi}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Identification signals and real Signals}$','Interpreter','latex','FontSize',9);
ylabel('$[Nm]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

subplot(3,1,2)
plot(t(1:length(X2)),X2(2,1:length(X2)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(t(1:length(X2)),v_estimate(2,1:length(X2)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
legend({'${\theta}$','$\hat{\theta}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[Nm]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

subplot(3,1,3)
plot(t(1:length(X2)),X2(3,1:length(X2)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(t(1:length(X2)),v_estimate(3,1:length(X2)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
legend({'${\psi}$','$\hat{\psi}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[Nm]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

set(gcf, 'Color', 'w'); % Sets axes background
export_fig Angles_estimation.pdf -q101


figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(3,1,1)
plot(t(1:length(X2)),X2(4,1:length(X2)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
plot(t(1:length(X2)),v_estimate(4,1:length(X2)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
grid on;
legend({'$\dot{{{\psi}}}$','$\dot{\hat{\psi}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Identification signals and real Signals}$','Interpreter','latex','FontSize',9);
ylabel('$[Nm]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

subplot(3,1,2)
plot(t(1:length(X2)),X2(5,1:length(X2)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(t(1:length(X2)),v_estimate(5,1:length(X2)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
legend({'$\dot{\theta}$','$\dot{\hat{\theta}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[Nm]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

subplot(3,1,3)
plot(t(1:length(X2)),X2(6,1:length(X2)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(t(1:length(X2)),v_estimate(6,1:length(X2)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
legend({'$\dot{{\psi}}$','$\dot{\hat{\psi}}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[Nm]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

set(gcf, 'Color', 'w'); % Sets axes background
export_fig Angular_velocity_estimation.pdf -q101