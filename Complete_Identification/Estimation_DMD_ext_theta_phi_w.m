%% Data Figure

clc, clear all, close all;

%% Load information
load("Data_identification.mat");
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

%% Filter Parameters
landa = 30;%lambda
F1=tf(landa,[1 landa]);

%% Reference Signals Velocities Body
ul_ref = ul_ref(1, des:end);
um_ref = um_ref(1, des:end);
un_ref = un_ref(1, des:end);

%% Filter Velocities Body
ul_ref_f=lsim(F1,ul_ref,t)';
um_ref_f=lsim(F1,um_ref,t)';
un_ref_f=lsim(F1,un_ref,t)';

%% General vector velocities body
u_ref = [ul_ref;um_ref;un_ref];
%% General vector velocities body 
u_ref_f = [ul_ref_f; um_ref_f; un_ref_f];



%% Real velocities Body
ul = hp(1, :);
um = hp(2, :);
un = hp(3, :);




%% General Vector Velocities Body
u = [ul; um; un];


%% Get Aceleration System Body
for k=1:length(t)
    if k>1
        p_p(k)=(p(k)- p(k-1))/ts;
        q_p(k)=(q(k)- q(k-1))/ts;
        r_p(k)=(r(k)- r(k-1))/ts;     
    else
        p_p(k)=0;
        q_p(k)=0;
        r_p(k)=0;
    end
end

%% General Vector Angular Velocities
wp = [p_p; q_p; r_p];

%% Get Aceleration Angular System Body
for k=1:length(t)
    if k>1
        ul_p(k)=(ul(k)- ul(k-1))/ts;
        um_p(k)=(um(k)- um(k-1))/ts;
        un_p(k)=(un(k)- un(k-1))/ts;     
    else
        ul_p(k)=0;
        um_p(k)=0;
        un_p(k)=0;
        
    end
end



%% General Vector Aceleration Body
up = [ul_p; um_p; un_p];



%% Split Forces 
fx = F(1, des:end);
fy = F(2, des:end);
fz = F(3, des:end);



%% General Vector Forces
F = [fx;fy;fz];


%% Split Torques
tx = T(1, des:end);
ty = T(2, des:end);
tz = T(3, des:end);


%% General Vector Torques
Tau = [tx;ty;tz];

%% Reference Angles
phi_ref = omega_ref(1, :);
theta_ref = omega_ref(2, :);
euler_ref = [phi_ref;...
             theta_ref];
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
%% Angular Aceleration theta

for k=1:length(t)
    if k>1
        theta_pp(k)=(theta_p(k)- theta_p(k-1))/ts;
        phi_pp(k)=(phi_p(k)- phi_p(k-1))/ts;
        psi_pp(k)=(psi_p(k)- psi_p(k-1))/ts;
    else
        theta_pp(k)=0;
        phi_pp(k)=0;
        psi_pp(k)=0;
    end
end

euler_pp = [phi_pp;...
             theta_pp;...
             psi_pp];

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
alpha = 0.02;
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
x0=ones(1,53).*rand(1,53);
f_obj1 = @(x)  funcion_costo__DMD_extend(x, N, X1, X2, Gamma, alpha);

%% Optimization Problem
x = fmincon(f_obj1,x0,[],[],[],[],[],[],[],options);
chi = x;

%% Model Of the system
A = [x(1), x(2), x(3), x(4), x(5), x(6);...
     x(7), x(8), x(9), x(10), x(11), x(12);...
     x(13), x(14), x(15), x(16), x(17), x(18);...
     x(19), x(20), x(21), x(22), x(23), x(24);...
     x(25), x(26), x(27), x(28), x(29), x(30);...
     x(31), x(32), x(33), x(34), x(35), x(36)];
 
B = [x(37), x(38), x(39);...
     x(40), x(41), x(42);...
     x(43), x(44), x(45);...
     x(46), x(47), x(48);...
     x(49), x(50), x(51);...
     x(52), x(53), x(53)];
 
v_estimate(:, 1) = X(:, 1);
for k= 1:length(t)
    v_estimate(:, k+1) = A*v_estimate(:, k)+B*U_ref(:,k);
end

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(2,1,1)
plot(t(1:length(ul_ref)),euler_p(3,1:length(t)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
plot(t(1:length(ul_ref)),v_estimate(6,1:length(t)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
grid on;
legend({'${\dot{\psi}}$','$\hat{\psi}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Identification signals and real Signals}$','Interpreter','latex','FontSize',9);
ylabel('$[Nm]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

subplot(2,1,2)
plot(t(1:length(ul_ref)),psi(1,1:length(t)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(t(1:length(ul_ref)),v_estimate(3,1:length(t)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
legend({'${\psi}$','$\hat{\psi}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[Nm]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])
set(gcf, 'Color', 'w'); % Sets axes background
export_fig omega_estimation.pdf -q101

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(2,1,1)
plot(t(1:length(ul_ref)),theta(1,1:length(t)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
plot(t(1:length(ul_ref)),v_estimate(2,1:length(t)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
grid on;
legend({'${\theta}$','$\hat{\theta}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Identification signals and real Signals}$','Interpreter','latex','FontSize',9);
ylabel('$[Nm]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

subplot(2,1,2)
plot(t(1:length(ul_ref)),phi(1,1:length(t)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(t(1:length(ul_ref)),v_estimate(1,1:length(t)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
legend({'${\phi}$','$\hat{\phi}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[Nm]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])
set(gcf, 'Color', 'w'); % Sets axes background
export_fig phi_theta_estimation.pdf -q101

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(2,1,1)
plot(t(1:length(ul_ref)),theta_p(1,1:length(t)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
plot(t(1:length(ul_ref)),v_estimate(5,1:length(t)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
grid on;
legend({'${\theta}$','$\hat{\theta}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Identification signals and real Signals}$','Interpreter','latex','FontSize',9);
ylabel('$[Nm]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

subplot(2,1,2)
plot(t(1:length(ul_ref)),phi_p(1,1:length(t)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(t(1:length(ul_ref)),v_estimate(4,1:length(t)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
legend({'${\phi}$','$\hat{\phi}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[Nm]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])
set(gcf, 'Color', 'w'); % Sets axes background
export_fig phi_theta__velocities_estimation.pdf -q101