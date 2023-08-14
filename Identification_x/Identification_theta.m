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


%% Filter Body Velocities
ul_f=lsim(F1,ul,t)';
um_f=lsim(F1,um,t)';
un_f=lsim(F1,un,t)';

%% General Vector Velocities Body
u = [ul; um; un];
u_f = [ul_f; um_f; un_f];

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

%% Filter Aceleration Body
ul_p_f=lsim(F1,ul_p,t)';
um_p_f=lsim(F1,um_p,t)';
un_p_f=lsim(F1,un_p,t)';

%% General Vector Aceleration Body
up = [ul_p; um_p; un_p];
up_f = [ul_p_f; um_p_f; un_p_f];


%% Split Forces 
fx = F(1, des:end);
fy = F(2, des:end);
fz = F(3, des:end);

%% Forces Filter
fx_f=lsim(F1,fx,t)';
fy_f=lsim(F1,fy,t)';
fz_f=lsim(F1,fz,t)';

%% General Vector Forces
F = [fx;fy;fz];
F_f = [fx_f; fy_f; fz_f];

%% Split Torques
tx = T(1, des:end);
ty = T(2, des:end);
tz = T(3, des:end);
%% Torques Filter
tx_f=lsim(F1,tx,t)';
ty_f=lsim(F1,ty,t)';
tz_f=lsim(F1,tz,t)';

%% General Vector Torques
Tau = [tx;ty;tz];
Tau_f = [tx_f; ty_f; tz_f];

%% Reference Angles
phi_ref = omega_ref(1, :);
theta_ref = omega_ref(2, :);

euler_ref = [phi_ref;...
             theta_ref];
%% Real Angles System
phi = h(8, :);
theta = h(9,:);

euler = [phi;...
         theta];

%% Angles velocities
for k =1:length(t)
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
    else
        theta_pp(k)=0;
        phi_pp(k)=0;
    end
end

euler_pp = [phi_pp;...
             theta_pp];
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
x0=ones(1,16).*rand(1,16);
f_obj1 = @(x) funcion_costo_theta(x, N, euler_ref, euler, euler_p, euler_pp,  Tau);

%% Optimization Problem
x = fmincon(f_obj1,x0,[],[],[],[],[],[],[],options);
chi = x;
chi_pid = [0.0006    0.0012   -0.0018    0.0009];
%% Estimation Optimization Problem
for k=1:length(t)
    ty_est(k) = chi_pid(1)*(theta_ref(k)) - chi_pid(2)*(theta(k))  + chi_pid(3)*theta_p(k) + chi_pid(4)*theta_pp(k); 
    theta_pp_estimado(k) =  (-x(1)/x(2))*phi_p(k)*psi_p(k) - (x(2)/x(2))*phi_p(k)*psi_p(k) + (x(3)/x(2))*phi_p(k)*psi_p(k) + (1/x(2))*ty_est(k);
    theta_pp_estimado_f(k) =  (-x(1)/x(2))*(1/(x(3)/x(4)))*phi_p(k)*psi_p(k) - (x(5)/x(6))*(1/(x(7)/x(8)))*phi_p(k)*psi_p(k) + (x(9)/x(10))*(1/(x(11)/x(12)))*phi_p(k)*psi_p(k) + (1/x(13))*(x(14)*(theta_ref(k)) - x(15)*(theta(k))  + x(16)*theta_p(k));

end

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(2,1,1)
plot(t(1:length(ul_ref)),theta_p(1,1:length(t)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
plot(t(1:length(ul_ref)),theta(1,1:length(t)),'-','Color',[100,76,44]/255,'linewidth',1); hold on
plot(t(1:length(ul_ref)),q(1,1:length(t)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
grid on;
legend({'$\dot{\theta}$','${\theta}$','$q$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Identification signals and real Signals}$','Interpreter','latex','FontSize',9);
ylabel('$[Nm]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

subplot(2,1,2)
plot(t(1:length(ul_ref)),theta_pp(1,1:length(t)),'-','Color',[46,188,89]/255,'linewidth',1); hold on
grid on;
plot(t(1:length(ul_ref)),q_p(1,1:length(t)),'--','Color',[46,100,89]/255,'linewidth',1); hold on
legend({'${\ddot{\theta}}$','$\dot{q}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[Nm]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(2,1,1)
plot(t(1:length(ul_ref)),phi_p(1,1:length(t)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
plot(t(1:length(ul_ref)),phi(1,1:length(t)),'-','Color',[100,76,44]/255,'linewidth',1); hold on
plot(t(1:length(ul_ref)),p(1,1:length(t)),'--','Color',[100,76,10]/255,'linewidth',1); hold on
grid on;
legend({'$\dot{\phi}$','${\phi}$','$p$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Identification signals and real Signals}$','Interpreter','latex','FontSize',9);
ylabel('$[Nm]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

subplot(2,1,2)
plot(t(1:length(ul_ref)),phi_pp(1,1:length(t)),'-','Color',[46,188,89]/255,'linewidth',1); hold on
grid on;
plot(t(1:length(ul_ref)),p_p(1,1:length(t)),'--','Color',[46,100,89]/255,'linewidth',1); hold on
legend({'${\ddot{\phi}}$','$\dot{p}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[Nm]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(1,1,1)
plot(t(1:length(ty)),ty(1,1:length(t)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(t(1:length(ty_est)),ty_est(1,1:length(t)),'--','Color',[100,50,40]/255,'linewidth',1); hold on
legend({'$t_y$','$\hat{t_y}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Identification signals and real Signals}$','Interpreter','latex','FontSize',9);
ylabel('$[N]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(1,1,1)
plot(t(1:length(ty)),theta_pp(1,1:length(t)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(t(1:length(ty_est)),theta_pp_estimado_f(1,1:length(t)),'--','Color',[100,50,40]/255,'linewidth',1); hold on
legend({'${\ddot{\theta}}$','$\hat{\theta}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Identification signals and real Signals}$','Interpreter','latex','FontSize',9);
ylabel('$[N]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])