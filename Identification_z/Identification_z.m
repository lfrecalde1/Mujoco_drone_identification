%% Data Figure

clc, clear all, close all;

%% Load information
load("Data_identification.mat");
des = 1;

%% Load Data System Pose
h = h(:, des:end-1);

%% Load Data Velocities
hp = hp(:, des:end-1);

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
x0=ones(1,4).*rand(1,4);
f_obj1 = @(x) funcion_costo_fz(x, N, u_ref, u, up,  F);

%% Optimization Problem
x = fmincon(f_obj1,x0,[],[],[],[],[],[],[],options);
chi = x;

%% Estimation Optimization Problem
for k=1:length(t)
    fz_est(k) =     x(1)*9.8 + x(2) * un_ref(k) - x(3)*un(k) + x(4)*un_p(k);  
end

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(1,1,1)
plot(t(1:length(fz)),fz(1,1:length(t)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(t(1:length(fz_est)),fz_est(1,1:length(t)),'--','Color',[100,50,40]/255,'linewidth',1); hold on
legend({'$f_z$','$\hat{f_z}$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Identification signals and real Signals}$','Interpreter','latex','FontSize',9);
ylabel('$[N]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])
