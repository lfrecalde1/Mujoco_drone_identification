%% Data Figure
%% Clear variables
clc, clear all, close all;

%% Load information
load("h.mat");
load("hp.mat");
load("u_ref.mat");
load("t.mat");


p = hp(4, :);
q = hp(5, :);
r = hp(6, :);

%% Angles velocities
for k =1:length(t)
[euler_p(:, k)] = Euler_p(hp(4:6, k),h(8:10, k));
end

figure
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [4 2]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 10 4]);
subplot(3,1,1)
plot(t(1:length(t)),u_ref(3,1:length(t)),'-','Color',[226,76,44]/255,'linewidth',1); hold on
grid on;
plot(t(1:length(t)),h(9,1:length(t)),'--','Color',[226,76,44]/255,'linewidth',1); hold on
legend({'${\theta_d}$','$\theta$','$q$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
title('$\textrm{Identification signals and real Signals}$','Interpreter','latex','FontSize',9);
ylabel('$[Nm]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

subplot(3,1,2)
plot(t(1:length(t)),u_ref(2,1:length(t)),'-','Color',[46,188,89]/255,'linewidth',1); hold on
grid on;
plot(t(1:length(t)),h(8,1:length(t)),'--','Color',[46,188,89]/255,'linewidth',1); hold on
legend({'${\phi_d}$','$\phi$','$p$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[Nm]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])

subplot(3,1,3)
plot(t(1:length(t)),u_ref(4,1:length(t)),'-','Color',[46,100,89]/255,'linewidth',1); hold on
grid on;
plot(t(1:length(t)),euler_p(3,1:length(t)),'--','Color',[46,100,89]/255,'linewidth',1); hold on
legend({'${\psi_d}$','$\psi$','$p$'},'Interpreter','latex','FontSize',11,'Orientation','horizontal');
legend('boxoff')
ylabel('$[Nm]$','Interpreter','latex','FontSize',9);
xlim([0 t(end)])