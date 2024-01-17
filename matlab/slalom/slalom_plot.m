clear;
close all
%% 測定値を格納
data = csvread("data_turn90_1.csv");
data_x = data(:,1);
data_y = data(:,2);
data_theta = data(:,3);
data_v = data(:,4);
data_omega = data(:,5);
data_kanayama_v = data(:,6);
data_kanayama_w = data(:,7);
%% 目標値を格納
ref = csvread("ref_turn90_1.csv");
ref_x = ref(:,1);
ref_y = ref(:,2);
ref_theta = ref(:,3);
ref_v = ref(:,4);
ref_omega = ref(:,5);
%% plot ref
f1 = figure(1);  
set(f1, 'position', get(0, 'screensize'))

subplot(3,2,[1,3,5]);
plot(ref_x,ref_y,'LineWidth',3);
grid on;
pbaspect([1 1 1]);
xlabel('$x$ [m]','Interpreter','latex','FontSize',25);
ylabel('$y$ [m]','Interpreter','latex','FontSize',25);
legend('ref','Interpreter','latex','Location','northwest','FontSize',20)
h_axes = gca;
h_axes.XAxis.FontSize = 25;
h_axes.YAxis.FontSize = 25;

subplot(3,2,2);
plot(ref_v,'LineWidth',3);
grid on;
xlabel('Time [ms]','Interpreter','latex','FontSize',25);
ylabel('$v$ [m/s]','Interpreter','latex','FontSize',25);
xlim([0 length(ref_v)])
legend('ref','Interpreter','latex','Location','northwest','FontSize',20)
h_axes = gca;
h_axes.XAxis.FontSize = 25;
h_axes.YAxis.FontSize = 25;

subplot(3,2,4);
plot(ref_theta,'LineWidth',3);
grid on;
xlabel('Time [ms]','Interpreter','latex','FontSize',25);
ylabel('$\theta$ [rad]','Interpreter','latex','FontSize',25);
xlim([0 length(ref_theta)])
legend('ref','Interpreter','latex','Location','northwest','FontSize',20)
h_axes = gca;
h_axes.XAxis.FontSize = 25;
h_axes.YAxis.FontSize = 25;

subplot(3,2,6);
plot(ref_omega,'LineWidth',3);
grid on;
xlabel('Time [ms]','Interpreter','latex','FontSize',25);
ylabel('$\omega$ [rad/s]','Interpreter','latex','FontSize',25);
xlim([0 length(ref_theta)])
legend('ref','Interpreter','latex','Location','northwest','FontSize',20)
h_axes = gca;
h_axes.XAxis.FontSize = 25;
h_axes.YAxis.FontSize = 25;
%% plot data
f2 = figure(2);  
set(f2, 'position', get(0, 'screensize'))

subplot(3,2,[1,3,5]);
plot(data_x,data_y,'LineWidth',3);
grid on;
hold on;
plot(ref_x,ref_y,'LineWidth',3);
pbaspect([1 1 1]);
xlabel('$x$ [m]','Interpreter','latex','FontSize',25);
ylabel('$y$ [m]','Interpreter','latex','FontSize',25);
legend('data','ref','Interpreter','latex','Location','northwest','FontSize',20)
h_axes = gca;
h_axes.XAxis.FontSize = 25;
h_axes.YAxis.FontSize = 25;

subplot(3,2,2);
plot(data_v,'LineWidth',3);
grid on;
hold on;
plot(data_kanayama_v,'LineWidth',3);
xlabel('Time [ms]','Interpreter','latex','FontSize',25);
ylabel('$v$ [m/s]','Interpreter','latex','FontSize',25);
xlim([0 length(data_v)])
legend('data','kanayama','Interpreter','latex','Location','northwest','FontSize',20)
h_axes = gca;
h_axes.XAxis.FontSize = 25;
h_axes.YAxis.FontSize = 25;

subplot(3,2,4);
plot(data_theta,'LineWidth',3);
grid on;
xlabel('Time [ms]','Interpreter','latex','FontSize',25);
ylabel('$\theta$ [rad]','Interpreter','latex','FontSize',25);
xlim([0 length(data_theta)])
legend('data','Interpreter','latex','Location','northwest','FontSize',20)
h_axes = gca;
h_axes.XAxis.FontSize = 25;
h_axes.YAxis.FontSize = 25;

subplot(3,2,6);
plot(data_omega,'LineWidth',3);
grid on;
hold on;
plot(data_kanayama_w,'LineWidth',3);
xlabel('Time [ms]','Interpreter','latex','FontSize',25);
ylabel('$\omega$ [rad/s]','Interpreter','latex','FontSize',25);
xlim([0 length(data_theta)])
legend('data','kanayama','Interpreter','latex','Location','northwest','FontSize',20)
h_axes = gca;
h_axes.XAxis.FontSize = 25;
h_axes.YAxis.FontSize = 25;