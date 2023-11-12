clear;
close all
%% 測定値を格納
data = csvread("data.csv");
data_x = data(:,1);
data_y = data(:,2);
data_theta = data(:,3);
data_v = data(:,4);
data_omega = data(:,5);
data_kanayama_v = data(:,6);
data_kanayama_w = data(:,7);
%% 目標値を格納
ref = csvread("ref.csv");
ref_x = ref(:,1);
ref_y = ref(:,2);
ref_theta = ref(:,3);
ref_v = 0.5064989 * ones(length(data_x),1);
ref_omega = ref(:,4);
%% plot
f1 = figure(1);  
set(f1, 'position', get(0, 'screensize'))

subplot(3,2,[1,3,5]);
plot(data_x,data_y,'LineWidth',3);
grid on;
hold on;
plot(ref_x,ref_y,'LineWidth',3);
pbaspect([1 1 1]);
xlabel('$x$ [m]','Interpreter','latex','FontSize',25);
ylabel('$y$ [m]','Interpreter','latex','FontSize',25);
legend('data','ref','Interpreter','latex','Location','northwest','FontSize',20)
set(gca, "FontName", "Times New Roman", "FontSize", 25);

subplot(3,2,2);
plot(data_v,'LineWidth',3);
grid on;
hold on;
plot(ref_v,'LineWidth',3);
plot(data_kanayama_v,'LineWidth',3);
xlabel('Time [ms]','Interpreter','latex','FontSize',25);
ylabel('$v$ [m/s]','Interpreter','latex','FontSize',25);
xlim([0 length(data_v)])
legend('data','ref','kanayama','Interpreter','latex','Location','northwest','FontSize',20)
set(gca, "FontName", "Times New Roman", "FontSize", 25);

subplot(3,2,4);
plot(data_theta,'LineWidth',3);
grid on;
hold on;
plot(ref_theta,'LineWidth',3);
xlabel('Time [ms]','Interpreter','latex','FontSize',25);
ylabel('$\theta$ [rad]','Interpreter','latex','FontSize',25);
xlim([0 length(data_theta)])
legend('data','ref','Interpreter','latex','Location','northwest','FontSize',20)
set(gca, "FontName", "Times New Roman", "FontSize", 25);

subplot(3,2,6);
plot(data_omega,'LineWidth',3);
grid on;
hold on;
plot(ref_omega,'LineWidth',3);
plot(data_kanayama_w,'LineWidth',3);
xlabel('Time [ms]','Interpreter','latex','FontSize',25);
ylabel('$\omega$ [rad/s]','Interpreter','latex','FontSize',25);
xlim([0 length(data_theta)])
legend('data','ref','kanayama','Interpreter','latex','Location','northwest','FontSize',20)
set(gca, "FontName", "Times New Roman", "FontSize", 25);