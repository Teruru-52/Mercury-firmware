clear;

%% load log
% data = csvread("data_kanayama.csv");
data = csvread("data_df.csv");
% data = csvread("data_tvf.csv");
% data = csvread("data_bug.csv");
data_x = data(:,1);
data_y = data(:,2);
data_theta = data(:,3);
data_v = data(:,4);
data_omega = data(:,5);
data_a = data(:,6);

ref_x = data(:,7);
ref_y = data(:,8);
ref_theta = data(:,9);
ref_v = data(:, 10);
ref_omega = data(:,11);
ref_a = data(:,12);
ref_new_v = data(:,13);
ref_new_w = data(:,14);
input_v = data(:,15);
input_w = data(:,16);

n = length(data_x);
ts = 0.001;
t = 0:1:n-1;
t = t'*ts;
%% plot data
close all;
f1 = figure(1);  
set(f1, 'position', get(0, 'screensize'))

subplot(5,2,[1,3]);
plot(data_x,data_y,'LineWidth',3);
grid on;
hold on;
plot(ref_x,ref_y,'LineWidth',3);
axis equal
pbaspect([1 1 1]);
xlabel('$x$ [mm]','Interpreter','latex');
ylabel('$y$ [mm]','Interpreter','latex');
legend('data','ref','Interpreter','latex','Location','northwest')
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(5,2,2);
plot(t, data_x,'LineWidth',3);
grid on;
hold on;
plot(t, ref_x,'LineWidth',3);
% xlabel('Time [s]','Interpreter','latex');
ylabel('$x$ [mm]','Interpreter','latex');
xlim([0 max(t)])
legend('data','ref','Interpreter','latex','Location','northwest')
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(5,2,4);
plot(t, data_y,'LineWidth',3);
grid on;
hold on;
plot(t, ref_y,'LineWidth',3);
% xlabel('Time [s]','Interpreter','latex');
ylabel('$y$ [mm]','Interpreter','latex');
xlim([0 max(t)])
legend('data','ref','Interpreter','latex','Location','northwest')
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(5,2,5);
plot(t, data_a,'LineWidth',3);
grid on;
hold on;
plot(t, ref_a,'LineWidth',3);
% xlabel('Time [s]','Interpreter','latex');
ylabel('$a$ [mm$^2$/s]','Interpreter','latex');
xlim([0 max(t)])
legend('data', 'ref','Interpreter','latex','Location','northwest')
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(5,2,6);
plot(t, data_theta,'LineWidth',3);
grid on;
hold on;
plot(t, ref_theta,'LineWidth',3);
% xlabel('Time [s]','Interpreter','latex');
ylabel('$\theta$ [rad]','Interpreter','latex');
xlim([0 max(t)])
legend('data', 'ref', 'Interpreter','latex','Location','northwest')
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(5,2,7);
plot(t, data_v,'LineWidth',3);
grid on;
hold on;
plot(t, ref_v,'LineWidth',3);
plot(t, ref_new_v,'LineWidth',3);
% xlabel('Time [s]','Interpreter','latex');
ylabel('$v$ [mm/s]','Interpreter','latex');
xlim([0 max(t)])
legend('data','ref', 'new ref','Interpreter','latex','Location','northwest')
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(5,2,8);
plot(t, data_omega,'LineWidth',3);
grid on;
hold on;
plot(t, ref_omega,'LineWidth',3);
plot(t, ref_new_w,'LineWidth',3);
% xlabel('Time [s]','Interpreter','latex');
ylabel('$\omega$ [rad/s]','Interpreter','latex');
xlim([0 max(t)])
legend('data', 'ref', 'new ref','Interpreter','latex','Location','northwest')
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(5,2,9);
plot(t, input_v,'LineWidth',3);
grid on;
xlabel('Time [s]','Interpreter','latex');
ylabel('$u_v$ [V]','Interpreter','latex');
xlim([0 max(t)])
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(5,2,10);
plot(t, input_w,'LineWidth',3);
grid on;
xlabel('Time [s]','Interpreter','latex');
ylabel('$u_w$ [V]','Interpreter','latex');
xlim([0 max(t)])
set(gca, "FontName", "Times New Roman", "FontSize", 15);