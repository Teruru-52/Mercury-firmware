clear;

%% import data
data = csvread("data_180.csv");

data_x = data(:,1);
data_y = data(:,2);
data_v = data(:,3);
data_a = data(:,4);
ref_x = data(:,5);
ref_v = data(:,6);
ref_a = data(:,7);
input_v = data(:,8);
input_w = data(:,9);

n = length(ref_v);
ref_y = zeros(n, 1);

ts = 0.001;
t = 0:1:n-1;
t = t*ts;
t = t';
%% plot
close all;
f1 = figure(1);  
set(f1, 'position', get(0, 'screensize'))

subplot(5,3,[1, 4, 7, 10, 13]);
plot(data_y, data_x,'LineWidth',3);
grid on;
hold on;
plot(ref_y, ref_x, '--', 'LineWidth',3);
xlabel('$y$ [mm]','Interpreter','latex');
ylabel('$x$ [mm]','Interpreter','latex');
xlim([-20 20])
% axis equal
% pbaspect([1 1 1]);
legend('data','ref','Interpreter','latex','Location','northwest');
set(gca, "FontName", "Times New Roman", "FontSize", 20);

subplot(5,3,[2, 3]);
plot(t, data_x,'LineWidth',3);
grid on;
hold on;
plot(t, ref_x,'LineWidth',3);
% xlabel('Time [s]','Interpreter','latex');
ylabel('$x$ [mm]','Interpreter','latex');
xlim([0 n*ts])
legend('data','ref','Interpreter','latex','Location','best');
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(5,3,[5, 6]);
plot(t, data_v,'LineWidth',3);
grid on;
hold on;
plot(t, ref_v,'LineWidth',3);
% xlabel('Time [s]','Interpreter','latex');
ylabel('$v$ [mm/s]','Interpreter','latex');
xlim([0 n*ts])
legend('data','ref','Interpreter','latex','Location','best');
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(5,3,[8, 9]);
plot(t, data_a,'LineWidth',3);
grid on;
hold on;
plot(t, ref_a,'LineWidth',3);
% xlabel('Time [s]','Interpreter','latex');
ylabel('$a$ [mm/s$^2$]','Interpreter','latex');
xlim([0 n*ts])
legend('data','ref','Interpreter','latex','Location','best');
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(5,3,[11, 12]);
plot(t, input_v,'LineWidth',3);
grid on;
% xlabel('Time [s]','Interpreter','latex');
ylabel('$u_v$ [V]','Interpreter','latex');
xlim([0 n*ts])
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(5,3,[14, 15]);
plot(t, input_w,'LineWidth',3);
grid on;
xlabel('Time [s]','Interpreter','latex');
ylabel('$u_w$ [V]','Interpreter','latex');
xlim([0 n*ts])
set(gca, "FontName", "Times New Roman", "FontSize", 15);
