clear;

%% import data
data = csvread("data_180.csv");

data_x = data(:,1)*0.001;
data_y = data(:,2)*0.001;
data_v = data(:,3)*0.001;
data_a = data(:,4)*0.001;
ref_x = data(:,5)*0.001;
ref_v = data(:,6)*0.001;
ref_a = data(:,7)*0.001;

n = length(ref_v);
ref_y = zeros(n, 1);

ts = 0.001;
t = 0:1:n-1;
t = t*0.001;
t = t';
%% plot
close all;
f1 = figure(1);  
set(f1, 'position', get(0, 'screensize'))

subplot(3,3,[1, 4, 7]);
plot(ref_y, ref_x,'LineWidth',3);
grid on;
hold on;
plot(data_y, data_x, '--','LineWidth',3);
xlabel('$y$ [m]','Interpreter','latex');
ylabel('$x$ [m]','Interpreter','latex');
xlim([-0.02 0.02])
legend('ref','data','Interpreter','latex','Location','northwest');
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(3,3,[2, 3]);
plot(t, ref_x,'LineWidth',3);
grid on;
hold on;
plot(t, data_x,'LineWidth',3);
xlabel('Time [s]','Interpreter','latex');
ylabel('$x$ [m]','Interpreter','latex');
xlim([0 n*ts])
legend('ref','data','Interpreter','latex','Location','northwest');
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(3,3,[5, 6]);
plot(t, ref_v,'LineWidth',3);
grid on;
hold on;
plot(t, data_v,'LineWidth',3);
xlabel('Time [s]','Interpreter','latex');
ylabel('$v$ [m/s]','Interpreter','latex');
xlim([0 n*ts])
legend('ref','data','Interpreter','latex','Location','northwest');
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(3,3,[8, 9]);
plot(t, ref_a,'LineWidth',3);
grid on;
hold on;
plot(t, data_a,'LineWidth',3);
xlabel('Time [s]','Interpreter','latex');
ylabel('$a$ [m/s$^2$]','Interpreter','latex');
xlim([0 n*ts])
legend('ref','data','Interpreter','latex','Location','northwest');
set(gca, "FontName", "Times New Roman", "FontSize", 15);
