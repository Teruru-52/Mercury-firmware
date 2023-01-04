clear;
close all

ts = 0.001;
%% import data
data = csvread("data_138.csv");

data_l = data(:,1);
data_v = data(:,2);
ref_l = data(:,3);
ref_v = data(:,4);

n = length(ref_v);
t = 0:1:n-1;
t = t*0.001;
t = t';
%% plot
f1 = figure(1);  
set(f1, 'position', get(0, 'screensize'))

subplot(2,1,1);
plot(t, ref_l,'LineWidth',3);
grid on;
hold on;
plot(t, data_l,'LineWidth',3);
xlabel('Time [s]','Interpreter','latex','FontSize',25);
ylabel('$l$ [m]','Interpreter','latex','FontSize',25);
xlim([0 n*ts])
legend('ref','data','Interpreter','latex','Location','northwest','FontSize',25)
h_axes = gca;
h_axes.XAxis.FontSize = 25;
h_axes.YAxis.FontSize = 25;

subplot(2,1,2);
plot(t, ref_v,'LineWidth',3);
grid on;
hold on;
plot(t, data_v,'LineWidth',3);
xlabel('Time [s]','Interpreter','latex','FontSize',25);
ylabel('$v$ [m/s]','Interpreter','latex','FontSize',25);
xlim([0 n*ts])
legend('ref','data','Interpreter','latex','Location','northwest','FontSize',25)
h_axes = gca;
h_axes.XAxis.FontSize = 25;
h_axes.YAxis.FontSize = 25;