clear;
close all

ts = 0.001;
%% import data
data = csvread("data1.csv");
ref_v = csvread("ref1.csv");
% data = csvread("data2.csv");
% ref_v = csvread("ref2.csv");
% data = csvread("data3.csv");
% ref_v = csvread("ref3.csv");
data_l = data(:,1);
data_v = data(:,2);

n = length(data_v);
t = 0:1:n-1;
t = t*0.01;
t = t';

%% calculate ref_l
ref_l(1) = 0;
for i = 2:n
    ref_l(i) = ref_l(i-1) + ref_v(i) * ts;
end
%% plot
figure(1);
plot(t, data_v,'LineWidth',3);
grid on;
hold on;
plot(t, ref_v,'LineWidth',3);
xlabel('Time [s]','Interpreter','latex','FontSize',25);
ylabel('$v$ [m/s]','Interpreter','latex','FontSize',25);
xlim([0 9.01])
legend('data','ref','Interpreter','latex','Location','northwest','FontSize',25)
h_axes = gca;
h_axes.XAxis.FontSize = 25;
h_axes.YAxis.FontSize = 25;

figure(2);
plot(t, data_l,'LineWidth',3);
grid on;
hold on;
plot(t, ref_l,'LineWidth',3);
xlabel('Time [s]','Interpreter','latex','FontSize',25);
ylabel('$l$ [m]','Interpreter','latex','FontSize',25);
xlim([0 9.01])
legend('data','ref','Interpreter','latex','Location','northwest','FontSize',25)
h_axes = gca;
h_axes.XAxis.FontSize = 25;
h_axes.YAxis.FontSize = 25;