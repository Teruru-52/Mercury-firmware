clear all
close all

%% 目標値を格納
% ref = csvread("ref_plot1.csv");
ref = csvread("ref_plot2.csv");
ref_x = ref(:,1);
ref_y = ref(:,2);
ref_theta = ref(:,3);
ref_omega = ref(:,4);

%% plot
f1 = figure(1);  
set(f1, 'position', get(0, 'screensize'))

subplot(3,2,[1,3,5]);
plot(ref_x,ref_y,'LineWidth',3);
grid on;
pbaspect([1 1 1]);
xlabel('$x$ [m]','Interpreter','latex','FontSize',25);
ylabel('$y$ [m]','Interpreter','latex','FontSize',25);
set(gca, "FontName", "Times New Roman", "FontSize", 25);

subplot(3,2,4);
plot(ref_theta,'LineWidth',3);
grid on;
xlabel('Time [ms]','Interpreter','latex','FontSize',25);
ylabel('$\theta$ [rad]','Interpreter','latex','FontSize',25);
xlim([0 length(ref_theta)])
set(gca, "FontName", "Times New Roman", "FontSize", 25);

subplot(3,2,6);
plot(ref_omega,'LineWidth',3);
grid on;
xlabel('Time [ms]','Interpreter','latex','FontSize',25);
ylabel('$\omega$ [rad/s]','Interpreter','latex','FontSize',25);
xlim([0 length(ref_theta)])
set(gca, "FontName", "Times New Roman", "FontSize", 25);