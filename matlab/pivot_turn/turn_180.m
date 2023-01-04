% https://teruru-52.github.io/post/2022-03-22-rotation-curved-acceleration/
%% 目標値の生成
clf;
clear;
close all
t = 0;
t1 = 0.03;
t2 = 0.07;
t3 = 0.1;

% U_turn
theta_ref = pi;
t4 = 0.426;
t5 = t1 + t4;
t6 = t2 + t4;
t7 = t3 + t4;

jm = 3500;
am = jm * t1;

v3 = 2*(0.5*jm * t1^2) + am * (t2 - t1);
x1 = 1/6 * jm * t1^3;
v1 = 0.5 * jm * t1^2;
v2 = v1 + am * (t2 - t1);
x2 = x1 + v1 * (t2 - t1) + 0.5 * am * (t2 -t1)^2;
x3 = x2 + v3 * (t3 - t2) - 1/6 * jm * (t3 - t2)^3;
x4 = x3 + v3 * (t4 - t3);
x5 = x4 + (x3 - x2);
x6 = x5 + (x2 - x1);
n = 1;

while 1
    if t <= t1
        theta_tar(n) = 1/6 * jm * t^3;
        omega_tar(n) = 0.5 * jm * t^2;
        a_tar(n) = jm * t;

    elseif t <= t2
        theta_tar(n) = x1 + v1 * (t - t1) + 0.5 * am * (t - t1)^2;
        omega_tar(n) = v1 + am * (t - t1);
        a_tar(n) = am;

    elseif t <= t3
        theta_tar(n) = x3 + v3 * (t - t3) - 1/6 * jm * (t - t3)^3;
        omega_tar(n) = v3 - 0.5 * jm * (t - t3)^2;
        a_tar(n) = am - jm * (t - t2);

    elseif t <= t4
        theta_tar(n) = x3 + v3 * (t - t3);
        omega_tar(n) = v3;
        a_tar(n) = 0;

    elseif t <= t5
        theta_tar(n) = x4 + v3 * (t - t4) - 1/6 * jm * (t - t4)^3;
        omega_tar(n) = v3 - 0.5 * jm * (t - t4)^2;
        a_tar(n) = -jm * (t - t4);

    elseif t <= t6
        theta_tar(n) = x5 + v2 * (t - t5) - 0.5 * am * (t - t5)^2;
        omega_tar(n) = v2 - am * (t - t5);
        a_tar(n) = -am;

    elseif t <= t7
        theta_tar(n) = x6 + v1 * (t - t6) - 1/6 * jm * (t - t6)^3; 
        omega_tar(n) = 0.5 * jm * (t - t7)^2;
        a_tar(n) = - am + jm * (t - t6);
    else
        break;
    end
    n = n + 1;
    t = t + 0.001;
end

figure(1);
plot(theta_tar, 'LineWidth', 3);
grid on;
xlabel('Time [ms]','Interpreter','latex','FontSize',20);
ylabel('$\theta_{ref}$ [rad/s]','Interpreter','latex','FontSize',20);
h_axes = gca;
h_axes.XAxis.FontSize = 20;
h_axes.YAxis.FontSize = 20;

figure(2);
plot(omega_tar, 'LineWidth', 3);
grid on;
xlabel('Time [ms]','Interpreter','latex','FontSize',20);
ylabel('$\omega_{ref}$ [rad/s]','Interpreter','latex','FontSize',20);
h_axes = gca;
h_axes.XAxis.FontSize = 20;
h_axes.YAxis.FontSize = 20;

% figure(3);
% plot(a_tar, 'LineWidth', 2);
% grid on;
% xlabel('Time [ms]','Interpreter','latex','FontSize',20);
% ylabel('$a_{ref}$ [rad/s$^2$]','Interpreter','latex','FontSize',20);
% h_axes = gca;
% h_axes.XAxis.FontSize = 20;
% h_axes.YAxis.FontSize = 20;

theta_tar(526)
omega_tar(526)
% a_tar(526)

omega_tar = omega_tar';
%% plot
close all
data= csvread('data_180.csv');
theta = data(:,1);
omega = data(:,2);

figure(1);
plot(theta_tar, 'LineWidth', 3);
hold on;
grid on;
plot(theta, 'LineWidth', 3);
xlabel('Time [ms]','Interpreter','latex','FontSize',20);
ylabel('$\theta$ [rad]','Interpreter','latex','FontSize',20);
legend('$\theta_{ref}$','$\theta$','Interpreter','latex','Location','southeast','FontSize',20)
h_axes = gca;
h_axes.XAxis.FontSize = 20;
h_axes.YAxis.FontSize = 20;

figure(2);
plot(omega_tar, 'LineWidth', 3);
hold on;
grid on;
plot(omega, 'LineWidth', 3);
xlabel('Time [ms]','Interpreter','latex','FontSize',20);
ylabel('$\omega$ [rad/s]','Interpreter','latex','FontSize',20);
legend('$\omega_{ref}$','$\omega$','Interpreter','latex','Location','southeast','FontSize',20)
h_axes = gca;
h_axes.XAxis.FontSize = 20;
h_axes.YAxis.FontSize = 20;