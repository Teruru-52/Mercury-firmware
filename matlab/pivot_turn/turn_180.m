% https://teruru-52.github.io/post/2022-03-22-rotation-curved-acceleration/
%% trajectory generation
clf;
clear;
close all

log_plot = 1;

%% parameter setting
dt = 0.001;
t = 0;

t1 = 0.04;
t2 = 0.1;
t3 = t1+ t2;
jm = 3500;

am = jm * t1;
v3 = 2*(0.5*jm * t1^2) + am * (t2 - t1);
x1 = 1/6 * jm * t1^3;
v1 = 0.5 * jm * t1^2;
v2 = v1 + am * (t2 - t1);
x2 = x1 + v1 * (t2 - t1) + 0.5 * am * (t2 -t1)^2;
x3 = x2 + v3 * (t3 - t2) - 1/6 * jm * (t3 - t2)^3;

theta_ref = pi; % U turn
t4 = (theta_ref - 2*(v1*(t2 - t1) + 0.5*am*(t2 - t1)^2 + v3*(t3 - t2))) / v3 + t3;
% t4 = round(t4 / dt) * dt;
t4 = floor(t4 / dt) * dt;

t5 = t1 + t4;
t6 = t2 + t4;
t7 = t3 + t4;

x4 = x3 + v3 * (t4 - t3);
x5 = x4 + (x3 - x2);
x6 = x5 + (x2 - x1);
n = 1;

while 1
    if t <= t1
        theta_tar(n) = 1/6 * jm * t^3;
        omega_tar(n) = 0.5 * jm * t^2;
        a_tar(n) = jm * t;
        j_tar(n) = jm;

    elseif t <= t2
        theta_tar(n) = x1 + v1 * (t - t1) + 0.5 * am * (t - t1)^2;
        omega_tar(n) = v1 + am * (t - t1);
        a_tar(n) = am;
        j_tar(n) = 0;

    elseif t <= t3
        theta_tar(n) = x3 + v3 * (t - t3) - 1/6 * jm * (t - t3)^3;
        omega_tar(n) = v3 - 0.5 * jm * (t - t3)^2;
        a_tar(n) = am - jm * (t - t2);
        j_tar(n) = -jm;

    elseif t <= t4
        theta_tar(n) = x3 + v3 * (t - t3);
        omega_tar(n) = v3;
        a_tar(n) = 0;
        j_tar(n) = 0;

    elseif t <= t5
        theta_tar(n) = x4 + v3 * (t - t4) - 1/6 * jm * (t - t4)^3;
        omega_tar(n) = v3 - 0.5 * jm * (t - t4)^2;
        a_tar(n) = -jm * (t - t4);
        j_tar(n) = -jm;

    elseif t <= t6
        theta_tar(n) = x5 + v2 * (t - t5) - 0.5 * am * (t - t5)^2;
        omega_tar(n) = v2 - am * (t - t5);
        a_tar(n) = -am;
        j_tar(n) = 0;

    elseif t <= t7
        theta_tar(n) = x6 + v1 * (t - t6) - 1/6 * jm * (t - t6)^3;
        omega_tar(n) = 0.5 * jm * (t - t7)^2;
        a_tar(n) = - am + jm * (t - t6);
        j_tar(n) = jm;
    else
        break;
    end
    n = n + 1;
    t = t + dt;
end

theta_tar(end)
omega_tar(end)
a_tar(end)

%% save as .csv file
dlmwrite('./ref_v_180.csv', omega_tar, 'delimiter', ',', 'precision', '%.5f');
dlmwrite('./ref_a_180.csv', a_tar, 'delimiter', ',', 'precision', '%.5f');

%% plot reference
T = (0:length(theta_tar)-1)'*dt;
if ~log_plot
    f1 = figure(1);
    set(f1, 'position', get(0, 'screensize'))

    figure(1);
    subplot(4, 1, 1);
    plot(T, theta_tar, 'LineWidth', 3);
    grid on;
    xlabel('Time [ms]','Interpreter','latex');
    ylabel('$\theta_{ref}$ [rad/s]','Interpreter','latex');
    xlim([0, T(end)]);
    set(gca, "Fontname", "Times New Roman", "Fontsize", 20);

    subplot(4, 1, 2);
    plot(T, omega_tar, 'LineWidth', 3);
    grid on;
    xlabel('Time [ms]','Interpreter','latex');
    ylabel('$\omega_{ref}$ [rad/s]','Interpreter','latex');
    xlim([0, T(end)]);
    set(gca, "FontName", "Times New Roman", "FontSize", 20);

    subplot(4, 1, 3);
    plot(T, a_tar, 'LineWidth', 3);
    grid on;
    xlabel('Time [ms]','Interpreter','latex');
    ylabel('$a_{ref}$ [rad/s$^2$]','Interpreter','latex');
    xlim([0, T(end)]);
    set(gca, "FontName", "Times New Roman", "FontSize", 20);

    subplot(4, 1, 4);
    plot(T, j_tar, 'LineWidth', 3);
    grid on;
    xlabel('Time [ms]','Interpreter','latex');
    ylabel('$j_{ref}$ [rad/s$^3$]','Interpreter','latex');
    xlim([0, T(end)]);
    set(gca, "FontName", "Times New Roman", "FontSize", 20);
end

%% plot
if log_plot
    data= csvread('data_180.csv');
%         data= csvread('data_180_fbff.csv');

    theta = data(:,1);
    omega = data(:,2);

    figure(2);
    subplot(2, 1, 1);
    plot(T, theta_tar, 'LineWidth', 3);
    hold on;
    grid on;
    plot(T, theta, 'LineWidth', 3);
    xlabel('Time [ms]','Interpreter','latex');
    ylabel('$\theta$ [rad/s]','Interpreter','latex');
    xlim([0, T(end)]);
    legend('$\theta_{ref}$','$\theta$','Interpreter','latex','Location','southeast');
    set(gca, "FontName", "Times New Roman", "FontSize", 15);

    subplot(2, 1, 2);
    plot(T, omega_tar, 'LineWidth', 3);
    hold on;
    grid on;
    plot(T, omega, 'LineWidth', 3);
    xlabel('Time [ms]','Interpreter','latex');
    ylabel('$\omega$ [rad/s]','Interpreter','latex');
    xlim([0, T(end)]);
    legend('$\omega_{ref}$','$\omega$','Interpreter','latex','Location','northeast')
    set(gca, "Fontname", "Times New Roman", "Fontsize", 15);
end