clear;
close all;

%% load log
% data = csvread("data_long_x.csv");
data = csvread("data_short_x.csv");
n = length(data);

ir_value = data(1:2:n, :); % odd rows
pos = data(2:2:n, :); % even rows

ir_sl = ir_value(:,3);
ir_sr = ir_value(:,4);
ir_s = (ir_sl + ir_sr) / 2;

pos_x = pos(:,1);
% pos_y = pos(:,2);

n = length(pos_x);
ts = 0.2;
t = 0:1:n-1;
t = t'*ts;

%% curve fitting (logarithmic model)
%     log_fittype = fittype("a*log(b*x+c)+d", dependent="y", independent="x", coefficients=["a" "b" "c" "d"]);
    log_fittype = fittype("a*log(max(1e-15, b * x + c))+d", dependent="y", independent="x", coefficients=["a" "b" "c" "d"]);

    a = 20; b = 0.005;  c = 1 - 2287*b; d = 0.01;
    % b*ir_s + c % must be positive
    initialCoefficients = [a, b, c, d];
    for i = 1:10
        if i > 1
            initialCoefficients = [f.a, f.b, f.c, f.d];
        end
        f = fit(ir_s, pos_x, log_fittype, 'StartPoint', initialCoefficients)
    end
    pos_x_fit = f.a.*log(f.b*ir_s + f.c) + f.d;
    % pos_x_fit = a.*log(b*ir_s + c) + d;
%% plot data
close all;
f1 = figure(1);
set(f1, 'position', get(0, 'screensize'))
ir_max = 2800;
ir_min = 2000;

subplot(3,2,[1,3]);
plot(ir_s, pos_x, 'o', 'MarkerSize',6);
grid on;
hold on;
plot(ir_s, pos_x_fit,'LineWidth',3);
xlabel('IR sensor value (mean)','Interpreter','latex');
ylabel('$x$ [mm]','Interpreter','latex');
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(3,2,2);
plot(t, pos_x,'LineWidth',3);
grid on;
xlabel('Time [s]','Interpreter','latex');
ylabel('$x$ [mm]','Interpreter','latex');
xlim([0 max(t)])
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(3,2,4);
plot(t, ir_s,'LineWidth',3);
grid on;
xlabel('Time [s]','Interpreter','latex');
ylabel('IR sensor value (mean)','Interpreter','latex');
xlim([0 max(t)])
ylim([ir_min ir_max]);
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(3,2,5);
plot(t, ir_sl,'LineWidth',3);
grid on;
xlabel('Time [s]','Interpreter','latex');
ylabel('IR sensor value (left)','Interpreter','latex');
xlim([0 max(t)])
ylim([ir_min ir_max]);
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(3,2,6);
plot(t, ir_sr,'LineWidth',3);
grid on;
xlabel('Time [s]','Interpreter','latex');
ylabel('IR sensor value (right)','Interpreter','latex');
xlim([0 max(t)])
ylim([ir_min ir_max]);
set(gca, "FontName", "Times New Roman", "FontSize", 15);