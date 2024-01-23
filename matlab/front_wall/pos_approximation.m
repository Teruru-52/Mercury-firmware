clear;
close all;

%% load log
data = csvread("data.csv");
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
%% plot data
close all;
f1 = figure(1);  
set(f1, 'position', get(0, 'screensize'))

subplot(3,2,[1,3]);
plot(ir_s, pos_x,'LineWidth',3);
grid on;
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
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(3,2,5);
plot(t, ir_sl,'LineWidth',3);
grid on;
xlabel('Time [s]','Interpreter','latex');
ylabel('IR sensor value (left)','Interpreter','latex');
xlim([0 max(t)])
set(gca, "FontName", "Times New Roman", "FontSize", 15);

subplot(3,2,6);
plot(t, ir_sr,'LineWidth',3);
grid on;
xlabel('Time [s]','Interpreter','latex');
ylabel('IR sensor value (right)','Interpreter','latex');
xlim([0 max(t)])
set(gca, "FontName", "Times New Roman", "FontSize", 15);