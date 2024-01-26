%% M系列信号の生成
close all
clear

N = 127;

input = idinput(N+1,'prbs',[0,1],[-1.5,1.5]);
figure(1);
stairs(0:N,input,'LineWidth',2)
grid on
xlim([0 127]);
ylim([-4 4]);
xlabel('Time n','Interpreter','latex','FontSize',20);
ylabel('$u_\omega$ [V]','Interpreter','latex','FontSize',20);
h_axes = gca;
h_axes.XAxis.FontSize = 20;
h_axes.YAxis.FontSize = 20;

%% 検証用入力の結合
input = input(1:127,1);
% 半周期だけずらして結合する
input_formar = input(1:63,1);
input_latter = input(64:127,1);
validation_input = cat(1, input_latter, input_formar);

% 実験・検証用入力
input2 = cat(1,input,validation_input);
figure(2);
stairs(0:2*N-1,input2,'LineWidth',3)
grid on
xlim([0 2*N-1])
ylim([-1.7 1.7])
xlabel('Time n','Interpreter','latex','FontSize',25);
ylabel('$u_\omega$ [V]','Interpreter','latex','FontSize',25);
set(gca, "FontName", "Times New Roman", "FontSize", 25);

%% save as .csv file
dlmwrite('./ref_u_v.csv', input2', 'delimiter', ',', 'precision', '%.1f');