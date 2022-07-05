%% M系列信号の生成
close all
clear

N = 127;

input = idinput(N+1,'prbs',[0,1],[-1.5,1.5]);
stairs(0:N,input,'LineWidth',2)
grid on
xlim([0 127]);
ylim([-4 4]);
xlabel('Time n','Interpreter','latex','FontSize',20);
ylabel('$u_\omega$ [V]','Interpreter','latex','FontSize',20);
h_axes = gca;
h_axes.XAxis.FontSize = 20;
h_axes.YAxis.FontSize = 20;