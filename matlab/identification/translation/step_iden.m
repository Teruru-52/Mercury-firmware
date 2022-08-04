% ステップ応答によるシステム同定
%% ステップ応答
close all
clear

output = csvread('data.csv');

figure(1);
plot(output,'LineWidth',3);
grid on
xlabel('Time n','Interpreter','latex','FontSize',20);
ylabel('$v$ [m/s]','Interpreter','latex','FontSize',20);
h_axes = gca;
h_axes.XAxis.FontSize = 20;
h_axes.YAxis.FontSize = 20;
%% 
pidTuner()
%%
%             Kp                      
%   G(s) = ----------                  
%           1+Tp1*s        

% 2022/07/05                 
%         Kp = 0.8293                  
%        Tp1 = 0.17158 

% 2022/07/09
%         Kp = 0.79586                 
%         Tp1 = 0.18577 