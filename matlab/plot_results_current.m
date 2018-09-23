%% Plot for force feeback
% Position tracking
from=1;%7859;
to=length(time);%14000;
time_fix=time-time(from);
hold on;
refp = plot(time_fix(from:to),ref(from:to),'r', 'LineWidth',2);
currentp = plot(time_fix(from:to),current(from:to),'b');
currentp.Color(4) = 0.4;
legend('Referencia','Corriente');
xlabel('Tiempo (s)');
ylabel('Corriente (A)');
%% Step current info
load('current_step.mat');
plot(time_step,ref_step,'r',time_step,current_step,'b');
legend('Referencia','Posición');
xlabel('Tiempo (s)');
ylabel('Corriente (A)');
start_step = 187;
stepinfo(current_step(start_step:end),time_step(start_step:end),3.0,'SettlingTimeThreshold',0.05)
