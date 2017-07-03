load('scorbot_test.mat')



subplot(2,1,1);
plot(time, base)
hold on;
plot(time_cmd, cmd, 'r')

legend('Posición', 'Comando')
xlabel('Tiempo [ms]');
ylabel('Posición [rad]');

subplot(2,1,2); 
set(gca,'xtick',[],'ytick',[])
plot(time_force, force_x, 'g')
legend('Fuerza feedback')
xlabel('Tiempo [ms]');
ylabel('Fuerza normalizada');