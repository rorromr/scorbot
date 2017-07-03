load('scorbot_test.mat')
plot(time, base)
hold on;
plot(time_cmd, cmd, 'r')
legend('Posición', 'Comando')
xlabel('Tiempo [ms]');
ylabel('Posición [rad]');


