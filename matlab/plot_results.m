load('result_base_joint.mat')
% Set time at zero
time = time - time(1);
% Plot for position tracking
hold on;
plot(time,ref,'r');
plot(time,pos,'b');
legend('Referencia','Posición');
xlabel('Tiempo (s)');
ylabel('Posición angular de articulación (rad)');

%% Plot for force feeback
% Position tracking
ax1 = subplot(2,1,1);
plot(time,ref,'r',time,pos,'b');
legend('Referencia','Posición');
xlabel('Tiempo (s)');
ylabel('Posición angular de articulación (rad)');
% Force
ax2 = subplot(2,1,2); 
plot(time,force,'c');
legend('Fuerza de feedback');
xlabel('Tiempo (s)');
ylabel('Intensidad');
% Link both subplot time axis
linkaxes([ax1,ax2],'x')