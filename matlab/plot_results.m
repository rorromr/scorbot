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

%% Plot current
load('current_10khz.mat')
time = time - time(1);
current = 0.00625*current;
ref = 0.00625*ref;
hold on;
plot(time,ref,'r');
plot(time,current,'b');
legend('Referencia','Posición');
xlabel('Tiempo (s)');
ylabel('Corriente (A)');
%% Noise analysis
load('current_10khz.mat');
time_10khz = time(6200:end)-time(6200);
current_10khz = 0.00625*current(6200:end);

load('current_1khz.mat');
time_1khz = time(1:774);
current_1khz = 0.00625*current(1:774);

hold on;
plot(time_1khz,current_1khz,'b');
plot(time_1khz,current_10khz,'r');
xlabel('Tiempo (s)');
ylabel('Corriente (A)');

% some noise indicators
disp(['mean 10khz: ', num2str(mean(current_10khz))]);
disp(['std 10khz: ', num2str(std(current_10khz))]);
disp(['mean 1khz: ', num2str(mean(current_1khz))]);
disp(['std 1khz: ', num2str(std(current_1khz))]);






