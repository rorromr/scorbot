%% Speed tests
% Scorbot encoder ratios
ENC2RAD = 2.0*pi/(3.0*160.0*96.0);
RAD2ENC = 1.0/(2.0*pi/(3.0*160.0*96.0));
Ts = 1e-3;
Thf = 1e-6;
Nr = 96;
% Speed arrays
speed_rad = diff(pos)/Ts;
speed_rps = speed_rad/(2*pi);
speed_rpm = speed_rps*60;
plot(time(2:end),speed_rpm,'r');
legend('Velocidad angular');
xlabel('Tiempo (s)');
ylabel('Velocidad (rpm)');
%% Max calculation

% Max speed with backward model
max_speed_rads = max(diff(pos))/Ts
max_speed_rps = max_speed_rads/(2*pi)
max_speed_rpm = max_speed_rps*60
% Encoder time at max speed
Tenc = 1/(max_speed_rps*RAD2ENC)
% Number of sys tick per encoder tick at max speed
ratio = Tenc/Thf
error = Nr*max_speed_rads*Thf/(2*pi)

%% Loop control data
% speed measurement at each control loop sample
w_rad = 2*pi/(Nr*Ts);
w_rps = w_rad/(2*pi);
w_rpm = w_rps*60
w_joint = w_rpm/(3.0*160.0)