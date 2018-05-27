%% Speed tests
% Scorbot encoder ratios
ENC2RAD = 2.0*pi/(3.0*160.0*96.0);
RAD2ENC = 1.0/(2.0*pi/(3.0*160.0*96.0));
Ts = 1e-3;
Thf = 1e-6;
Nr = 96;
% Max speed with backward model
max_speed_rads = max(diff(pos))/Ts
max_speed_rps = max_speed_rads/(2*pi)
max_speed_rpm = max_speed_rps*60
% Encoder time at max speed
Tenc = 1/(max_speed_rps*RAD2ENC)
% Number of sys tick per encoder tick at max speed
ratio = Tenc/Thf
error = Nr*max_speed_rads*Thf/(2*pi)

