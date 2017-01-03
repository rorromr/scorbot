%% Current probe calibration
% Load data
load('current_probe_calibration.mat');
figure;
plot(itest,vprobe,'b*-','LineWidth',2);
ylabel('Voltaje V');
xlabel('Corriente A');
k_probe = 1/(itest\vprobe);
disp(['Probe constant: ',num2str(k_probe),' A/V']);
clearvars -except k_probe
%% Stall current
load('stall_data.mat');
figure;
plot(vstall,istall,'b*-','LineWidth',2);
xlabel('Voltaje V');
ylabel('Corriente A');
% Armature resistance
ra = istall\vstall;
disp(['Armature resistance: ',num2str(ra),' Ohm']);
clearvars -except ra k_probe
%% Speed
load('p3_raw.mat');
Np = 96;% Encoder pulse per revolution
Ts = t(2)-t(1); % Sample time
Fs = 1/Ts;
speed = encoder_to_speed(encoder, Ts, Np); % Speed rad/s
% Speed low pass filter
Fc = 1000; % 1 KHz
[b,a] = butter(6,Fc/(Fs/2));
speed_filtered = filter(b,a,speed);
figure;
plot(t,speed_filtered*60/(2*pi),'LineWidth',2);
ylabel('Motor speed RPM');
xlabel('Time s');
%% Current
no_load_speed = mean(speed(50000:80000));
current_cal = current * k_probe;
max_current = max(current_cal);
no_load_current = mean(current_cal(50000:80000));
va_mean = mean(va(50000:80000));
plot(t,current_cal);
ylabel('Motor current A');
xlabel('Time s');
kt = (va_mean-ra*no_load_current)/no_load_speed;
km = kt/sqrt(ra);
oz2nm=0.00706155183333;
disp(['K_t = ',num2str(kt),' N.m/A | ',num2str(kt/oz2nm),' oz.in/A']);
disp(['K_m = ',num2str(km),' N.m/sqrt(W) | ',num2str(km/oz2nm),' oz.in/sqrt(W)']);
stall_current = va_mean/ra;
disp(['Stall current: ',num2str(stall_current),' A']);
stall_torque = stall_current*kt;
disp(['Stall torque: ',num2str(stall_torque),' N.m | ',num2str(stall_torque/oz2nm),' oz.in']);
disp(['Armature voltage: ',num2str(va_mean),' V']);
disp(['No load speed: ',num2str(no_load_speed),' rad/s | ', num2str(no_load_speed*60/(2*pi)), ' rpm']);

