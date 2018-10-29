Fs=60; % Sample freq Hz
Ts=1/Fs; % Sample time s
B=1*2*pi; % Bandwidth 1Hz
kp=2*B;
ki=kp^2/4;
% Speed observer TF
num_so = [kp/ki 1 0];
den_so = [1/ki kp/ki 1];
tf_so = tf(num_so,den_so);
tf_d = c2d(tf_so,Ts,'tustin');
bode(tf_d);
%%
% Evaluation
% Position data at 100 Hz, spline interpolation
pos=pos-pos(1); % Normalize
time_60hz = time(1):1/60:time(end); 
pos_60hz = spline(time,pos,time_60hz);
[speed_est, speed_est_t] = lsim(tf_d,pos_60hz,time_60hz);
figure;
hold on;
RAD2RPM=30/(pi);
ratio = 3.0*160.0*RAD2RPM;
plot(time_60hz(2:end),diff(pos_60hz)/(time_60hz(2)-time_60hz(1))*ratio,'r');
xlabel('Tiempo (s)');
ylabel('Velocidad del motor (RPM)');
plot(speed_est_t, speed_est*ratio, 'b');
legend('Diferenciación', 'Estimación')
%% Max speed
RAD2RPM=30/(pi);
joint_speed = (0.558+0.101)/(7.68-6.17) % rad/s at joint
joint_speed_rpm = joint_speed*RAD2RPM % rpm at joint
motor_rpm = joint_speed_rpm*3.0*160.0 % rpm at motor, timming belt and harmonic drive reduction


