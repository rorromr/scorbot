%% Motor params GM9434
L=2.51e-3;
R=2.96;
motor_num = 1;
motor_den = [L, R];
motor_sys = tf(motor_num,motor_den);
% Current loop running at 10kHz (sampling at 10Khz)
Fs = 10e3;
Ws = 2*pi*Fs;
Ts = 1/Fs;
% Max bandwidth should be 1/10 of Ws
max_bandwidth = 0.1*Ws;
disp(['Max bandwidth: ',num2str(max_bandwidth),'rad/s ,',num2str(max_bandwidth/(2*pi)), ' Hz'])
rltool(motor_sys)

fb = bandwidth(motor_sys);
fb_hz = fb/(2*pi);
%bode(motor_sys);
fn = damp(motor_sys)/(2*pi)
disp(['GM9434 bandwidth: ', num2str(fb_hz), ' Hz']);

%% Motor params GM9413
L=6.17e-3;
R=8.33;
motor_num = 1;
motor_den = [L, R];
motor_sys = tf(motor_num,motor_den);
fb = bandwidth(motor_sys);
fb_hz = fb/(2*pi);
%bode(motor_sys);
disp(['GM9413 bandwidth: ', num2str(fb_hz), ' Hz']);

%% Using sym

% Sym variables
kp = sym('kp');
ki = sym('ki');
s = sym('s');
La = sym('L');
Ra = sym('R');
% PI Controller
Cs = kp+ki*1/s; % PI
% Plant
Gs = 1/(La*s+Ra);
% Close loop TF
Hs = (sctrl*splant)/(1+sctrl*splant);
% Get num and den of close loop TF
[Hs_num, Hs_den] = numden(Hs)
Hs_num_coeff = fliplr(eval(feval(symengine,'coeff',snum,s,'All')))
Hs_den_coeff = fliplr(eval(feval(symengine,'coeff',sden,s,'All')))

% Motor params GM9434
vLa = 2.51e-3;
vRa = 2.96;
% From aalto course
Fs = 10e3;
Ws = 2*pi*Fs;
Ts = 1/Fs;
alpha = 0.1*Ws;
vkp = alpha*vL
vki = alpha*vR
vkd = 0
values = [vkp, vki, vkd, vL, vR];
svars = [kp, ki, kd, La, Ra];

num_values = double(subs(Hs_num_coeff,svars,values))
den_values = double(subs(Hs_den_coeff,svars,values))

motor_sys = tf(num_values,den_values);
wb = bandwidth(motor_sys);
fb  = wb/(2*pi);
wn = damp(motor_sys)
fn = wn/(2*pi);
bode(motor_sys);
disp(['GM9413 bandwidth: ', num2str(fb), ' Hz']);
