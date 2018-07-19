%% Motor params GM9434
L=2.51e-3;
R=2.96;
motor_num = 1;
motor_den = [L, R];xs
motor_sys = tf(motor_num,motor_den);
fb = bandwidth(motor_sys);
fb_hz = fb/(2*pi);
%bode(motor_sys);
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
kp = sym('kp');
ki = sym('ki');
kd = sym('kd');
tau = sym('tau');
s = sym('s');
L = sym('L');
R = sym('R');

%scontroller = (s+s^2*kp/ki)/(1+s*kp/ki+s^2*1/ki); % PID
%scontroller = kp + kd*s/(s*tau+1); % PD
sctrl = kp+ki*1/s; % PI
splant = 1/(L*s+R);
close_loop = (sctrl*splant)/(1+sctrl*splant);

[snum, sden] = numden(close_loop)
snum_coeff = fliplr(eval(feval(symengine,'coeff',snum,s,'All')))
sden_coeff = fliplr(eval(feval(symengine,'coeff',sden,s,'All')))

%% Ctrl eval
vkp = 7.5;
vki = 8;
vkd = 0;
vL = 2.51e-3;
vR = 2.96;
values = [vkp, vki, vkd, vL, vR];
svars = [kp, ki, kd, L, R];

num_values = double(subs(snum_coeff,svars,values))
den_values = double(subs(sden_coeff,svars,values))

motor_sys = tf(num_values,den_values);
fb = bandwidth(motor_sys);
fb_hz = fb/(2*pi);
bode(motor_sys);
disp(['GM9413 bandwidth: ', num2str(fb_hz), ' Hz']);
