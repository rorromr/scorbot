%% Motor parameters
% GM9434 all in SI units
gm9434.name = 'GM9434';
gm9434.kt = 0.0365; % N*m/A
gm9434.ke = 0.0365; % V/(rad/s)
gm9434.Ra = 2.96; % Ohm
gm9434.La = 0.00251; % H
gm9434.Jm = 4.17e-6; % Kg*m^2
gm9434.Bm = 2.6e-6; % N*m/(rad*s)
% GM9413 all in SI units
gm9413.name = 'GM9413';
gm9413.kt = 0.0395; % N*m/A
gm9413.ke = 0.0395; % V/(rad/s)
gm9413.Ra = 8.33; % Ohm
gm9413.La = 0.00617; % H
gm9413.Jm = 2.80e-6; % Kg*m^2
gm9413.Bm = 7.6e-7; % N*m/(rad*s)

%% Motor electrical dynamics
motor = gm9434;
motor_sys = tf([1],[motor.La, motor.Ra]);
% Current loop running at 10kHz (sampling at 10Khz)
Fs = 10e3;
Ws = 2*pi*Fs;
Ts = 1/Fs;
% Max bandwidth should be 1/10 of Ws
max_bandwidth = 0.1*Ws;
disp(['Max bandwidth: ',num2str(max_bandwidth),'rad/s ,',num2str(max_bandwidth/(2*pi)), ' Hz'])
%rltool(motor_sys)

fb = bandwidth(motor_sys);
fb_hz = fb/(2*pi);
fn = damp(motor_sys)/(2*pi)
disp([motor.name,' bandwidth: ', num2str(fb_hz), ' Hz']);

%% Current Controller
% Sym variables
kp = sym('kp');
ki = sym('ki');
s = sym('s');
La = sym('La');
Ra = sym('Ra');
% PI Controller
Cs = kp+ki*1/s; % PI
% Plant
Gs = 1/(La*s+Ra);
% Close loop TF
Hs = (Cs*Gs)/(1+Cs*Gs);
% Get num and den of close loop TF
[Hs_num, Hs_den] = numden(Hs)
Hs_num_coeff = fliplr(eval(feval(symengine,'coeff',Hs_num,s,'All')))
Hs_den_coeff = fliplr(eval(feval(symengine,'coeff',Hs_den,s,'All')))
% Value to get a 1st order closed loop system with alpha bandwidth
alpha = 0.1*Ws;
current_ctrl.kp = alpha*motor.La;
current_ctrl.ki = alpha*motor.Ra;
current_ctrl.Fs = 10000; 
current_ctrl.Ts = 1/Fs;
current_ctrl.z_num = [current_ctrl.kp+current_ctrl.Ts*current_ctrl.ki*(1.0/2.0), ...
-current_ctrl.kp+current_ctrl.Ts*current_ctrl.ki*(1.0/2.0)];
current_ctrl.z_den = [1.0 -1.0];
% Evaluate values
values = [current_ctrl.kp, current_ctrl.ki, motor.La, motor.Ra];
svars = [kp, ki, La, Ra];
% Num and den of closed loop tranfer function 
num_values = double(subs(Hs_num_coeff,svars,values))
den_values = double(subs(Hs_den_coeff,svars,values))
% Closed loop transfer function parameters
hs_sys = tf(num_values,den_values);
wb = bandwidth(motor_sys);
fb  = wb/(2*pi);
wn = damp(motor_sys)
fn = wn/(2*pi);
bode(hs_sys);
disp([motor.name,' controller bandwidth: ', num2str(fb), ' Hz']);
%% Mechanical part
encoder=(2*pi)/96;
vest.B = wb/200;
vest.kp = 2*vest.B;
vest.ki = vest.kp*vest.kp/4;

