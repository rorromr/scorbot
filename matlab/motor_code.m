%% Motor parameters
% GM9434 all in SI units
gm9434.name = 'GM9434';
gm9434.kt = 0.0365; % N*m/A
gm9434.ke = 0.0365; % V/(rad/s)
gm9434.Ra = 2.96; % Ohm
gm9434.La = 0.00251; % H
gm9434.Jm = 4.17e-6; % Kg*m^2
gm9434.Bm = 2.6e-6; % N*m/(rad*s)
gm9434.max_speed = 600; % rad/s
% GM9413 all in SI units
gm9413.name = 'GM9413';
gm9413.kt = 0.0395; % N*m/A
gm9413.ke = 0.0395; % V/(rad/s)
gm9413.Ra = 8.33; % Ohm
gm9413.La = 0.00617; % H
gm9413.Jm = 2.80e-6; % Kg*m^2
gm9413.Bm = 7.6e-7; % N*m/(rad*s)
gm9413.max_speed = 600; % rad/s

%% Motor electrical dynamics
motor = gm9434;
motor_sys = tf(1,[motor.La, motor.Ra]);
% Current loop running at 10kHz (sampling at 10Khz)
Fs = 10e3;
Ws = 2*pi*Fs;
Ts = 1/Fs;
% Max bandwidth of controller should be 1/10 of Ws
max_bandwidth = 0.1*Ws;
disp(['Max bandwidth: ',num2str(max_bandwidth),'rad/s ,',num2str(max_bandwidth/(2*pi)), ' Hz'])
fb = bandwidth(motor_sys);
fb_hz = fb/(2*pi);
fn = damp(motor_sys)/(2*pi);
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
current_ctrl.Fs = 10e3; % Current sampling at 10kHz
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
wb = bandwidth(hs_sys);
fb  = wb/(2*pi);
bode(hs_sys);
disp([motor.name,' controller bandwidth: ', num2str(fb), ' Hz']);
%% Mechanical part
% Speed observer
encoder=(2*pi)/96;
vest.B = wb/100;
disp(['Speed observer bandwidth: ', num2str(vest.B/(2*pi)), ' Hz']);
vest.kp = 2*vest.B
vest.ki = vest.kp*vest.kp/4
% Mechanical system
joint.i = (160*3)/1; % Gear ratio
joint.J_load = 15*(0.5^2); % equivalent joint inertia 15kg*(0.5m)^2
joint.J = motor.Jm + joint.J_load/(joint.i^2) % Equivalent inertia motor side
%% Speed controller
mechanical_sys = tf([1],[joint.J, motor.Bm])
% Speed controller
speed_ctrl.B = vest.B/10;
disp(['Speed controller bandwidth: ', num2str(speed_ctrl.B/(2*pi)), ' Hz']);
%rltool(mechanical_sys) % Controller for natural frequency 15 rad/s, 0.707 damping ratio
speed_ctrl.kp = 0.00055;
speed_ctrl.ki = speed_ctrl.kp*9.0;
% Sym variables
kp = sym('kp');
ki = sym('ki');
s = sym('s');
J = sym('J');
B = sym('B');
% PI Controller
Cs = kp+ki*1/s; % PI
% Plant
Gs = 1/(J*s+B);
% Close loop TF
Hs = (Cs*Gs)/(1+Cs*Gs)
% Get num and den of close loop TF
[Hs_num, Hs_den] = numden(Hs)
Hs_num_coeff = fliplr(eval(feval(symengine,'coeff',Hs_num,s,'All')))
Hs_den_coeff = fliplr(eval(feval(symengine,'coeff',Hs_den,s,'All')))
% Evaluate values
values = [speed_ctrl.kp, speed_ctrl.ki, joint.J, motor.Bm];
svars = [kp, ki, J, B];
% Num and den of closed loop tranfer function 
num_values = double(subs(Hs_num_coeff,svars,values))
den_values = double(subs(Hs_den_coeff,svars,values))
% Closed loop transfer function parameters
hs_sys = tf(num_values,den_values);
wb = bandwidth(hs_sys)
fb  = wb/(2*pi)
bode(hs_sys)
disp([motor.name,' speed controller bandwidth: ', num2str(fb), ' Hz']);
%% Position controller
tf_speed_ctrl = tf(num_values,den_values)
tf_speed_plant = tf(num_values, conv(den_values,[1 0])) % Add 1/s (speed to position plant)
bode(tf_speed_plant)
%rltool(tf_speed_plant) % Controller for natural frequency 3.5 rad/s, 1 damping ratio
%%
position_ctrl.kp = 1.3;
tf_position_ctrl = tf([position_ctrl.kp],[1])
tf_closed_loop = feedback(tf_speed_plant, tf_position_ctrl)
bode(tf_closed_loop)
%%
wb = bandwidth(tf_closed_loop)
fb  = wb/(2*pi)
disp([motor.name,' position controller bandwidth: ', num2str(fb), ' Hz']);
