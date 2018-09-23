kp = sym('kp');
ki = sym('ki');
kd = sym('kd');
tau = sym('tau');
s = sym('s');
alpha = sym('alpha');
Ts = sym('Ts');
w = sym('w'); % w = z^-1
z = sym('z');

scontroller = kp + ki*1/s + kd*s/(s*tau+1); % PID
%scontroller = kp + kd*s/(s*tau+1); % PD
%scontroller = kp+ki*1/s; % I
zcontroller = subs(scontroller, s, (2/Ts)*(1-w)/(1+w));
% Tustin (2/Ts)*(1-w)/(1+w)
% Euler backward (1-w)/Ts

[znum, zden] = numden(zcontroller);
znum_coeff = fliplr(eval(feval(symengine,'coeff',znum,w,'All')));
zden_coeff = fliplr(eval(feval(symengine,'coeff',zden,w,'All')));
% Coef normalization
znum_coeff = znum_coeff./zden_coeff(end)
zden_coeff = zden_coeff./zden_coeff(end)
% [ak ... a1 a0] => a_k * z^-k + ... + a_1 * z^-1 + a0

% C code in order
b=ccode(fliplr(sym(znum_coeff)))
a=ccode(fliplr(sym(zden_coeff)))

%% Eval
vkp = 1000;
vki = 0.5;
vkd = 1.8;
vtau = 0.1;
vTs = 0.01;
valpha = 2/vTs;
values = [vkp, vki, vkd, vtau, vTs, valpha];
svars = [kp, ki, kd, tau, Ts, alpha];

znum_values = double(subs(znum_coeff,svars,values));
zden_values = double(subs(zden_coeff,svars,values));
% Flip coeffs for z^-1
zcontroller_tf1 = tf(fliplr(znum_values), fliplr(zden_values), vTs, 'Variable', 'z^-1');

%% Verify
[snum, sden] = numden(scontroller);
snum_coeff =  fliplr(eval(feval(symengine,'coeff',snum,s,'All')));
sden_coeff =  fliplr(eval(feval(symengine,'coeff',sden,s,'All')));
snum_values = double(subs(snum_coeff,svars,values));
sden_values = double(subs(sden_coeff,svars,values));
scontroller_tf = tf(snum_values, sden_values);
zcontroller_tf2 = c2d(scontroller_tf,vTs,'tustin');

figure;
bode(zcontroller_tf1, zcontroller_tf2);
legend('Direct','C2D');


