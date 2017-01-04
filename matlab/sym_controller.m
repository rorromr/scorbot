kp = sym('kp');
ki = sym('ki');
kd = sym('kd');
tau = sym('tau');
s = sym('s');
alpha = sym('alpha');
Ts = sym('Ts');
z = sym('z');

scontroller = kp + ki*1/s + kd*s/(s*tau+1);
zcontroller = subs(scontroller, s, (2/Ts)*(z-1)/(z+1));

[znum, zden] = numden(zcontroller);
znum_coeff = fliplr(eval(feval(symengine,'coeff',znum,z,'All')));
zden_coeff = fliplr(eval(feval(symengine,'coeff',zden,z,'All')));

ccode(znum_coeff)
ccode(zden_coeff)

%% Eval
vkp = 1;
vki = 0.5;
vkd = 0.1;
vtau = 0.1;
vTs = 0.01;
valpha = 2/vTs;
values = [vkp, vki, vkd, vtau, vTs, valpha];
svars = [kp, ki, kd, tau, Ts, alpha];

znum_values = double(subs(znum_coeff,svars,values));
zden_values = double(subs(zden_coeff,svars,values));
zcontroller_tf1 = tf(znum_values, zden_values, vTs);

%% Verify
[snum, sden] = numden(scontroller);
snum_coeff =  fliplr(eval(feval(symengine,'coeff',snum,s,'All')));
sden_coeff =  fliplr(eval(feval(symengine,'coeff',sden,s,'All')));
snum_values = double(subs(snum_coeff,svars,values));
sden_values = double(subs(sden_coeff,svars,values));
scontroller_tf = tf(snum_values, sden_values);
zcontroller_tf2 = c2d(scontroller_tf,vTs,'tustin');

bode(zcontroller_tf2, zcontroller_tf1)
legend('Direct','Manual')


