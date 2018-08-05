% Pittman-9434G697
km = 3.01; % oz*in/sqrt(w)
no_load_speed = 6151; % rpm
friction_torque = 0.60; % oz*in
v = 12; % Voltage supply
stall_current = 1.8; % Amps
% Torque conversion factor oz*in to n*m
ozin2nm=0.11298482933;
% Angular speed conversion factor rpm to rad/s
rpm2rads=0.104719755;
% Using SI units

kv = v/no_load_speed;
kt = kv;
peak_torque = stall_current*kt;

km = km*ozin2nm;
no_load_speed = no_load_speed*rpm2rads;
friction_torque = friction_torque*ozin2nm;
% Estimation of other params

peak_torque = stall_current*kt;
ra = v*v/(no_load_speed*peak_torque)
peak_torque = peak_torque*ozin2nm;
