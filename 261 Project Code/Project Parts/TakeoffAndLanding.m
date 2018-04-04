clear,clc
% Assumes wing is mounted at 0 degrees
% All data is just saved to the workspace.

g = 32.17; % ft / s^2
rho = 0.0023769; % slugs / ft^3
W = 1400; % lbs
S = 45; % ft^2
k = 0.0909;
c_D0 = 0.1210;
c_L_max = 1.5052;
c_L_g = 0.1542;
n_t = 1.15; % Load Factor on Takeoff
n_l = 1.2; % Load Factor on Landing
mu_R = 0.02;
mu_B = 0.4;
h_t = 35; % ft Takeoff
h_l = 50; % ft Landing

v_stall = sqrt((2*W)/(rho*S*c_L_max)); % ft/s
v_LO = 1.1 * v_stall; % ft/s
v_AP = 1.3 * v_stall; % ft/s
v_FL = 1.23 * v_stall; % ft/s
v_TD = 1.15 * v_stall; % ft/s

q = 0.5 * rho * (v_LO*0.7)^2; % dynamic pressure @ 0.7 * v_LO
T = S*c_D0*q+(k*W^2)/(q*S); % lbs  @ 0.7 * v_LO
D = q * S * (c_D0 + k*c_L_g^2); % Lift @ 0.7 * v_LO @ alpha = 0
L = q * S * c_L_g; % Lift @ 0.7 * v_LO @ alpha = 0

% Takeoff
angle_t = 5; % degrees
s_g = v_LO^2 / (2*g*((T-D)/W - mu_R*(1-L/W))); % ft
R_tr = v_LO^2 / ((n_t-1)*g); % ft
s_tr = R_tr * sind(angle_t); % ft
h_tr = R_tr - R_tr * cosd(angle_t); % ft
s_cl = (h_t - h_tr) / tand(angle_t); % ft
s_TO = s_g + s_tr + s_cl; % ft

q = 0.5 * rho * (v_TD*0.7)^2; % dynamic pressure @ 0.7 * v_TD
D = q * S * (c_D0 + k*c_L_g^2); % Lift @ 0.7 * v_TD @ alpha = 0
L = q * S * c_L_g; % Lift @ 0.7 * v_TD @ alpha = 0

% Landing
angle_l = 3; % degrees
R_f = v_FL^2 / ((n_l-1)*g); % ft
s_f = R_f * sind(angle_l); % ft
h_f = R_f - R_f * cosd(angle_l); % ft
s_a = (h_l - h_f)/tand(angle_l); % ft
s_g = v_TD^2 / (2*g*(D/W + mu_B*(1-L/W))); % ft
s_LA = s_g + s_f + s_a; % ft




