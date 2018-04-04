clear,clc

%THIS FILE HAS NO OUTPUTS. ALL DATA IS IN THE WORKSPACE
%Data include the radius and omega of the maneuvers

n_structure = 3; % max structural load factor
v = 177; %ft/s (Loiter Speed)
rho = 0.0023769; % slugs / ft^3
q = 1/2 * rho * v^2; %slugs / ft^2 * s
g = 32.17405; % ft/s^2
s = 45; % ft^2
W = 1400; % lbf
c_l = 1.258; % c_L max
n_aero = q * s * c_l / W; % max aerodynamic load factor

% Structurally Limited
% Pullup Maneuver
R_s_pullup = v^2 / (g * (n_structure-1));
Omega_s_pullup = (180 / pi) * g * (n_structure-1) / v;

% Turning Maneuver
R_s_turn = v^2 / (g * sqrt(n_structure^2-1));
Omega_s_turn = (180/pi) * (g * sqrt(n_structure^2-1)) / v;

% Maneuvering Speed
v_maneuvering = sqrt( (2/(rho * c_l)) * (W/s) ) * sqrt(n_structure);

% Aerodynamically Limited
% Pullup Maneuver
R_a_pullup = v^2 / (g * (n_aero-1));
Omega_a_pullup = (180 / pi) * g * (n_aero-1) / v;

% Turning Maneuver
R_a_turn = v^2 / (g * sqrt(n_aero^2-1));
Omega_a_turn = (180/pi) * (g * sqrt(n_aero^2-1)) / v;

