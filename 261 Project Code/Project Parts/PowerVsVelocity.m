clear,clc

c_D0 = 0.047118076373558;
rho = 1.181087302711655; % Density
S = 5; % Planform
k = 0.093620554759938;
W = 3558; % N % Plane: 3558, Cargo: 2670, Total: 6228 
E = 1000000000; % J (1 Giga-joule)
E_m = E / 1000000000; % GJ

fprintf('Available Energy: %.1f Gigajoule\n\n',E_m)

eta_pr = 0.9;
eta_mo = 0.92;

v_ra = sqrt(2*W/(rho*S)*sqrt(k/c_D0)); % m/s
v_R = v_ra * 3.6; % km/hr
v_en = sqrt(2*W/(rho*S)*sqrt(k/(3*c_D0))); % m/s
v_E = v_en * 3.6; % km/hr

T_min = 0.5*rho*S*c_D0*v_ra^2+(2*k*W^2)/(rho*S*v_ra^2); % N
P_min = 0.5*rho*S*c_D0*v_en^3+(2*k*W^2)/(rho*S*v_en); % N * m / s

% Max Range
Range_max_meters = E * eta_pr * eta_mo / T_min; % m
Range_R = Range_max_meters / 1000; % km
Time_R = E * eta_pr * eta_mo / (T_min * v_ra) / 3600; % Hours

fprintf('Max Range: \n')
fprintf('\tDistance Travelled: %.0f Kilometers\n',Range_R)
fprintf('\t       Time in Air: %.1f Hours\n',Time_R)
fprintf('\t          Velocity: %.0f km/hr\n\n',v_R)

% Max Endurance
Endurance = E * eta_pr * eta_mo / P_min; % s
Time_E = Endurance / 3600; % Hours
Range_E_feet = Endurance * v_en; % m
Range_E = Range_E_feet / 1000; % km

fprintf('Max Endurance: \n')
fprintf('\tDistance Travelled: %.0f Kilometers\n',Range_E)
fprintf('\t       Time in Air: %.1f Hours\n',Time_E)
fprintf('\t          Velocity: %.0f km/hr\n\n',v_E)

% Graphs
v = 20:100;
T = 0.5*rho*S*c_D0*v.^2+(2*k*W^2)./(rho*S*v.^2); % N
P = 0.5*rho*S*c_D0*v.^3+(2*k*W^2)./(rho*S*v); % N * m / s
T_Range(1:length(v),1) = T_min;
P_Endurance(1:length(v),1) = P_min;

figure(1), cla, title 'Thrust vs Velocity'
hold on, grid on, xlabel 'Velocity ft/s', ylabel 'Thrust lbs'
plot(v,T)
plot(v,T_Range)
plot(v,P_Endurance./v_en)
legend('Required Thrust','Thrust_m_i_n (Max Range)', ... 
    'Power_m_i_n (Max Endurance)','Location','northwest')

figure(2), cla, title 'Power vs Velocity'
hold on, grid on, xlabel 'Velocity ft/s', ylabel 'Power lbs*ft/s'
plot(v,P)
plot(v,T_Range.*v_ra)
plot(v,P_Endurance)
legend('Required Thrust','Thrust_m_i_n (Max Range)', ... 
    'Power_m_i_n (Max Endurance)','Location','northwest')

