%% Atmosphere
clear,clc

% Atmosphere data will be necessary to determine aircraft
% performance at altitude, specifically density (needed
% for q) and viscosity (needed for Reynolds number).

% begin atmosphere input parameters
airport_alt = 157; % meters
cruise_alt = airport_alt + 150; % cruise altitude meters
v_cruise_approx = 59; % approximate cruise speed (m/s); to be used for Reynolds number calculations
a1 = -6.5e-3; % Temperature lapse rate below 11 km, in deg K/m
Temp0 = 288.16; % Nominal sea level temperature (std. atmosphere), deg K
rho0 = 1.225; % Air density at sea level, kg/m^3
Ratm = 287; % ideal gas constant J/(kg*K)
g_planet = 9.8; % acceleration due to gravity m/s^2
gamma_atm = 1.4; % ratio of specific heats
Beta = 1.458e-6; % kg/(s*m*sqrt(K)) from 1976 US standard atmosphere, page 19
Suth = 110.4; % deg K, Sutherland's constant, from 1976 US standard atmosphere, page 1
% end atmosphere input parameters

% now calculate Tcruise, Tairport, rhocruise, rhoairport,
% mucruise, muairport, c_soundcruise

Tcruise = Temp0 + a1 * cruise_alt; % Calculate temperature at cruise altitude based on lapse rate
Tairport = Temp0 + a1 * airport_alt; % Calculate temperature at airport altitude based on lapse rate
rhocruise = rho0 * (Tcruise/Temp0)^(-g_planet / (a1*Ratm)); % Calculate density at cruise altitude based on standard atmosphere formula
rhoairport = rho0 * (Tairport/Temp0)^(-g_planet / (a1*Ratm)); % Calculate density at cruise altitude based on standard atmosphere formula

mucruise = (Beta*Tcruise^(3/2))/(Tcruise+Suth);  % Dynamic viscosity from 1976 US standard atmosphere, page 19, eq 51.... Valid below 86 km
muairport = (Beta*Tairport^(3/2))/(Tairport+Suth);  % Dynamic viscosity from 1976 US standard atmosphere, page 19, eq 51.... Valid below 86 km
c_soundcruise = sqrt(gamma_atm*Ratm*Tcruise); % Speed of sound

fprintf(1,'%% atmosphere output results\n');
fprintf(1,'Tcruise=%.4g; %% cruise temperature (deg K)\n',Tcruise);
fprintf(1,'Tairport=%.4g; %% airport temperature (deg K)\n',Tairport);
fprintf(1,'rhocruise=%.4g; %% cruise density (kg/m^3)\n',rhocruise);
fprintf(1,'rhoairport=%.4g; %% airport density (kg/m^3)\n',rhoairport);
fprintf(1,'mucruise=%.4g; %% cruise viscosity (kg/m^3)\n',mucruise);
fprintf(1,'muairport=%.4g; %% airport viscosity (kg/m^3)\n',muairport);
fprintf(1,'c_soundcruise=%.4g; %% speed of sound at cruise (m/s)\n',c_soundcruise);
fprintf(1,'%% end atmosphere output results\n\n');

%% Plane

% rough cruise flight parameters: inputs
Swing = 5; % Wing area (m^2)
b = 5; % wing span (m)
% end rough cruise flight parameters: inputs

avgchord = Swing / b; % average wing chord (m)
WingRe_cruise = rhocruise * v_cruise_approx * avgchord / mucruise; % Wing Reynolds number at cruise
M_cruise = v_cruise_approx / c_soundcruise; % Mach number at cruise

% These are needed inputs for XFLR5 to calculate airfoil parameters
% (use cruise values because ultimate performance is usually most
% significant at cruise)
fprintf(1,'%% rough cruise flight parameter results\n');
fprintf(1,'avgchord=%.4g; %% average wing chord (m)\n',avgchord);
fprintf(1,'WingRe_cruise=%.4g; %% typical cruise Reynolds number\n',WingRe_cruise);
fprintf(1,'M_cruise=%.4g; %% typical cruise Mach number\n',M_cruise);
fprintf(1,'%% end rough cruise flight parameter results\n\n');

% Suggest load and plot your XFLR5 data here ...
taper = 0;
sweep = 0;

filename = 'SD8040';
datafile = [filename '.mat'];
if(exist(datafile,'file') ~= 2)
    xflr5file = [filename '.txt'];
    xflr5_save(xflr5file,datafile)
end
load(datafile)

plane = Airplane(v_cruise_approx,g_planet,c_soundcruise);
plane = plane.add_wing(avgchord,b,taper,sweep,rhocruise,mucruise,c_soundcruise);
plane = plane.Wing_2D(Airfoil_Data);
plane.plot

% airfoil input parameters (2D/3D)
a0 = plane.wing.a0; % lift slope in 1/deg
cd0 = plane.wing.c_d0; % zero lift drag coefficient
zeroliftalpha = plane.wing.alpha_L0; % zero lift angle of attack (deg) (alpha_L=0)
alphastall = plane.wing.alpha_range(2); % stall angle of attack (deg)
wing_sweepbackangle = sweep; % wing sweep back angle (degrees)
e = 0.7; % estimated span efficiency factor (airfoil alone)
% end airfoil input parameters (2D/3D)



% 2D lift model is cl = a0(alpha-zeroliftalpha)
% Treat cl,max as a0(alphastall-zeroliftalpha)
% 2D drag model is cd = cd0

% 3D wing lift model is CL = a(alpha-zeroliftalpha)
% 3D wing drag model is CD = CD0_wing + kCL^2

% perform calculations of CD0_wing, a, k, CL,max
CD0_wing = cd0; % this is the same as 2D cd0, remember?
AR = b^2 / Swing; % aspect ratio
a = plane.wing.a;
k = plane.wing.k;  % This is the wing k, based on span efficiency factor e,
% not the the whole airplane K based on Oswald efficiency eo
CLmax= plane.wing.c_L_range(2);

fprintf(1,'%% 3D airfoil output results\n');
fprintf(1,'CD0_wing=%.4g; %% Wing zero lift 3D drag\n',a);
fprintf(1,'a=%.4g; %% 1/deg\n',a);
fprintf(1,'k=%.4g; %% 3D airfoil quadratic drag coefficient\n',k);
fprintf(1,'CLmax=%.4g; %% Maximum CL prior to stall\n',CLmax);
fprintf(1,'%% end 3D airfoil output results\n\n');


%% Drag Buildup

% begin drag build-up/whole airplane input parameters
% Swing already specified above.
wing_Q = 1.1; % Wing interference factor

fuselage_length = 4.5; % fuselage length (m)
fuselage_dmax = 0.72; % fuselage max diameter (m)
fuselage_Swet = 3.4 * (fuselage_dmax * fuselage_length); % fuselage wetted area (m^2)

tail_Sexposed = 1.125;  % Tail planform area (m^2) (total for horizontal + vertical stabilizer)... or you can split them apart and do them separately
tail_toverc = 0.098;   % Tail thickness over chord
tail_xoverc = 0.3;   % Tail position of maximum thickness over chord
tail_sweepbackangle = 15; % Tail sweep back angle (degrees)
tail_avgchord = 0.5; % tail average chord (m)
tail_Q = 1.04;

% We do not have an external engine
% num_engines_drag =; % number of external engines in pods for drag calculation
% engine_length =; % (m)
% engine_dmax =; % (m)  % engine max diameter  (m)
% engine_Swet = ; % Wetted area around engine nacelle
% engine_Q = ; % Engine interference factor

% end drag build-up/whole airplane input parameters

% Now calculate the drag build-up/whole airplane
% parameters based on your 2D/3D airfoil parameters/results
% and the above parameters

% Have CD0_wing from above (but don't forget to consider wing
% interference factor when adding it into CD0)
fuselageRe_cruise = rhocruise * v_cruise_approx * fuselage_length / mucruise;
fuselage_Cf = 0.455 / (log10(fuselageRe_cruise)^2.58 * (1 + 0.144 * M_cruise^2)^0.65);
fuselage_f = fuselage_length / sqrt(2 * (fuselage_dmax/2)^2);
fuselage_FF = 1 + 60/fuselage_f^3 + fuselage_f/400;

CD0_fuselage = fuselage_Cf * fuselage_FF * fuselage_Swet / Swing;

fprintf(1,'%% fuselage drag results\n');
fprintf(1,'fuselageRe_cruise=%.4g;\n',fuselageRe_cruise);
fprintf(1,'fuselage_Cf=%.4g;\n',fuselage_Cf);
fprintf(1,'fuselage_f=%.4g;\n',fuselage_f);
fprintf(1,'fuselage_FF=%.4g;\n',fuselage_FF);
fprintf(1,'CD0_fuselage=%.4g;\n',CD0_fuselage);
fprintf(1,'%% end fuselage drag results\n\n');

tailRe_cruise = rhocruise * v_cruise_approx * tail_avgchord / mucruise;
tail_Swet = tail_Sexposed * (1.977 + 0.52*tail_toverc);
tail_Cf = 0.455 / (log10(tailRe_cruise)^2.58 * (1 + 0.144 * M_cruise^2)^0.65);
tail_FF = (1+(0.6*tail_toverc/tail_xoverc)+100*tail_toverc^4)*(1.34*c_soundcruise^0.18*cosd(tail_sweepbackangle)^0.28);

CD0_tail =  tail_Cf * tail_FF * tail_Swet * tail_Q / Swing;% include interference factor

fprintf(1,'%% tail drag results\n');
fprintf(1,'tailRe_cruise=%.4g;\n',tailRe_cruise);
fprintf(1,'tail_Cf=%.4g;\n',tail_Cf);
fprintf(1,'tail_FF=%.4g;\n',tail_FF);
fprintf(1,'CD0_tail=%.4g;\n',CD0_tail);
fprintf(1,'%% end tail drag results\n\n');

% We don't have an external engine
% engineRe_cruise= ;
% engine_Cf= ;
% engine_f=;
% engine_FF=;
% CD0_engines=; % include all engines and interference factors

% fprintf(1,'%% engine drag results\n');
% fprintf(1,'engineRe_cruise=%.4g;\n',engineRe_cruise);
% fprintf(1,'engine_Cf=%.4g;\n',engine_Cf);
% fprintf(1,'engine_f=%.4g;\n',engine_f);
% fprintf(1,'engine_FF=%.4g;\n',engine_FF);
% fprintf(1,'CD0_engines=%.4g;\n',CD0_engines);
% fprintf(1,'%% end engine drag results\n\n');

Strut_c_d0 = 0.000667;
Wheel_c_d0 = 0.002182;
CD0_misc = 3 * (Strut_c_d0 + Wheel_c_d0); % Other sources of CD0 drag
CD0 = CD0_misc + CD0_fuselage + CD0_wing*wing_Q + CD0_tail; % Whole airplane CD0... don't forget to include wing interference factor
eo = 0.68; % estimated Oswald efficiency factor (entire airplane)

K = 1 / (pi * eo * AR); % Whole airplane K

fprintf(1,'%% drag buildup output results\n');
fprintf(1,'CD0_misc=%.4g; %% Other wing-referenced drag contributions\n',CD0_misc);
fprintf(1,'CD0=%.4g; %% Whole airplane CD0\n',CD0);
fprintf(1,'eo=%.4g; %% Estimated Oswald efficiency (entire airplane)',eo);
fprintf(1,'K=%.4g; %% Whole airplane quadratic drag coefficient',K);
fprintf(1,'%% end drag buildup output results\n\n');
% Your whole airplane drag model is CD=CD0+KCL^2
% Your whole airplane lift model is unchanged from the wing


% Performance calculations for a typical mission profile:
% Takeoff... climb... cruise... loiter... land

%% Performance

% begin performance input parameters
W_cargo = 2670;
Wzerofuel = 6228 - W_cargo; % Loaded, zero-fuel weight (N) % Cargo is 2670
W = Wzerofuel + W_cargo;
% Wfuel = 0; % Initial fuel load (N)
m = 0; % Engine thrust/power altitude dependence exponent (Battery Powered)
E = 1000000000;

% For a Jet
% Ta_airport =; % Thrust available (airport,max)
% TSFC =; % Thrust-specific fuel consumption in (N/s)/N
% or for a Prop
FuelCellPower = 260000; % Watts
eta_pr = 0.9; % Propeller efficiency factor
eta_mo = 0.92; % Motor efficiency factor
Pa_airport = FuelCellPower * eta_pr * eta_mo; % Power available (airport, max) Watts
% BSFC = ; % Brake specific fuel consumption in (N/s)/W

% end performance input parameters

LovDmax = sqrt(1 / (4*K*CD0)); % (L/D)max
CL12ovCDmax = 3/4 * (1/(3*K*CD0^3))^(1/4); % (CL^(1/2)/CD)max
CL32ovCDmax = 1/4 * (3/(K*CD0^(1/3)))^(3/4); % (CL^(3/2)/CD)max

fprintf(1,'%% simple performance output results\n');
fprintf(1,'LovDmax = %.4g; %% (L/D)max\n',LovDmax);
fprintf(1,'CL12ovCDmax = %.4g; %% (CL^(1/2)/CD)max\n',CL12ovCDmax);
fprintf(1,'CL32ovCDmax = %.4g; %% (CL^(3/2)/CD)max\n',CL32ovCDmax);
fprintf(1,'%% end simple performance output results\n\n');


%% Takeoff

% takeoff input parameters
n_takeoff = 1.15; % takeoff load factor (typically 1.15)
vlo_factor = 1.1; % liftoff velocity relative to stall (typically 1.1)
alpha_wheelsdown = 0; % angle of attack when wheels on runway (deg)
mu_r = 0.02; % Rolling friction coefficient
takeoffclimbangle = 5; % gamma (deg); typically 5 deg.
% end takeoff input parameters


v_stall_takeoff = sqrt(2 * W / (rhoairport * Swing * CLmax)); % v_stall on takeoff (fully fueled) (m/s)
v_lo = vlo_factor * v_stall_takeoff; % liftoff velocity (m/s)
v_accel = 0.7 * v_lo;
CL_wheelsdown = a * (alpha_wheelsdown-zeroliftalpha); % CL of the wing with wheels on runway (during acceleration)
q_accel = 0.5 * rhoairport * v_accel^2; % Dynamic Pressure when v=0.7 v_Lo
T_accel = Pa_airport / v_accel; % Thrust force during acceleration when v=0.7 v_lo
D_accel = q_accel * Swing * (CD0 + K * CL_wheelsdown^2); % Drag force during acceleration when v=0.7 v_lo
L_accel = q_accel * Swing * CL_wheelsdown; % Lift force during acceleration when v=0.7 v_lo
a_takeoff = g_planet * (T_accel/W - D_accel/W - mu_r*(1-L_accel/W)); % acceleration at takeoff (m/s^2), calculated @ v=0.7 v_lo
sg = v_lo^2 / (2 * a_takeoff); % ground distance (m) = v_lo^2/(2*a_takeoff)
R_takeoffpullup = v_lo^2 / ((n_takeoff-1)*g_planet); % radius of pull-up maneuver (m)
str = R_takeoffpullup * sind(takeoffclimbangle); % distance for transition to flight (m)
htr = R_takeoffpullup - R_takeoffpullup * cosd(takeoffclimbangle); % height for transition to flight (m)
scl = (10.7 - htr)/tand(takeoffclimbangle); % climb distance to clear 35' (10.7 m) obstacle (if positive)
s_takeoff = sg + str + scl; % total takeoff distance

fprintf(1,'%% takeoff output results\n');
fprintf(1,'v_stall_takeoff=%.4g; %% v_stall on takeoff (fully fueled) (m/s)\n',v_stall_takeoff);
fprintf(1,'v_lo=%.4g; %% liftoff velocity (m/s)\n',v_lo);
fprintf(1,'CL_wheelsdown=%.4g; %% CL during wheels-down takeoff run\n',CL_wheelsdown);
fprintf(1,'D_accel=%.4g; %% Aerodynamic drag at 0.7v_lo during wheels-down takeoff run (N)\n',D_accel);
fprintf(1,'L_accel=%.4g; %% Aerodynamic lift at 0.7v_lo during wheels-down takeoff run (N)\n',L_accel);
fprintf(1,'a_takeoff=%.4g; %% takeoff acceleration (m/s^2)\n',a_takeoff);
fprintf(1,'sg=%.4g; %% ground distance (m)\n',sg);
fprintf(1,'str=%.4g; %% transition distance (m)\n',str);
fprintf(1,'htr=%.4g; %% transition height (m)\n',htr);
fprintf(1,'scl=%.4g; %% climb distance (m)\n',scl);
fprintf(1,'s_takeoff=%.4g; %% takeoff distance (m)\n',s_takeoff);
fprintf(1,'%% end takeoff output results\n\n');

%% Climb

% Climb to cruising altitude

v_RoC_airport = sqrt(2 * W * sqrt(K / (3 * CD0)) / (rhoairport * Swing));
q_RoC_airport = 0.5 * rhoairport * v_RoC_airport^2;
Pr_Roc_airport = v_RoC_airport * (q_RoC_airport * Swing * CD0 + K * W^2 / (q_RoC_airport * Swing));
Max_RoC_airport = (Pa_airport - Pr_Roc_airport) / W; % maximum rate of climb (airport altitude) (m/s)
% Ta_cruise = % Thrust available, max (cruise altitude) (N) (jet only)
Pa_cruise = FuelCellPower * eta_pr * eta_mo; % Power available, max (cruise altitude) (W) (prop only)
v_RoC_cruise = sqrt(2 * W * sqrt(K / (3 * CD0)) / (rhocruise * Swing));
q_RoC_cruise = 0.5 * rhocruise * v_RoC_cruise^2;
Pr_Roc_cruise = v_RoC_cruise * (q_RoC_cruise * Swing * CD0 + K * W^2 / (q_RoC_cruise * Swing));
Max_RoC_cruise = (Pa_cruise - Pr_Roc_cruise) / W; % maximum rate of climb (cruise altitude) (m/s)
% Service ceiling is define as the height where the max rate of climb
% falls below 100 ft/m (.508 m/s)
% Is your cruise altitude above the service ceiling? (this next
% line will cause an error if it is)
assert(Max_RoC_cruise > .508);

% Estimate time-to-climb using the average of the cruise and airport RoC's.
% (time = height/rate )
TimeToClimb = (cruise_alt - airport_alt - 10.7) / ((Max_RoC_airport+Max_RoC_cruise)/2); % (s) 10.7 (m) obstacle
% Fuel use in climb at max thrust would be Power or Thrust available (average)
% multiplied by BSFC or TSFC as appropriate, multiplied by time
% Wclimbfuel = ; % fuel used during climb
E_climb = ((Pr_Roc_airport + Pr_Roc_cruise)/2 * TimeToClimb) / (eta_pr * eta_mo); % Energy used during climb
E = E - E_climb; % Energy after climb
% Wfuel_after_climb = ; % Weight of fuel remaining after initial climb (N)

fprintf(1,'%% climb output results\n');
fprintf(1,'Max_RoC_airport=%.4g; %% max rate of climb at airport altitude (m/s)\n',Max_RoC_airport);
% jet only:
% fprintf(1,'Ta_cruise=%.4g; %% max thrust available at cruise altitude (N)\n',Ta_cruise);
% prop only
fprintf(1,'Pa_cruise=%.4g; %% max power available at cruise altitude (W)\n',Pa_cruise);
fprintf(1,'Max_RoC_cruise=%.4g; %% max rate of climb at cruise altitude (m/s)\n',Max_RoC_cruise);
fprintf(1,'TimeToClimb=%.4g; %% time to climb to cruise altitude (s)\n',TimeToClimb);
% fprintf(1,'Wclimbfuel=%.4g; %% Weight of fuel expended during climb (N)\n',Wclimbfuel);
% fprintf(1,'Wfuel_after_climb=%.4g; %% Weight of fuel remaining after climb (N)\n',Wfuel_after_climb);
fprintf(1,'E_climb=%.4g; %% Weight of fuel expended during climb (N)\n',E_climb);
fprintf(1,'E_after_climb=%.4g; %% Weight of fuel remaining after climb (N)\n',E);
fprintf(1,'%% end climb output results\n\n');

%% Cruise1

% cruise input parameter
CruiseDist = 482803; % Cruise distance (m)
% end cruise input parameter

% (In a battery powered aircraft you would be tracking battery capacity
% instead of weight)

% You can find the fuel used in (optimal) cruise by plugging CruiseDist
% into the appropriate range equation. W1 would be your
% (Wfuel_after_climb+Wzerofuel). Then solve for W2, which is
% equivalent to (Wfuel_after_cruise+Wzerofuel)
% Wfuel_after_cruise = ; % (N)

v_cruise = sqrt(2 * W * sqrt(K / CD0) / (rhocruise * Swing)); % Get a typical cruise velocity by finding the velocity that
% Gives you the optimal cruise range at the average of W1 and
% W2.

q_cruise =  0.5 * rhocruise * v_cruise^2;
Tr_cruise = q_cruise * Swing * CD0 + K * W^2 / (q_cruise * Swing); % Thrust required for steady level flight at cruise
Pr_cruise = Tr_cruise * v_cruise; % Power Required in cruise

TimeCruise = CruiseDist / v_cruise;
E_cruise = Pr_cruise * TimeCruise;
E = (E - E_cruise) / (eta_pr * eta_mo);

M_cruise = v_cruise / c_soundcruise; % Cruise mach number

fprintf(1,'%% cruise output results\n');
% fprintf(1,'Wfuel_after_cruise=%.4g; %% Weight of fuel remaining after cruise (N)\n',Wfuel_after_cruise);
fprintf(1,'E_after_cruise=%.4g; %% Weight of fuel remaining after cruise (N)\n',E);
fprintf(1,'v_cruise=%.4g; %% averaged cruise velocity (m/s)\n',v_cruise);
fprintf(1,'M_cruise=%.4g; %% Approximate cruise mach number\n',M_cruise);
fprintf(1,'%% end cruise output results\n\n');

assert(M_cruise < 0.85); % Equations we are using assume speed significantly below Mach 1

%% Loiter1

% loiter input parameter
% W = W - W_cargo;
LoiterTime = 3600/2; % Loiter time (s)
% end loiter input parameter

% You can find the fuel used in (optimal) loiter by plugging LoiterTime
% into the appropriate endurance equation. W1 would be your
% (Wfuel_after_cruise+Wzerofuel). Then solve for W2, which is
% equivalent to (Wfuel_after_loiter+Wzerofuel)
% Wfuel_after_loiter = ; % (N)

v_loiter = sqrt(2 * W * sqrt(K / (3 * CD0)) / (rhocruise * Swing)); % Get a typical loiter velocity by finding the velocity that
% Gives you the optimal loiter time at the average of W1 and W2.

q_loiter =  0.5 * rhocruise * v_loiter^2;
Tr_loiter = q_loiter * Swing * CD0 + K * W^2 / (q_loiter * Swing); % Thrust required for steady level flight at cruise
Pr_loiter = Tr_loiter * v_loiter; % Power Required in cruise

E_loiter = (Pr_loiter * LoiterTime) / (eta_pr * eta_mo);
E = E - E_loiter;

% Also a good idea to check that your v_loiter > v_stall
v_stall_loiter = sqrt(2 * W / (rhocruise * Swing * CLmax)); % stall speed at loiter altitude/avg. weight

assert(v_loiter >= v_stall_loiter); % will fail if v_loiter < v_stall

W = W - W_cargo;

fprintf(1,'%% loiter output results\n');
% fprintf(1,'Wfuel_after_loiter=%.4g; %% Weight of fuel remaining after loiter (N)\n',Wfuel_after_loiter);
fprintf(1,'E_after_loiter=%.4g; %% Weight of fuel remaining after loiter (N)\n',E);
fprintf(1,'v_loiter=%.4g; %% averaged loiter velocity (m/s)\n',v_loiter);
fprintf(1,'v_stall_loiter=%.4g; %% stall speed at loiter altitude (m/s)\n',v_stall_loiter);
fprintf(1,'%% end loiter output results\n\n');

%% Loiter2

% loiter input parameter
LoiterTime2 = 3600/4; % Loiter time (s)
% end loiter input parameter
v_loiter2 = sqrt(2 * W * sqrt(K / (3 * CD0)) / (rhocruise * Swing)); % Get a typical loiter velocity by finding the velocity that

q_loiter2 =  0.5 * rhocruise * v_loiter2^2;
Tr_loiter2 = q_loiter2 * Swing * CD0 + K * W^2 / (q_loiter2 * Swing); % Thrust required for steady level flight at cruise
Pr_loiter2 = Tr_loiter2 * v_loiter2; % Power Required in cruise

E_loiter2 = Pr_loiter2 * LoiterTime2;
E = (E - E_loiter2) / (eta_pr * eta_mo);

% Also a good idea to check that your v_loiter > v_stall
v_stall_loiter2 = sqrt(2 * W / (rhocruise * Swing * CLmax)); % stall speed at loiter altitude/avg. weight

assert(v_loiter2 >= v_stall_loiter2); % will fail if v_loiter < v_stall

fprintf(1,'%% loiter output results\n');
% fprintf(1,'Wfuel_after_loiter=%.4g; %% Weight of fuel remaining after loiter (N)\n',Wfuel_after_loiter);
fprintf(1,'E_after_loiter=%.4g; %% Weight of fuel remaining after loiter (N)\n',E);
fprintf(1,'v_loiter=%.4g; %% averaged loiter velocity (m/s)\n',v_loiter);
fprintf(1,'v_stall_loiter=%.4g; %% stall speed at loiter altitude (m/s)\n',v_stall_loiter);
fprintf(1,'%% end loiter output results\n\n');

%% Cruise 2

CruiseDist_2 = CruiseDist; % Cruise distance (m)
% end cruise input parameter

v_cruise = sqrt(2 * W * sqrt(K / CD0) / (rhocruise * Swing)); % Get a typical cruise velocity by finding the velocity that
% Gives you the optimal cruise range at the average of W1 and W2.

q_cruise =  0.5 * rhocruise * v_cruise^2;
Tr_cruise = q_cruise * Swing * CD0 + K * W^2 / (q_cruise * Swing); % Thrust required for steady level flight at cruise
Pr_cruise = Tr_cruise * v_cruise; % Power Required in cruise

TimeCruise = CruiseDist_2 / v_cruise;
E_cruise = (Pr_cruise * TimeCruise) / (eta_pr * eta_mo);
E = E - E_cruise;

fprintf(1,'%% cruise output results\n');
% fprintf(1,'Wfuel_after_cruise=%.4g; %% Weight of fuel remaining after cruise (N)\n',Wfuel_after_cruise);
fprintf(1,'E_after_cruise=%.4g; %% Weight of fuel remaining after cruise (N)\n',E);
fprintf(1,'v_cruise=%.4g; %% averaged cruise velocity (m/s)\n',v_cruise);
fprintf(1,'M_cruise=%.4g; %% Approximate cruise mach number\n',M_cruise);
fprintf(1,'%% end cruise output results\n\n');

%% Descent

% Typical descent uses a 3 degree glideslope
% This is like climbing (Lec 21) but theta in that perspective is negative.
% Assume a smooth transition from v_loiter at the top of descent
% to vapproach (typically 1.3vstall at airport elevation) at landing

% descent input parameters
theta_approach = 3; % typically 3 (deg)
vapproach_factor = 1.3; % vapproach/vstall... typically 1.3
% end descent input parameters

q_topofdescent =  0.5 * rhocruise * v_loiter^2;
Treq_topofdescent = q_topofdescent * Swing * CD0 + K * W^2 / (q_topofdescent * Swing); % Thrust required for steady level flight at top of descent (speed v_loiter)
Preq_topofdescent = v_loiter * Treq_topofdescent; % Power required for steady level flight at top of descent

v_stall_landing = sqrt(2 * W / (rhoairport * Swing * CLmax)); % v_stall on landing (airport altiude, most fuel gone, OK to neglect fuel consumption during descent in this calc.) (m/s)
v_approach = v_stall_landing * vapproach_factor; % approach velocity (m/s)
q_approach = 0.5 * rhoairport * v_approach^2;
Treq_approach = q_approach * Swing * CD0 + K * W^2 / (q_approach * Swing); % Thrust required for steady level flight at airport elevation and approach velocity (N) (also neglect fuel consumed during descent)
Preq_approach = Treq_approach * v_approach; % Power required for steady level flight at airport elevation and approach velocity (W) (also neglect fuel consumed during descent)


% From climb (lec 21), we had sin(theta) = (T-TR)/W
% where TR is the thrust required for steady level flight
% at the corresponding altitude.

% For descent sin(theta) is negative (theta=-theta_approach=-3deg)
% Solve the above equation for T (engine thrust setting for descent)
% both at loiter altitude and airport altitude;
% If a prop, multiply by corresponding v to get P_descent (engine
% power setting for descent). Then multiply by TSFC or
% BSFC as appropriate to get fuel flow rate. Multiply by
% approximate descent time (rate of descent = v*sin(theta_approach))
% to get fuel used in descent.

T_topofdescent = W * sind(theta_approach) + Treq_topofdescent; % engine thrust setting for top of descent (N)
T_approach = W * sind(theta_approach) + Treq_approach; % engine thrust setting for bottom of descent (N)
P_topofdescent = T_topofdescent * v_loiter; % engine power setting for top of descent (W)
P_approach = T_approach * v_loiter; % engine power setting for bottom of descent (W)
% The above numbers should be LESS THAN the thrust and power required
% numbers farther up because in descent some of the power is coming
% from work done by gravity, versus the Tr and Pr correspond to
% steady level flight.

% fuelflow_topofdescent = ; % (N/s)
% fuelflow_approach = ; % (N/s)
rateofdescent = v_loiter * sind(theta_approach);
descenttime = cruise_alt / airport_alt; % (s)
% fuelconsumption_descent = ; % (N)

E_descent = (Preq_approach+Preq_topofdescent)/2 * descenttime; % Energy used during descenttime
E = (E - E_descent) / (eta_pr * eta_mo);

% Wfuel_after_descent = ; % (N)

% assert(Wfuel_after_descent > 0); % must land with some fuel remaining
assert(E > 0);

fprintf(1,'%% descent output results\n');
fprintf(1,'Treq_topofdescent=%.4g; %% Thrust required for steady level flight at top of descent (N)\n',Treq_topofdescent);
fprintf(1,'Preq_topofdescent=%.4g; %% Power required for steady level flight at top of descent (W)\n',Preq_topofdescent);
fprintf(1,'v_stall_landing=%.4g; %% stall speed at landing altitude (m/s)\n',v_stall_landing);
fprintf(1,'v_approach=%.4g; %% approach velocity (m/s)\n',v_approach);
fprintf(1,'Treq_approach=%.4g; %% Thrust required for steady level flight at bottom of descent (N)\n',Treq_approach);
fprintf(1,'Preq_approach=%.4g; %% Power required for steady level flight at bottom of descent (W)\n',Preq_approach);

fprintf(1,'T_topofdescent=%.4g; %% Thrust setting for top of descent (N)\n',T_topofdescent);
fprintf(1,'T_approach=%.4g; %% Thrust setting for bottom of descent (N)\n',T_approach);
fprintf(1,'P_topofdescent=%.4g; %% Power setting for top of descent (W)\n',P_topofdescent);
fprintf(1,'P_approach=%.4g; %% Power setting for bottom of descent (W)\n',P_approach);

% fprintf(1,'fuelflow_topofdescent=%.4g; %% Fuel flow for top of descent (N/s)\n',fuelflow_topofdescent);
% fprintf(1,'fuelflow_approach=%.4g; %% Fuel flow for bottom of descent (N/s)\n',fuelflow_approach);

fprintf(1,'descenttime=%.4g; %% Descent time (s)\n',descenttime);
% fprintf(1,'fuelconsumption_descent=%.4g; %% Descent fuel weight consumed (N)\n',fuelconsumption_descent);
% fprintf(1,'Wfuel_after_descent=%.4g; %% Weight of fuel remaining after descent (N)\n',Wfuel_after_descent);
fprintf(1,'E_after_descent=%.4g; %% Weight of fuel remaining after descent (N)\n',E);


fprintf(1,'%% end descent output results\n\n');


%% Landing

% landing input parameters
h_obstacle = 15.2; % (m) ... obstacle height 50' (15.2 m)
theta_f = 3; % flare angle ... less than or equal to theta_a which is usually 3.0 deg.
n_flare = 1.2; % flare pullup maneuver load factor ... typically 1.2
mu_braking = 0.4; % Braking friction coefficient
% end landing input parameters

v_flare = 1.23 * v_stall_landing; % avg flare velocity (m/s)
v_touchdown = 1.15 * v_stall_landing; % touchdown velocity (m/s)


R_flare = v_flare^2 / ((n_flare-1)*g_planet); % Flare pullup maneuver radius (m)
sa = (15.2 - R_flare + R_flare * cosd(theta_approach))/tand(theta_approach); % approach distance from obstacle to start of flare (meters)
sf = R_flare * sind(theta_approach); % flare distance (meters)

% deceleration distance is v_touchdown^2/2a
% lift and drag affect deceleration; they are calculated in the
% wheels down configuration (based on CL_wheelsdown)
v_decel = 0.7 * v_touchdown;
q_decel = 0.5 * rhoairport * v_decel^2; % Dynamic Pressure when v=0.7 v_Lo
D_decel = q_decel * Swing * (CD0 + K * CL_wheelsdown^2); % Drag during deceleration @ 0.7 v_td (N)
L_decel = q_decel * Swing * CL_wheelsdown; % Lift during deceleration @ 0.7 v_td (N)
a_landing = g_planet * (D_decel/W + mu_braking*(1-L_decel/W)); % deceleration on landing, calculated @ v=0.7 v_td
sg_landing = v_touchdown^2 / (2 * a_landing); % ground distance (m) = v_lo^2/(2*a_landing)

% total landing distance
s_land =  sa + sf + sg_landing; % Total landing distanced

fprintf(1,'%% landing output results\n');
fprintf(1,'v_flare=%.4g; %% flare velocity (m/s)\n',v_flare);
fprintf(1,'v_touchdown=%.4g; %% touchdown velocity (m/s)\n',v_touchdown);
fprintf(1,'R_flare=%.4g; %% Flare pullup maneuver radius (m)\n',R_flare);
fprintf(1,'sa=%.4g; %% approach distance (m)\n',sa);
fprintf(1,'sf=%.4g; %% flare distance (m)\n',sf);
fprintf(1,'D_decel=%.4g; %% drag during deceleration @ 0.7v_td (N)\n',D_decel);
fprintf(1,'L_decel=%.4g; %% lift during deceleration @ 0.7v_td (N)\n',L_decel);
fprintf(1,'a_landing=%.4g; %% ground deceleration (m/s^2)\n',a_landing);
fprintf(1,'sg_landing=%.4g; %% ground deceleration distance (m)\n',sg_landing);
fprintf(1,'s_land=%.4g; %% landing distance (m)\n',s_land);
fprintf(1,'%% end landing output results\n\n');
