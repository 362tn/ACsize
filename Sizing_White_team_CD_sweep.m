clear,clc
%Vehicle sizing code Boeing White team

%##########################################################################
%% ################### INPUTS #############################################
%##########################################################################

%% --------------------Vehicle input---------------------------------------
% Configuration name
    NAME = 'COAX';          %Reference name for figures

% FIXED and CARGO WEIGHT
    W_FIX = 24000;           %Fixed weight in pounds
    W_EXP = 0;          %Expendeble cargo weight in pounds

% ROTOR
    D_ROT = [55 60];          %Rotor diameter in Feet
    A_ROT_V = (D_ROT./2).^2*pi; %Rotor area in Feet^2 vector
    N_ROT = 2;                  %Number of rotors

% DRAG AND REFERENCE AREA
    CD_V    = [.269 .3 .4];     %Coefficient of drag
    A_LIF = 100;             %Reference area of lifter feet^2
    A_CAR = 0;             %Reference area of cargo feet ^2
    A_TOT = A_LIF+A_CAR;    %Combined reference area feet^2

% EFFICIENCIES
    BSFC_V  = [.5 .503];   %BSFC vector lb/(hp*hr)
    ETA_M = .90;            %Mechanical efficiency 
    PR_TR = 0;              %Ratio of tail rotor to main rotor power
    FOM   = .7;             %Figure of merit
    F     = 1.03;           %Downwash factor

%% --------------------Mission and atm inputs------------------------------
% DISTANCE AND TIME
    RNG_1 = 100;           %Range for legs 4-5 and part of 13-14 (NMi)
    RNG_2 = 100;           %Range for legs 8-9, 12-13, and part of 13-14(NMi)

    T_LOT = 20/60;          %Time to loiter hour
    T_VTOL= 3/60;           %Hover times hour
% ACCELERATING FLIGHT
    ACC_G = 32.174;         %Acceleration of gravity ft/s^2
    ACC_R = .125*ACC_G;     %Acceleration rate ft/s^2
    C_ANG = 5.0;            %Climb angle degrees
    C_ALT = 1500;           %Altitude feet
% NUMBER OF TRIPS
    N_TRP = 1;              %Number of trips vector

% ATMOSPHERE
    RHO = .002223;          %Air density slug/ft^3

%##########################################################################
%% ################### CALCULATIONS #######################################
%##########################################################################

%Output variable matricies
D(1) = length(CD_V);
D(2) = length(A_ROT_V);
D(3) = length(BSFC_V);
D(4) = 7+max(N_TRP)*8;
v_fps = [5:1:340];
POWER_CR = zeros(D(1),D(2),D(3),2,length(v_fps)); %Power to cruise i,j,k: 1: cruise unloaded 2: cruise loaded 
POWER_CL = POWER_CR;                        %Power to climb i,j,k: 1: climb unloaded 2: climb loaded 
POWER_R = zeros(D(1),D(2),D(3),D(4));       %Power required by mission segment
WEIGHT_E = zeros(D(1),D(2),D(3));           %Empty weight by configuration
POWER_MAX = WEIGHT_E;                       %Maximum power required by configuration
WEIGHT_GTOW = WEIGHT_E;                     %Gross takeoff weight w/o container
WEIGHT_FTOT = WEIGHT_E;                     %Total fuel burn by configuration
WEIGHT_GW = POWER_R;                        %Gross weight by mission segment
WEIGHT_MAX = WEIGHT_E;                      %Max weight by configuration
WEIGHT_F = POWER_R;                         %Fuel burn by mission segement
%Optimum velocities by configuration (i,j,k,1) is no payload (i,j,k,2) is
%with payload
V_CRUISE = zeros(D(1),D(2),D(3),4);         %Cruise knots  1: unloaded cruise 2: loaded cruise 3: unloaded climb 4:loaded climb  
V_LOITER = V_CRUISE;                        %Loiter knots
CARGO_LB = POWER_R;                         %Payload by mission segment
TIME = POWER_R;                             %Time by mission segment

for i=1:D(1) %LOOP FOR 1ST DIMENSION
    CD = CD_V(i);
    for j=1:D(2) %LOOP FOR 2ND DIMENSION
        A_ROT = A_ROT_V(j);
        for k=1:D(3)  %LOOP FOR 3RD DIMENSION
            BSFC = BSFC_V(k);
            
            err_w = 100;          %Initial error start 
            step_w = 0;         %Weight secant counter
            while err_w>.1     %LOOP WHILE WEIGHT HAS MISMATCH
    %SECANT METHOD ON WEIGHT
                step_w  = step_w+1;
                %Secant method to produce new GTOW
                if step_w ==1
                    GTOW(step_w) = 20000;   %Initial guess 
                elseif step_w == 2
                    GTOW(step_w) = 18000;   %Second guess 
                else
                    %Calculated for all other steps
                    GTOW(step_w) = GTOW(step_w-1)-ERROR(step_w-1)*...
                        ((GTOW(step_w-1)-GTOW(step_w-2))/...
                        (ERROR(step_w-1)-ERROR(step_w-2))); 
                end
                
                %Corrlated weight storage
                step_i = 1;                
                WEIGHT_GW(i,j,k,step_i) = GTOW(step_w);
                WEIGHT_GTOW(i,j,k) = GTOW(step_w);
%% --------------------Cruise velocities-----------------------------------
    %DONE
    %CHECKED
            if step_w<=5
                
            for iv = 1:2
                %If loop to determine if loaded or not
                if iv == 1
                    %Unloaded cruise
                    gw_v= WEIGHT_GW(i,j,k,1);
                    a_ref = A_LIF;
                else 
                    %Loaded cruise
                    gw_v = WEIGHT_GW(i,j,k,1) +W_EXP;
                    a_ref = A_TOT;
                end
                v_ih = sqrt((F*gw_v)/(2*RHO*A_ROT*N_ROT)); %Hover Vi ft/s
                p_h = F*gw_v/(FOM*550)*v_ih*(1+PR_TR)/ETA_M; %Hover power HP
                p_tot = zeros(length(v_fps),1);
                for n=1:length(v_fps)
                    drag = 1/2*RHO*v_fps(n)^2*CD*a_ref; %Drag in cruise lb
                    alpha = atan(drag/(F*gw_v));          %Tilt angle rad
                    thrust = F*gw_v/cos(alpha);           %Thrust lb
                    %Parasitic power HP
                    p_para = drag*v_fps(n)/(550*FOM)*(1+PR_TR)/ETA_M; 
                    %Climb power
                    p_climb = v_fps(n)*sind(C_ANG)*gw_v/(550*FOM)*(1+PR_TR)/ETA_M;
                    %Secant method to induced cruise velocity ft/s
                    vi = sec_vi(v_ih,v_fps(n),alpha);  
                    %Induced power in HP
                    p_induc= F*gw_v*vi/(550*FOM)*(1+PR_TR)/ETA_M; 
                    p_tot(n) = p_para+p_induc;      %Total power HP
                    p_tot_cl(n) = p_para+p_induc+p_climb; %Total power to climb
                end
                
                POWER_CR(i,j,k,iv,:) = p_tot;       %Store power to cruse
                POWER_CL(i,j,k,iv,:) = p_tot_cl;     %Store power to climb
                
                
                clear n drag alpha thrust p_para vi p_induc 
                %Loiter velocity is minimum power required knots
                V_LOITER(i,j,k,iv) = v_fps(find(p_tot==min(p_tot)))/1.68781;
                V_LOITER(i,j,k,iv+2) = v_fps(find(p_tot_cl==min(p_tot_cl)))/1.68781;
                b_y = length(v_fps);
                b_y2 = b_y;
                for n = 1:(length(v_fps)-1)
                    m = (p_tot(n+1)-p_tot(n))/(v_fps(n+1)-v_fps(n)); %Slope
                    b_y(n) = abs(p_tot(n)-m*v_fps(n)); %y intercept abs value
                    m2 = (p_tot_cl(n+1)-p_tot_cl(n))/(v_fps(n+1)-v_fps(n)); %Slope
                    b_y2(n) = abs(p_tot_cl(n)-m2*v_fps(n)); %y intercept abs value
                end
                %Cruise velocity is tangent to line through orgin knots
                V_CRUISE(i,j,k,iv) = v_fps(find(b_y == min(b_y)))/1.68781;
                V_CRUISE(i,j,k,iv+2) = v_fps(find(b_y2 == min(b_y2)))/1.68781;
                clear n m b_y m2 b_y2
                clear p_tot p_climb gw_v a_ref v_ih p_h
            end
            clear iv
            
            end
            
                                  
%% --------------------Phase 1---------------------------------------------
TIME(i,j,k,1) = 0.0; %Initialize the time vector

% WARM UP: 1-2
step_i = 2; %Mission step counter 
WEIGHT_GW(i,j,k,step_i) = 0.99*WEIGHT_GW(i,j,k,step_i-1); %lbs
WEIGHT_F(i,j,k,step_i-1) = WEIGHT_GW(i,j,k,step_i) - WEIGHT_GW(i,j,k,step_i-1); %fuel burned during engine warm up, lbs
TIME(i,j,k,step_i) = TIME(i,j,k,step_i-1) + 10/60; %hours, adds time for warm up -- assumes engine warm up time of 10 minutes


% TAKE OFF: 2-3
step_i = 3; %Mission step counter increases by 1
t = WEIGHT_GW(i,j,k,step_i-1)/N_ROT; %thrust for each rotor, lbs
POWER_R(i,j,k,step_i) = N_ROT*1.2*F*t*sqrt(F*t/(2*RHO*A_ROT))/550/FOM/ETA_M*(1+PR_TR); %power required for take off, HP
WEIGHT_F(i,j,k,step_i-1) = (-POWER_R(i,j,k,step_i)*BSFC*T_VTOL); %fuel weight burned in vertical takeoff, lbs
WEIGHT_GW(i,j,k,step_i) = WEIGHT_GW(i,j,k,step_i-1) + WEIGHT_F(i,j,k,step_i-1); %weight at end of warm up - fuel burned in takeoff, lbs
TIME(i,j,k,step_i) = TIME(i,j,k,step_i-1) + T_VTOL; %adding time for take off, hours

%Calculate the time for accelerated climb:
cr_fps = V_CRUISE(i,j,k,3)*1.68781; %convert the cruise velocity for vehicle WITHOUT container from knots -> ft/sec
dt_accel = cr_fps/ACC_R/3600; %time to accelerate to cruise speed, hours
dt_climb = C_ALT/(cr_fps*sind(C_ANG))/3600; %time to climb to cruise altitude, hours
dt_accel_climb = dt_accel+dt_climb; %time for accelerated climb, hours

% CLIMB w/ ACCELERATION: 3-4
step_i = 4; %Mission step counter increases by 1
t = WEIGHT_GW(i,j,k,step_i-1)/N_ROT; %thrust for each rotor using updated weight, lbs
POWER_R(i,j,k,step_i) = N_ROT*F*t*sqrt(F*t/(2*RHO*A_ROT))/550/FOM/ETA_M*(1+PR_TR); %Hover Power used in fuel burn for climb/acceleration relation, HP
WEIGHT_F(i,j,k,step_i-1) = -0.83*POWER_R(i,j,k,step_i)*BSFC*dt_accel_climb; %approximation for fuel burn during accelerated climb, lbs
WEIGHT_GW(i,j,k,step_i) = WEIGHT_GW(i,j,k,step_i-1) + WEIGHT_F(i,j,k,step_i-1); %weight at end of warm up - fuel burned in takeoff, lbs
TIME(i,j,k,step_i) = TIME(i,j,k,step_i-1) + dt_accel_climb; %adding time for accelerating climb, hours


% CRUISE, base to port: 4-5
step_i = 5; %Mission step counter increases by 1
t = WEIGHT_GW(i,j,k,step_i-1)/N_ROT; %thrust for each rotor using updated weight, lbs

v_ih = sqrt(F*t/(2*RHO*A_ROT)); %induced hover velocity, ft/sec
vi = v_ih^2/cr_fps; %induced velocity, ft/sec 
p_ind = (t*vi/ETA_M/FOM/550*(1+PR_TR))*N_ROT; %induced power, HP
drag = 0.5*RHO*CD*cr_fps^2*A_LIF; %lbs
p_para = cr_fps*drag/(550*FOM*ETA_M)*(1+PR_TR); %parasitic power, HP 
p_tot = p_ind+p_para; %total power required, HP
POWER_R(i,j,k,step_i) = p_tot; %HP
dt_cr = RNG_1/V_CRUISE(i,j,k,1); %time to cruise from base to port, hours (NOTE: 4th index in V_CRUISE = 1 for unloaded vehicle condition)
WEIGHT_F(i,j,k,step_i-1) = -(p_tot)*BSFC*dt_cr; %fuel burned in cruise
WEIGHT_GW(i,j,k,step_i) = WEIGHT_GW(i,j,k,step_i-1) + WEIGHT_F(i,j,k,step_i-1); %weight at end of initial cruise from base to port, lbs
TIME(i,j,k,step_i) = TIME(i,j,k,step_i-1) + dt_cr; %adding time required for cruise from base to port, hours

clear t v_ih vi p_ind drag p_para p_tot dt_cr


%% --------------------Phase 2: looped-------------------------------------
            for nt=1:N_TRP
% PICKUP: 5-6
                step_i = step_i+1;      %Mission step_i counter
                t = WEIGHT_GW(i,j,k,step_i-1)/N_ROT;  %Thrust of each rotor
                POWER_R(i,j,k,step_i) = N_ROT*1.2*F*t/FOM*...%Power for hover
                    sqrt(F*t/(2*RHO*A_ROT))/550*(1+PR_TR)/ETA_M;
                %Fuel usage
                WEIGHT_F(i,j,k,step_i-1) = -POWER_R(i,j,k,step_i)*BSFC*T_VTOL;
                %Time for operation
                TIME(i,j,k,step_i) = TIME(i,j,k,step_i-1)+T_VTOL;
                %weight with fuel use and add cargo
                WEIGHT_GW(i,j,k,step_i) = WEIGHT_GW(i,j,k,step_i-1)+... 
                    WEIGHT_F(i,j,k,step_i-1)+W_EXP;
                clear t
    % TAKEOFF: 6-7
                step_i = step_i+1;      %Mission step_i counter
                t = WEIGHT_GW(i,j,k,step_i-1)/N_ROT;  %Thrust of each rotor
                POWER_R(i,j,k,step_i) = N_ROT*1.2*F*t/FOM*...%Power for hover
                    sqrt(F*t/(2*RHO*A_ROT))/550*(1+PR_TR)/ETA_M;
                %Fuel usage
                WEIGHT_F(i,j,k,step_i-1) = -POWER_R(i,j,k,step_i)*BSFC*T_VTOL;
                %Time for operation
                TIME(i,j,k,step_i) = TIME(i,j,k,step_i-1)+T_VTOL;
                %weight with fuel drop
                WEIGHT_GW(i,j,k,step_i) = WEIGHT_GW(i,j,k,step_i-1)+... 
                    WEIGHT_F(i,j,k,step_i-1);
                clear t
    % CLIMB w/ ACCEL: 7-8
                step_i = step_i+1;      %Mission step_i counter
                t = WEIGHT_GW(i,j,k,step_i-1)/N_ROT;  %Thrust of each rotor
                POWER_R(i,j,k,step_i) = N_ROT*F*t/FOM*...%Power for hover
                sqrt(F*t/(2*RHO*A_ROT))/550*(1+PR_TR)/ETA_M;
            
                %TIME REQUIREMENTS
                %Time to accelerate in hours
                dt_accel = (V_CRUISE(i,j,k,4)*1.68781)/ACC_R/3600; 
                %Time to climb in hours
                dt_climb = C_ALT/(V_CRUISE(i,j,k,4)*1.68781*sind(C_ANG))/3600;
                %Combine time
                dt_a_c = dt_accel+dt_climb;
                %Fuel usage
                WEIGHT_F(i,j,k,step_i-1) = -.83*POWER_R(i,j,k,step_i)*...
                    BSFC*dt_a_c;
                %Time for operation
                TIME(i,j,k,step_i) = TIME(i,j,k,step_i-1)+dt_a_c;
                %weight with fuel drop
                WEIGHT_GW(i,j,k,step_i) = WEIGHT_GW(i,j,k,step_i-1)+... 
                    WEIGHT_F(i,j,k,step_i-1);
                clear t dt_a_c dt_accel dt_climb

    % CRUISE: 8-9
                step_i = step_i+1;
                t = WEIGHT_GW(i,j,k,step_i-1)/N_ROT; %Thrust per rotor
                v_ih = sqrt(F*t/(2*RHO*A_ROT));     %Induced hover velocity
                %Estimated induced velocity for cruise
                vi = v_ih^2/(V_CRUISE(i,j,k,2)*1.68781);  
                %Induced power
                p_i = t*vi/FOM/550*(1+PR_TR)/ETA_M*N_ROT;
                %Drag for cruise
                drag = .5*RHO*(V_CRUISE(i,j,k,2)*1.68781)^2*CD*A_TOT;
                %Parasitic power
                p_par = (V_CRUISE(i,j,k,2)*1.68781)*drag/(550*FOM)*...
                    (1+PR_TR)/ETA_M;
                %Total power required
                POWER_R(i,j,k,step_i) = p_i+p_par;
                
                %Cruise time
                dt = RNG_2/(V_CRUISE(i,j,k,2));
                %Fuel burn
                WEIGHT_F(i,j,k,step_i-1) = -POWER_R(i,j,k,step_i)*BSFC*dt;
                %Gross weight after fuel burn
                WEIGHT_GW(i,j,k,step_i) = WEIGHT_GW(i,j,k,step_i-1)+...
                    WEIGHT_F(i,j,k,step_i-1);
                %Time after flight
                TIME(i,j,k,step_i) = TIME(i,j,k,step_i-1)+dt;
                clear t v_ih vi p_i drag p_par dt
                

% DROPOFF: 9-10
                step_i = step_i+1;      %Mission step_i counter
                t = WEIGHT_GW(i,j,k,step_i-1)/N_ROT;  %Thrust of each rotor
                POWER_R(i,j,k,step_i) = N_ROT*1.2*F*t/FOM*...%Power for hover
                    sqrt(F*t/(2*RHO*A_ROT))/550*(1+PR_TR)/ETA_M;
                %Fuel usage
                WEIGHT_F(i,j,k,step_i-1) = -POWER_R(i,j,k,step_i)*BSFC*T_VTOL;
                %Time for operation
                TIME(i,j,k,step_i) = TIME(i,j,k,step_i-1)+T_VTOL;
                %weight with fuel use and add cargo
                WEIGHT_GW(i,j,k,step_i) = WEIGHT_GW(i,j,k,step_i-1)+... 
                    WEIGHT_F(i,j,k,step_i-1)-W_EXP;
                clear t

% TAKEOFF: 10-11
                step_i = step_i+1;      %Mission step_i counter
                t = WEIGHT_GW(i,j,k,step_i-1)/N_ROT;  %Thrust of each rotor
                POWER_R(i,j,k,step_i) = N_ROT*1.2*F*t/FOM*...%Power for hover
                    sqrt(F*t/(2*RHO*A_ROT))/550*(1+PR_TR)/ETA_M;
                %Fuel usage
                WEIGHT_F(i,j,k,step_i-1) = -POWER_R(i,j,k,step_i)*BSFC*T_VTOL;
                %Time for operation
                TIME(i,j,k,step_i) = TIME(i,j,k,step_i-1)+T_VTOL;
                %weight with fuel drop
                WEIGHT_GW(i,j,k,step_i) = WEIGHT_GW(i,j,k,step_i-1)+... 
                    WEIGHT_F(i,j,k,step_i-1);
                clear t
% CLIMB w/ACCEL: 11-12
                step_i = step_i+1;      %Mission step_i counter
                t = WEIGHT_GW(i,j,k,step_i-1)/N_ROT;  %Thrust of each rotor
                POWER_R(i,j,k,step_i) = .83*N_ROT*F*t/FOM*...%Power for hover
                sqrt(F*t/(2*RHO*A_ROT))/550*(1+PR_TR)/ETA_M;
            
                %TIME REQUIREMENTS
                %Time to accelerate in hours
                dt_accel = V_CRUISE(i,j,k,3)*1.68781/ACC_R/3600; 
                %Time to climb in hours
                dt_climb = C_ALT/(V_CRUISE(i,j,k,3)*1.68781...
                    *sind(C_ANG))/3600;
                %Combine time
                dt_a_c = dt_accel+dt_climb;
                %Fuel usage
                WEIGHT_F(i,j,k,step_i-1) = -POWER_R(i,j,k,step_i)*BSFC*dt_a_c;
                %Time for operation
                TIME(i,j,k,step_i) = TIME(i,j,k,step_i-1)+dt_a_c;
                %weight with fuel drop
                WEIGHT_GW(i,j,k,step_i) = WEIGHT_GW(i,j,k,step_i-1)+... 
                    WEIGHT_F(i,j,k,step_i-1);
               clear t dt_a_c dt_accel dt_climb

% CRUISE: 12-13
                step_i = step_i+1;
                t = WEIGHT_GW(i,j,k,step_i-1)/N_ROT; %Thrust per rotor
                v_ih = sqrt(F*t/(2*RHO*A_ROT));     %Induced hover velocity
                %Estimated induced velocity for cruise
                vi = v_ih^2/(V_CRUISE(i,j,k,1)*1.68781);  
                %Induced power
                p_i = t*vi/FOM/550*(1+PR_TR)/ETA_M*N_ROT;
                %Drag for cruise
                drag = .5*RHO*(V_CRUISE(i,j,k,1)*1.68781)^2*CD*A_LIF;
                %Parasitic power
                p_par = (V_CRUISE(i,j,k,1)*1.68781)*drag/(550*FOM)*...
                    (1+PR_TR)/ETA_M;
                %Total power required
                POWER_R(i,j,k,step_i) = p_i+p_par;
                
                %Cruise time
                dt = RNG_2/(V_CRUISE(i,j,k,1));
                %Fuel burn
                WEIGHT_F(i,j,k,step_i-1) = -POWER_R(i,j,k,step_i)*BSFC*dt;
                %Gross weight after fuel burn
                WEIGHT_GW(i,j,k,step_i) = WEIGHT_GW(i,j,k,step_i-1)+...
                    WEIGHT_F(i,j,k,step_i-1);
                %Time after flight
                TIME(i,j,k,step_i) = TIME(i,j,k,step_i-1)+dt;
                clear t v_ih vi p_i drag p_par dt
                
                
                
                
            end

%% ---------------------Phase 3--------------------------------------------
% CRUISE: 13-14
step_i = step_i + 1; %Mission step counter increases by 1

t = WEIGHT_GW(i,j,k,step_i-1)/N_ROT; %thrust for each rotor using updated weight after container delivery trips, lbs

v_ih = sqrt(F*t/(2*RHO*A_ROT)); %induced hover velocity, ft/sec
vi = v_ih^2/cr_fps; %induced velocity, ft/sec 
p_ind = (t*vi/ETA_M/FOM/550*(1+PR_TR))*N_ROT; %induced power, HP
drag = 0.5*RHO*CD*cr_fps^2*A_LIF; %lbs
p_para = cr_fps*drag/(550*FOM*ETA_M)*(1+PR_TR); %parasitic power, HP 
p_tot = p_ind+p_para; %total power required, HP
POWER_R(i,j,k,step_i) = p_tot; %HP
dt_cr = RNG_1/V_CRUISE(i,j,k,1); %time to cruise from port to base (use only RNG_1), hours (NOTE: 4th index in V_CRUISE = 1 for unloaded vehicle condition)
WEIGHT_F(i,j,k,step_i-1) = -(p_tot)*BSFC*dt_cr; %fuel burned in cruise
WEIGHT_GW(i,j,k,step_i) = WEIGHT_GW(i,j,k,step_i-1) + WEIGHT_F(i,j,k,step_i-1); %weight at end of initial cruise from base to port, lbs
TIME(i,j,k,step_i) = TIME(i,j,k,step_i-1) + dt_cr; %adding time required for cruise from base to port, hours


% LOITER: 14-15
step_i = step_i + 1; %Mission step counter increases by 1

t = WEIGHT_GW(i,j,k,step_i-1)/N_ROT; %thrust for each rotor using updated weight after cruise back to base, lbs

%Calculate the loiter speed in ft/sec
loiter_fps = V_LOITER(i,j,k,2)*1.68781; %convert loiter velocity for unloaded vehicle from knots to ft/sec

v_ih = sqrt(F*t/(2*RHO*A_ROT)); %induced hover velocity, ft/sec
vi = v_ih^2/loiter_fps; %induced velocity, ft/sec 
p_ind = (t*vi/ETA_M/FOM/550*(1+PR_TR))*N_ROT; %induced power, HP
drag = 0.5*RHO*CD*loiter_fps^2*A_TOT; %lbs      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%CHANGES TO A_TOT
p_para = loiter_fps*drag/(550*FOM*ETA_M)*(1+PR_TR); %parasitic power, HP 
p_tot = p_ind+p_para; %total power required for loiter, HP
POWER_R(i,j,k,step_i) = p_tot; %HP
WEIGHT_F(i,j,k,step_i-1) = -(p_tot)*BSFC*T_LOT; %fuel burned in loiter
WEIGHT_GW(i,j,k,step_i) = WEIGHT_GW(i,j,k,step_i-1) + WEIGHT_F(i,j,k,step_i-1); %weight at end of required loiter period, lbs
TIME(i,j,k,step_i) = TIME(i,j,k,step_i-1) + T_LOT; %adding time required for loiter, hours


% LAND: 15-16
step_i = step_i + 1; %Mission step counter increases by 1
t = WEIGHT_GW(i,j,k,step_i-1)/N_ROT; %thrust for each rotor using updated weight after loiter period, lbs
POWER_R(i,j,k,step_i) = N_ROT*1.2*F*t*sqrt(F*t/(2*RHO*A_ROT))/550/FOM/ETA_M*(1+PR_TR); %hover power, HP
WEIGHT_F(i,j,k,step_i-1) = -POWER_R(i,j,k,step_i)*BSFC*T_VTOL; %fuel burned in landing, lbs
WEIGHT_GW(i,j,k,step_i) = WEIGHT_GW(i,j,k,step_i-1) + WEIGHT_F(i,j,k,step_i-1); %weight at end of mission, lbs
TIME(i,j,k,step_i) = TIME(i,j,k,step_i-1) + T_VTOL; %adding time required for landing, hours

%% ---------------------Weight correlation---------------------------------
                %Determine maximum weight during mission
                wt_max = max(WEIGHT_GW(i,j,k,:));
                %Determine total fuel burn with fuel line reserve of 6%
                WEIGHT_FTOT(i,j,k) = -1.06*sum(WEIGHT_F(i,j,k,:));
                %Determine empty weight based on mission calculations
                WEIGHT_E(i,j,k) = GTOW(step_w) - WEIGHT_FTOT(i,j,k) - W_FIX;
              %Correlation beteween max weight and empty weight estimation
                
                
                wt_e_cor = .4*wt_max+593.1;
                %wt_e_cor = .49156*wt_max;
                
                %Difference of weights used in secant method above
                ERROR(step_w) = WEIGHT_E(i,j,k)-wt_e_cor;
                %While loop condition
                err_w = abs(ERROR(step_w));

%Secant method for weights
            end
            % store max weight
            WEIGHT_MAX(i,j,k) = max(WEIGHT_GW(i,j,k,:));
            %Store max power required
            POWER_MAX(i,j,k) = max(POWER_R(i,j,k,:));
            clear GTOW ERROR
        end
    end
end
clear cr_fps drag dt_accel_climb dt_cr err_w i j k loiter_fps nt p_ind
clear p_para p_tot step_i step_w t v_ih vi wt_e_cor wt_max
%##########################################################################
%% ################### POST PROCESS #######################################
%##########################################################################

N_CREW = 0;             %Number of crew
RHO_F = 6.67632;        %Density of jet fuel lb/gal
PR_PA = .9;             %Ratio Power required for flight to total power required
CONTAINER_W = 8000;     %Weight of container

COST = zeros(D(1),D(2),D(3));   %Total mission cost
COST_LB = COST;                 %Cost per pound of cargo delivered
VOLUME_FTOT=WEIGHT_FTOT./RHO_F; %Volume of fuel gal
POWER_TOT = POWER_R/PR_PA; %Total power required


for i=1:D(1)
    for j=1:D(2)
        for k=1:D(3)
            %Cost from equation given by Boeing in White paper multiplited
            %by flight time
            COST(i,j,k) = (2500*sqrt(WEIGHT_E(i,j,k)*POWER_TOT(i,j,k))/10000+...
                75*(.04*WEIGHT_MAX(i,j,k)^.5+N_CREW)+...
                5*VOLUME_FTOT(i,j,k))*max(TIME(i,j,k,:));
            %Cost per pound of cargo delivered
            COST_LB(i,j,k) = COST(i,j,k)/((W_EXP-CONTAINER_W)*N_TRP);
        end
    end
end

%% PLOTS

figure('name',[NAME ': Total cost'],'NumberTitle','off')
for i=1:D(1)
    z(:,:) = COST(i,:,:);
    
    %Contour plot of total cost vs BSFC and Rotor diameter
    subplot(2,2,i)
    colormap(jet);
    contourf(BSFC_V,D_ROT,z)
    caxis([8000 20000]);            %IF Plots have bad colors change this
    xlabel('BSFC (lb/(HP*hr))')
    ylabel('Rotor Diameter (ft)')
    title(['Total cost for ' num2str(CD_V(i)) ' Coefficent of Drag'])
    colorbar
    cb = colorbar;
    ylabel(cb,'cost ($)')
    
end

figure('name',[NAME ': Cost per pound'],'NumberTitle','off')
for i= 1:D(1)
    
    z(:,:) = COST_LB(i,:,:);
    
    %Contour plot of  cost/cargo pound vs BSFC and Rotor diameter
    subplot(2,2,i)
    colormap(jet);
    contourf(BSFC_V,D_ROT,z)
    caxis([0 .4]);              %If plots have bad colors change this
    xlabel('BSFC (lb/(HP*hr))')    
    ylabel('Rotor Diameter (ft)')
    title(['Cost per pound for ' num2str(CD_V(i)) ' Coefficient of Drag'])   
    colorbar
    cb = colorbar;
    ylabel(cb,'specific cost ($/lb)')
end

%% Mission specific outputs

%Input configuration for mission plots to be output
%Can be vector but outputs several plots per configuration
D_ROT_OUT = [60];
BSFC_OUT = [.503];
CD_OUT = [.269 .3 .4];
%Plot vs time or segment number
X_VAR = 2;      %1 for segment %2 for time

clear i j k
%Looped for multiple configurations
for i = 1:length(CD_OUT)
    I = find(abs(CD_V - CD_OUT(i))<.00001);
    L_VEC = 8+8*N_TRP; 
    for j = 1:length(D_ROT_OUT)
        J = find(D_ROT == D_ROT_OUT(j));
        for k = 1:length(BSFC_OUT)
            K = find(abs(BSFC_V-BSFC_OUT(k))<.00001);
            
            %Vectors for output
            power_r(1:L_VEC) = POWER_R(I,J,K,1:L_VEC);
            time(1:L_VEC) = TIME(I,J,K,1:L_VEC);
            weight_f(2:L_VEC) = -WEIGHT_F(I,J,K,1:(L_VEC-1));
            weight_gw(1:L_VEC) = WEIGHT_GW(I,J,K,1:L_VEC);
            
            %If for plotting vs time or mission segment
            if X_VAR == 1
                x_lab = 'Segment Number';
                X = 1:L_VEC;
            else
                x_lab = 'Mission Time (Hr)';
                X = time;
            end
            
            fig_name = [': CD= ' num2str(CD_OUT(i)) ...
                ' Rotor Diam.= ' num2str(D_ROT_OUT(j)) 'ft. BSFC= '...
                num2str(BSFC_OUT(k)) ' lb/(HP*hr)'];
            
            %Plot gross weight
            figure('name',[NAME fig_name],'NumberTitle','off')
            subplot(2,2,1)
            plot(X,weight_gw)
            title('Gross weight')
            xlabel(x_lab)
            ylabel('Gross weight (lb)')
            
            %Plot fuel burn
            subplot(2,2,2)
            plot(X,weight_f)
            title('Fuel burn')
            xlabel(x_lab)
            ylabel('Fuel burn (lb)')
            
            %Plot power required for flight
            subplot(2,2,3)
            plot(X,power_r)
            title('Power required for flight')
            xlabel(x_lab)
            ylabel('Power required (Hp)')
            
            %Plot total power required given assumed fraction
            subplot(2,2,4)
            power_tot = power_r/PR_PA;
            plot(X,power_tot)
            title('Total power required')
            xlabel(x_lab)
            ylabel('Power required (Hp)')
            
            %Print data to command window
            fprintf([fig_name '\n \t Cruise velocties'...
                '\n\t\t unloaded max range: %4.0f knots'...
                '\n\t\t loaded max range: %4.0f knots'...
                '\n\t\t unloaded max endurance: %4.0f knots'...
                '\n\t\t laded max endurance: %4.0f knots'...
                '\n \t GTOW: %7.1f lbs'...
                '\n \t Empty weight: %7.1f lbs'...
                '\n \t Max weight: %7.1f lbs'...
                '\n \t Cost \n\t\t total: $%8.2f'... 
                '\n\t\t per pound of cargo: $%4.2f/pound \n\n'],...
                V_CRUISE(I,J,K,1),V_CRUISE(I,J,K,2),V_LOITER(I,J,K,1),...
                V_LOITER(I,J,K,2),WEIGHT_GTOW(I,J,K),WEIGHT_E(I,J,K),...
                WEIGHT_MAX(I,J,K),COST(I,J,K),COST_LB(I,J,K))
            
        end
    end
end

%Clear working variables
clear cb D A_ROT BSFC BSFC_OUT D D_ROT_OUT fig_name i I j J k K L_VEC 
clear N_TRIP_OUT N_TRP power_r power_tot time weight_f weight_gw X x_lab
clear X_VAR z

