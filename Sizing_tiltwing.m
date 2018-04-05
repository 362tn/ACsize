clear,clc
%Vehicle sizing code AerE 362 Team A

%##########################################################################
%% ################### INPUTS #############################################
%##########################################################################

%% --------------------Vehicle input---------------------------------------

% FIXED WEIGHT
    W_FIX = 500;               %Fixed weight in pounds

% ROTOR
    D_ROT = [19];          %Rotor diameter in Feet
    A_ROT_V = (D_ROT./2).^2*pi; %Rotor area in Feet^2 vector
    N_ROT = 4;                  %Number of rotors

% DRAG AND REFERENCE AREA
    CD_V    = [.025];      %Coefficient of drag c_D0
    K       = 1/(pi*1*3);       %quadriatic drag coefficient
    S       = 144;               %Planform area ft^2

% EFFICIENCIES
    ERHO_V = [120];       %Energy density w*hr/kg
    ERHO_V = 1./(.000608277388.*ERHO_V); %Convert to lb/(hp*hr) for calc
    ETA_M = .95;                %Mechanical efficiency 
    PR_TR = 0;                  %Ratio of tail rotor to main rotor power
    FOM   = .8;                 %Figure of merit
    F     = 1.03;               %Downwash factor

    
%% --------------------Mission and atm inputs------------------------------
% DISTANCE AND TIME
    RNG_V = [30];           %Range for legs 4-5 and part of 13-14 (NMi)

    T_LOT = 20/60;          %Time to loiter hour
    T_VTOL= 4/60;           %Hover times hour

% ATMOSPHERE
    RHO = .002223;          %Air density slug/ft^3
    
%##########################################################################
%% ################### CALCULATIONS #######################################
%##########################################################################

%Output variable matricies
D(1) = length(CD_V);
D(2) = length(A_ROT_V);
D(3) = length(ERHO_V);
D(4) = length(RNG_V);
D(5) = 6;
POWER_R = zeros(D(1),D(2),D(3),D(4),D(5));       %Power required by mission segment
WEIGHT_E = zeros(D(1),D(2),D(3),D(4));           %Empty weight by configuration
POWER_MAX = WEIGHT_E;                       %Maximum power required by configuration
WEIGHT_GTOW = WEIGHT_E;                     %Gross takeoff weight 
WEIGHT_BAT = WEIGHT_E;                      %Total battery required by configuration
WEIGHT_BAT_S = POWER_R;                     %Battery required by mission segement
V_CRUISE = WEIGHT_E;                        %Cruise velocity by configuration
V_LOITER = WEIGHT_E;                        %Loiter velocity by configuration

for i=1:D(1) %LOOP FOR 1ST DIMENSION
    CD = CD_V(i);
    for j=1:D(2) %LOOP FOR 2ND DIMENSION
        A_ROT = A_ROT_V(j);
        for k=1:D(3)  %LOOP FOR 3RD DIMENSION
            ERHO = ERHO_V(k);
            for l=1:D(4) %LOOP FOR 4TH DIMENSION
                RNG = RNG_V(l);

                err_w = 100;          %Initial error start 
                step_w = 0;         %Weight secant counter
                while err_w>.1     %LOOP WHILE WEIGHT HAS MISMATCH

        %SECANT METHOD ON WEIGHT
                    step_w  = step_w+1;
                    %Secant method to produce new GTOW
                    if step_w ==1
                        GTOW(step_w) = 5000;   %Initial guess 
                    elseif step_w == 2
                        GTOW(step_w) = 4500;   %Second guess 
                    else
                        %Calculated for all other steps
                        GTOW(step_w) = GTOW(step_w-1)-ERROR(step_w-1)*...
                            ((GTOW(step_w-1)-GTOW(step_w-2))/...
                            (ERROR(step_w-1)-ERROR(step_w-2))); 
                    end            

                    %Corrlated weight storage
                    step_i = 1;                
                    WEIGHT_GTOW(i,j,k,l) = GTOW(step_w);            

    TIME(i,j,k,l,7) = 0.0; %Initialize the time vector

    % WARM UP: 1-2
    step_i = 2; %Mission step counter 
    WEIGHT_BAT_S(i,j,k,l,step_i-1) = WEIGHT_GTOW(i,j,k,l)*.01; %Battery usage in warm up
    TIME(i,j,k,l,step_i) = TIME(i,j,k,l,step_i-1) + 10/60; %hours, adds time for warm up -- assumes engine warm up time of 10 minutes

    % TAKE OFF: 2-3
    step_i = 3; %Mission step counter increases by 1
    t = WEIGHT_GTOW(i,j,k,l)/N_ROT; %thrust for each rotor, lbs
    POWER_R(i,j,k,l,step_i) = N_ROT*1.2*F*t*sqrt(F*t/(2*RHO*A_ROT))/550/FOM/ETA_M*(1+PR_TR); %power required for take off, HP
    WEIGHT_BAT_S(i,j,k,l,step_i-1) = POWER_R(i,j,k,l,step_i)*ERHO*T_VTOL; %fuel weight burned in vertical takeoff, lbs
    TIME(i,j,k,l,step_i) = TIME(i,j,k,l,step_i-1) + T_VTOL; %adding time for take off, hours    

    % CRUISE: 3-4
    step_i = 4;
    
    v_cr = sqrt((2*WEIGHT_GTOW(i,j,k,l)/(RHO*S)*sqrt(K/CD)));  %ft/s
    POWER_R(i,j,k,l,step_i) = (.5*RHO*S*CD*v_cr^3+(2*K*WEIGHT_GTOW(i,j,k,l)^2)/(RHO*S*v_cr))/550; %hp
    v_cr = v_cr*.592484;
    WEIGHT_BAT_S(i,j,k,l,step_i-1) = POWER_R(i,j,k,l,step_i)*ERHO*(RNG)/v_cr; %battery weight in lbs
    V_CRUISE(i,j,k,l) = v_cr;   %Knot
    
    % LOITER: 4-5
    step_i = 5;

    v_en = sqrt((2*WEIGHT_GTOW(i,j,k,l)/(RHO*S)*sqrt(K/(3*CD)))); %ft/s
    POWER_R(i,j,k,l,step_i) = (.5*RHO*S*CD*v_en^3+2*K*WEIGHT_GTOW(i,j,k,l)^2/(RHO*S*v_en))/550; %hp 
    WEIGHT_BAT_S(i,j,k,l,step_i-1) = POWER_R(i,j,k,l,step_i)*ERHO*T_LOT; %battery weight in lbs 
    V_LOITER(i,j,k,l) = v_en*.592484;   %Knot

    % LAND: 5-6
    step_i = 6; 
    
    t = WEIGHT_GTOW(i,j,k,l)/N_ROT; %thrust for each rotor using updated weight after loiter period, lbs
    POWER_R(i,j,k,l,step_i) = N_ROT*1.2*F*t*sqrt(F*t/(2*RHO*A_ROT))/550/FOM/ETA_M*(1+PR_TR); %hover power, HP
    WEIGHT_BAT_S(i,j,k,l,step_i-1) = POWER_R(i,j,k,l,step_i)*ERHO*T_VTOL; %fuel burned in landing, lbs
    TIME(i,j,k,l,step_i) = TIME(i,j,k,l,step_i-1) + T_VTOL; %adding time required for landing, hours


    %% ---------------------Weight correlation---------------------------------
                    %Determine maximum weight during mission
                    wt_max = WEIGHT_GTOW(i,j,k,l);
                    %Determine total fuel burn with fuel line reserve of 6%
                    WEIGHT_BAT(i,j,k,l) = sum(WEIGHT_BAT_S(i,j,k,l,:));
                    %Determine empty weight based on mission calculations
                    WEIGHT_E(i,j,k,l) = GTOW(step_w) - WEIGHT_BAT(i,j,k,l)*.75 - W_FIX;
                  %Correlation beteween max weight and empty weight estimation
                    
                    %wt_e_cor = 410.49+.666*wt_max;
                    wt_e_cor = (1-.05)*.9527*wt_max^.9295;
                    %wt_e_cor = .2236*wt_max^1.1697;
                    %wt_e_cor = (1-0)*.8578*wt_max^.9813; %Based on correlation from 11 tilt wing and rotor aircraft
                    %wt_e_cor = .4*wt_max+593.1;
                    %wt_e_cor = .49156*wt_max;%


                    %Difference of weights used in secant method above
                    ERROR(step_w) = (WEIGHT_E(i,j,k,l)-wt_e_cor);
                    %While loop condition
                    err_w = abs(ERROR(step_w));

    %Secant method for weights
                end
                %Store max power required
                POWER_MAX(i,j,k,l) = max(POWER_R(i,j,k,l,:));
                clear GTOW ERROR
            end
        end
    end
end
    