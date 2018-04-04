classdef Wing
    %WING
    
    % Units
    properties
        velocity
        mach
    end
    
    % Wing Data
    properties 
        c_d
        c_l
        alpha
        step
    end
    
    properties
        chord
        span
        sweep % Angle in degrees
        taper % Ratio
        planform
        AspectRatio
        Reynolds
        k
        a0
        a
        e = 0.7;
        e0 % Not in use
        c_d0
        alpha_L0
    end
    
    properties
        alpha_range
        c_l_range
        c_L_range
    end
    
    % User Called Functions
    methods
        function obj = Wing(chord,wingspan,taper,sweep,velocity,rho,mu,speed_sound)
            obj.chord = chord;
            obj.span = wingspan;
            obj.sweep = sweep;
            obj.velocity = velocity;
            obj.taper = taper;
            obj = init(obj,rho,mu,speed_sound);
        end
        function obj = Update(obj,WingData)
            %Update Adds 2D airfoil data and calculates values
            % WingData is a matrix with 3 columns arranged as:
            % alpha, c_l, c_d
            obj.alpha = WingData(:,1);
            obj.c_l = WingData(:,2);
            obj.c_d = WingData(:,3);
            
            obj = Update_2D(obj);
            obj = Update_3D(obj);
        end
    end
    
    % Lift, Drag, and Graphs based on alpha
    methods
        function c_l = lift2(obj,alpha)
            %LIFT2 Takes input alpha and approximates 2D c_l
            c_l = obj.a0 * (alpha - obj.alpha_L0);
        end
        function c_L = Lift(obj,alpha)
            %LIFT3 Takes input alpha and approximates 3D c_L
            c_L = obj.a * (alpha - obj.alpha_L0);
        end
        function c_Di = InducedDrag(obj,alpha)
            c_Di = obj.k * obj.Lift(alpha).^2;
        end
        function c_D = Drag(obj,alpha)
            %WingDrag Takes input alpha and approximates 3D c_D
            c_D = obj.c_d0 + obj.InducedDrag(alpha);
        end
        function plot2D(obj)
            %PLOT2D Plots the 2D airfoil data and equations
            % Must use Update_2D first
            
            % Alpha vs C_l
            figure(1)
            cla; hold on; grid on;
            xlabel 'alpha (degrees)'; ylabel 'coefficient of lift'; title 'alpha vs c_L';
            plot(obj.alpha,obj.c_l) % Actual Data
            Alpha = obj.alpha_range(1):obj.step:obj.alpha_range(2);
            C_l = obj.lift2(Alpha);
            plot(Alpha,C_l) % Linear Approximation
            
            % C_l vs C_d
            figure(2)
            cla; hold on; grid on;
            xlabel 'coefficient of lift'; ylabel 'coefficient of drag'; title 'c_L vs c_D';
            plot(obj.c_l,obj.c_d)
            plot(obj.c_l_range,[obj.c_d0 obj.c_d0])
        end
        function plot3D(obj)
            %PLOT3D Plots the 3D airfoil data and equations
            % Must use Update_3D first
            
            % Alpha vs C_L
            figure(1)
            Alpha = obj.alpha_range(1):obj.step:obj.alpha_range(2);
            C_L = obj.Lift(Alpha);
            plot(Alpha,C_L)
            
            % C_L vs C_D
            figure(2)
            C_D = obj.Drag(Alpha);
            plot(C_L,C_D)
            
        end
    end
    
    % Initiate and Update Functions
    methods (Access = private)
        function obj = init(obj,rho,mu,speed_sound)
            obj.mach = obj.velocity / speed_sound;
            obj.Reynolds = (rho * obj.velocity * obj.chord) / mu;
            obj.mach = obj.velocity / speed_sound;
            obj.planform = (obj.chord + obj.chord*obj.taper) / 2 * obj.span;
            obj.AspectRatio = obj.span.^2 / obj.planform;
        end
        function obj = Update_2D(obj)
            obj.step = obj.alpha(2) - obj.alpha(1);
            pos = find(obj.alpha==0);
            range = pos-1:pos+1;
            c = data_fit(obj.alpha(range),obj.c_l(range),1);
            obj.a0 = c(2);
            obj.alpha_L0 = -c(1) / c(2);
            
            % Find the range that alpha works on
            x = zeros(2,1);
            for i = 1:2
                y = c(1); t = pos;
                while abs(y - obj.c_l(t)) <= 0.1
                    if i == 1
                        t = t - 1;
                    else
                        t = t + 1;
                    end
                    if(t < 1)
                        break
                    end
                    x(i) = obj.alpha(t);
                    y = c(1) + x(i) * c(2);
                end
            end
            
            % ranges
            obj.alpha_range = x;
            obj.c_l_range = obj.a0 * (x - obj.alpha_L0);
            
            % c_d0
            obj.c_d0 = min(obj.c_d);
        end
        function obj = Update_3D(obj)
            %UPDATE_3D Takes the 2D airfoil data and updates it to 3D
            
            % 3D Slope for Lift
            num = obj.a0 * cosd(obj.sweep);
            den = 1 + (num / (pi * obj.e * obj.AspectRatio)) * (180/pi);
            obj.a = num / den;
            
            % 3D Drag (without plane body)
            obj.k = 1 / (pi * obj.e * obj.AspectRatio);
            obj.c_L_range = obj.a * (obj.alpha_range - obj.alpha_L0);
        end
    end
end

% Wing.m
% Trey Green
% treybgreen@gmail.com
% 10/7/2017