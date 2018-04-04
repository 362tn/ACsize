classdef Airplane
    %AIRPLANE Simulates an aiplane to estimate basic characteristics
    % Estimated features include range or endurance
    %
    % Must stick with unit system entered
    %
    % Entered Velocity is to estimate reynolds number, c_l, and c_d
    % Therefor data is not viable above ~ mach 0.7 or 0.8
    
    % Plane Parts
    properties
        wing
    end
    
    properties
        velocity
        mach
        mass
        weight
    end
    
    % Input Functions
    methods
        function obj = Airplane(velocity,g,speed_sound)
            obj.velocity = velocity;
            obj = obj.update(g,speed_sound);
            
        end
        function obj = add_wing(obj,chord,wingspan,taper,sweep,rho,mu,speed_sound)
            obj.wing = Wing(chord,wingspan,taper,sweep,obj.velocity,rho,mu,speed_sound);
        end
        function obj = Wing_2D(obj,WingData)
            %Wing_2D Adds 2D airfoil data and calculates values
            % WingData is a matrix with 3 columns arranged as:
            % alpha, c_l, c_d
            obj.wing = obj.wing.Update(WingData);
        end
    end
    
    % Outputs and Plots
    methods
        function plot(obj)
            %PLOT Plots both 2D and 3D graphs
            obj.wing.plot2D
            obj.wing.plot3D
            figure(1)
            legend('XFLR5 Data','C_l','C_L','Location','southeast')
            figure(2)
            legend('XFLR5 Data','2D','3D')
        end
    end
    
    % Update Functions
    methods (Access = private)
        function obj = update(obj,g,speed_sound)
            obj.mass = obj.weight / g;
            obj.mach = obj.velocity / speed_sound;
        end
    end
end

% Airplane.m
% Trey Green
% treybgreen@gmail.com
% 10/7/2017