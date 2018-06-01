classdef EKFfilter < handle
   %EKF AHRS Eldar's implementation of AHRS algorithm
    %
    %   Date          Author          				Notes
    %   09/08/2016    R POmarnacki, E Sabanovic     Initial release
    
    %% Public properties
    properties (Access = public)
        approx_prediction = false;
        use_inertia_matrix = false; %neturime J matricos
        
        xa_apo;
        Pa_apo;
        Rot_matrix;
        eulerAngles = [0 0 0];
        debugOutput;
 
    end

    %% Private properties
    properties (Access = private)
        q_rotSpeed=10^-4; %Body angular rate process noise
        q_rotAcc=0.08; %Body angular acceleration process noise
        q_acc=0.009; %Acceleration process noise
        q_mag=0.005; %Magnet field vector process noise
        r_gyro=0.0008; %Gyro measurement noise
        r_accel=10000.0; %Accel measurement noise
        r_mag=100.0; %Mag measurement noise
        x_apo;
        P_apo;
        dt; %time delta
        
        % nurodome, kad turime acc, gyro ir mag duomenis
        zFlag=[1 1 1];
        
         % Moment of inertia matrix diagonal entry unit kg*m^2
        J=[0.0018 0 0;
           0 0.0018 0;
           0 0 0.0018];
    end    
    
    %% Public methods
    methods (Access = public)
        function obj = EKFfilter(varargin)
        end
        function obj = Update(obj, dt, Gyroscope, Accelerometer, Magnetometer)

            Accelerometer = Accelerometer * -9.81;    % normalise magnitude

            % Normalise magnetometer measurement
            if(norm(Magnetometer) == 0), return; end    % handle NaN
            Magnetometer = Magnetometer / norm(Magnetometer);   % normalise magnitude

            z = zeros(9,1);
            
            z(1) = Gyroscope(1);
            z(2) = Gyroscope(2);
            z(3) = Gyroscope(3);
            z(4) = Accelerometer(1);
            z(5) = Accelerometer(2);
            z(6) = Accelerometer(3);
            z(7) = Magnetometer(1);
            z(8) = Magnetometer(2);
            z(9) = Magnetometer(3);
            
            %dabar be x_apo ir P_apo
            [obj.xa_apo,obj.Pa_apo,obj.Rot_matrix,obj.eulerAngles,obj.debugOutput] = AttitudeEKF(obj.approx_prediction, obj.use_inertia_matrix,obj.x_apo,obj.P_apo, obj.zFlag,dt,z,obj.q_rotSpeed,obj.q_rotAcc,obj.q_acc,obj.q_mag,obj.r_gyro,obj.r_accel,obj.r_mag,obj.J);
            obj.x_apo = obj.xa_apo;
            obj.P_apo = obj.Pa_apo;
        end
    end
    
end

