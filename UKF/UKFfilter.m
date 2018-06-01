classdef UKFfilter < handle
   %UKF AHRS
    %
    %   Date          Author          Notes
    %   09/08/2016    E Sabanovic     Initial release
    
    %% Public properties
    properties (Access = public)
        
        eulerAngles = [0 0 0 0 0 0];
        %Rezultatu masyvai
        Rout = zeros(1,3);
        dwout = zeros(1,3);
        wout = zeros(1,3);
        aout = zeros(1,3);
        mout = zeros(1,3);
        TRIADout = zeros(1,3);        
 
    end

    %% Private properties
    properties (Access = private)
        abc=0;
        Parameters;
        Filter;
    end    
    
    %% Public methods
    methods (Access = public)
        function obj = UKFfilter(varargin)
        end
        function obj = Update(obj, dt, Gyroscope, Accelerometer, Magnetometer)

            Accelerometer = Accelerometer * -9.81;    % normalise magnitude

            % Normalise magnetometer measurement
            if(norm(Magnetometer) == 0), return; end    % handle NaN
            Magnetometer = Magnetometer / norm(Magnetometer);   % normalise magnitude
            

            
            if (obj.abc ~= 1)
                %AHRS parameters
                %Magnetic Field Vector In Navigation Frame
                obj.Parameters(1).mn = [0.315777529635464; 0.057133095826051; -0.947111588535720];
                %Acceleration vector In Navigation Frame
                obj.Parameters(1).an = [0; 0; -1];
                %Sampling Rate 1/Hz
                obj.Parameters(1).dT = dt;

                %Initial attitude quaternion value
                q = angle2quatMANO(0.0, 0.0, 0.0, 'ZYX');
                %Initial State Vector
                obj.Filter(1).x = [q'; 0; 0; 0];
                %Initial Covariance Matrix
                obj.Filter(1).P = eye(7,7)*10^-5;
                %Measuremens Noise
                obj.Filter(1).R = eye(6,6);
                %System Noise
                obj.Filter(1).Q = diag([10^-4 10^-4 10^-4 10^-4 10^-10 10^-10 10^-10]);
            end
            
            obj.abc=1;
            
            %Sensors(1).w = [Gyroscope(1) Gyroscope(2) Gyroscope(3)];
            %Sensors(1).a = [Accelerometer(1) Accelerometer(2) Accelerometer(3)];
            %Sensors(1).m = [Magnetometer(1) Magnetometer(2) Magnetometer(3)];
              
            % AHRS Part
            
            Wb = [Gyroscope(1) Gyroscope(2) Gyroscope(3)];
            Ab = [Accelerometer(1) Accelerometer(2) Accelerometer(3)];
            Mb = [Magnetometer(1) Magnetometer(2) Magnetometer(3)];

            An = obj.Parameters.an;
            Mn = obj.Parameters.mn;
            dT = obj.Parameters.dT;

            %Measurements vector
            z = [Ab'; Mb'];

            %Unscendent Kalman Filter
            obj.Filter = updateUKF(obj.Filter, z, Wb, An, Mn, dT);
              
            [Rout(1,1) Rout(1,2) Rout(1,3)] = quat2angle(obj.Filter(1).x(1:4)');
            dwout(1,:) = [obj.Filter(1).x(5) obj.Filter(1).x(6) obj.Filter(1).x(7)];
    
            % TRIAD part            
            W1 = Ab(1,:) / norm(Ab(1,:),'fro');
            W2 = Mb(1,:) / norm(Mb(1,:),'fro');
    
            V1 = obj.Parameters(1).an' / norm(obj.Parameters(1).an','fro');
            V2 = obj.Parameters(1).mn' / norm(obj.Parameters(1).mn','fro');
    
            Ou1 = W1;
            Ou2 = cross(W1,W2) / norm(cross(W1,W2),'fro');
            Ou3 = cross(W1,cross(W1,W2)) / norm(cross(W1,W2),'fro');
    
            R1 = V1;
            R2 = cross(V1,V2) / norm(cross(V1,V2),'fro');
            R3 = cross(V1,cross(V1,V2)) / norm(cross(V1,V2),'fro');
    
            Mou = [Ou1' Ou2' Ou3'];
            Mr = [R1' R2' R3'];
    
            A = Mou * Mr';
    
            % Calculate angles
            [TRIADout(1,1) TRIADout(1,2) TRIADout(1,3)]= dcm2angle(A,'ZYX');
            
            obj.eulerAngles = [Rout(1,3) Rout(1,2) Rout(1,1) TRIADout(1,1) TRIADout(1,2) TRIADout(1,3)];
    
        end
    end
    
end