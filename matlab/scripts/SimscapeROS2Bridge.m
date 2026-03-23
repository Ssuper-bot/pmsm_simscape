%% MATLAB-side bridge for ROS2 co-simulation
% This script runs alongside the Simscape model to exchange data with
% the ROS2 system via file-based bridge.
%
% Usage:
%   1. Start ROS2 nodes: ros2 launch pmsm_ros2 pmsm_system.launch.py
%   2. Run this script in MATLAB
%   3. Run the Simscape model
%
% Alternatively, use MATLAB's ROS Toolbox for direct ROS2 integration.

classdef SimscapeROS2Bridge < handle
    properties
        DataDir string = "/tmp/pmsm_bridge"
        SampleTime double = 0.001  % 1kHz
        Timer
    end
    
    properties (Access = private)
        LastPWM = [0.5, 0.5, 0.5]
    end
    
    methods
        function obj = SimscapeROS2Bridge(data_dir)
            if nargin > 0
                obj.DataDir = data_dir;
            end
            if ~exist(obj.DataDir, 'dir')
                mkdir(obj.DataDir);
            end
        end
        
        function write_motor_feedback(obj, ia, ib, ic, theta_e, omega_m)
            %WRITE_MOTOR_FEEDBACK Write motor sensor data for ROS2 node
            data = struct();
            data.ia = ia;
            data.ib = ib;
            data.ic = ic;
            data.theta_e = theta_e;
            data.omega_m = omega_m;
            data.timestamp = posixtime(datetime('now'));
            
            json_str = jsonencode(data);
            fid = fopen(fullfile(obj.DataDir, 'motor_feedback.json'), 'w');
            if fid ~= -1
                fprintf(fid, '%s', json_str);
                fclose(fid);
            end
        end
        
        function [da, db, dc] = read_pwm_duty(obj)
            %READ_PWM_DUTY Read PWM duty cycles from ROS2 controller
            pwm_file = fullfile(obj.DataDir, 'pwm_input.json');
            if exist(pwm_file, 'file')
                try
                    json_str = fileread(pwm_file);
                    data = jsondecode(json_str);
                    da = data.duty_a;
                    db = data.duty_b;
                    dc = data.duty_c;
                    obj.LastPWM = [da, db, dc];
                catch
                    da = obj.LastPWM(1);
                    db = obj.LastPWM(2);
                    dc = obj.LastPWM(3);
                end
            else
                da = obj.LastPWM(1);
                db = obj.LastPWM(2);
                dc = obj.LastPWM(3);
            end
        end
        
        function status = read_controller_status(obj)
            %READ_CONTROLLER_STATUS Read controller status from ROS2
            status_file = fullfile(obj.DataDir, 'status.json');
            if exist(status_file, 'file')
                try
                    json_str = fileread(status_file);
                    status = jsondecode(json_str);
                catch
                    status = struct('values', []);
                end
            else
                status = struct('values', []);
            end
        end
    end
end
