function[dataStore] = zigzag(Robot,maxTime)
% TURNINPLACE: simple example program to use with iRobot Create (or simulator).
% Reads data from sensors, makes the robot turn in place and saves a datalog.
% 
%   dataStore = TURNINPLACE(Robot,maxTime) runs 
% 
%   INPUTStype
%       Robot       Port configurations and robot name (get from running CreatePiInit)
%       maxTime     max time to run program (in seconds)
% 
%   OUTPUTS
%       dataStore   struct containing logged data

% 
%   NOTE: Assume differential-drive robot whose wheels turn at a constant 
%         rate between sensor readings.
% 
%   Cornell University
%   MAE 5180: Autonomous Mobile Robots
%
% 	Modified: Liran 2023


% Set unspecified inputs
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    maxTime = 500;
end

try 
    % When running with the real robot, we need to define the appropriate 
    % ports. This will fail when NOT connected to a physical robot 
    CreatePort=Robot.CreatePort;
catch
    % If not real robot, then we are using the simulator object
    CreatePort = Robot;
end

% declare dataStore as a global variable so it can be accessed from the
% workspace even if the program is stopped
global dataStore;

% initialize datalog struct (customize according to needs)
dataStore = struct('truthPose', [],...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump', [], ...
                   'beacon', []);


% Variable used to keep track of whether the overhead localization "lost"
% the robot (number of consecutive times the robot was not identified).
% If the robot doesn't know where it is we want to stop it for
% safety reasons.
noRobotCount = 0;

SetFwdVelAngVelCreate(Robot, 0, 0);
wheel2Center = 0.13;
maxV = 0.5;
% Set angular velocity
fwdVelact = 0.3;
angVelact = 0.2;
zizgzagtime = 5;
over = 0;
tic

while toc < maxTime
    % READ & STORE SENSOR DATA
    [noRobotCount,dataStore]=readStoreSensorData(Robot,noRobotCount,dataStore);
    
    % CONTROL FUNCTION (send robot commands)
    if(toc > zizgzagtime && over == 0)
        A = [1 0
            0 1/0.2];
        pos = dataStore.truthPose(end, [2 3]);
        theta = dataStore.truthPose(end, 4);
        R_BI = [cos(theta) sin(theta)
                -sin(theta) cos(theta)];
        if(~exist('waypoints', 'var'))
            waypoints = [1 1
                        2 0
                        3 1];
            for i = 1:size(waypoints, 1)
                waypoints(i, :) = robot2global(dataStore.truthPose(end, [2 3 4]),...
                    waypoints(i, :));
            end
            gotopt = 1;
        end

        if(gotopt > size(waypoints,1))
            fwdVel = fwdVelact;
            angVel = angVelact;
            over = 1;
        else
            if(norm(waypoints(gotopt,:) - pos) > 0.1)
                pos = waypoints(gotopt,:) - pos;
                vel = A*R_BI*pos';
                fwdVel = vel(1);
                angVel = vel(2);
            else
                gotopt = gotopt + 1;
                fwdVel = 0;
                angVel = 0;
            end
        end
    else
        fwdVel = fwdVelact;
        angVel = angVelact;      
    end
    [cmdV,cmdW] = limitCmds(fwdVel,angVel,maxV,wheel2Center);
    
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(Robot, 0,0);
    else
        SetFwdVelAngVelCreate(Robot, cmdV, cmdW );
    end
%     pause(0.1);
end

% set forward and angular velocity to zero (stop robot) before exiting the function
SetFwdVelAngVelCreate(Robot, 0,0);