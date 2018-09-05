% clear all; close all; clc;
%
% MATLAB can use zeroMQ, but it is not necessarily so straight-forward. The easiest solution found was
% using "jeroMQ", which can be downloaded from https://github.com/zeromq/jeromq. After installation,
% Update the path below and you should be all set.
%
% For more information, check out:
% https://mathworks.com/matlabcentral/answers/269061-how-do-i-integrate-zeromq-library-with-matlab-i-want-my-matlab-program-to-be-a-subscriber-of-zeromq
% 
% Note: to install jeroMQ, you need to have 'maven' installed. When using Maven to install jeroMQ,
% you may run into an error about the unit testing. If so, disable them and run again using
% 'mvn install -DskipTests'
%
% Recommended Java JDK version: 1.8.0_171 (tested by excluding unit tests)
%
%

% Setup zeroMQ server
zmqServer = zeromqObj('/home/bmdoekemeijer/OpenFOAM/zeroMQ/jeromq-0.4.4-SNAPSHOT.jar',5553,300,true);

% Add FLORIS path and setup model
addpath(genpath('FLORISSE_M/FLORISSE_M'))
turbines = struct('turbineType', nrel5mw() , ...
                      'locIf', {[921.5, 1141.3]; [0877.6 1390.3]; [0833.7 1639.2]; ...
                                [1543.9 1251.0]; [1500.0 1500.0]; [1456.1 1749.0];...
                                [2166.3 1360.8]; [2122.4 1609.7]; [2078.5 1858.7]});
layout = layout_class(turbines, 'fitting_9turb'); 
layout.ambientInflow = ambient_inflow_log('WS', 8,'HH', 90.0,'WD', 0,'TI0', .06);
controlSet = control_set(layout, 'pitch');
subModels = model_definition('','rans','','selfSimilar','','quadraticRotorVelocity','', 'crespoHernandez');                    
florisRunner = floris(layout, controlSet, subModels);

% Initial control settings
yawAngleArrayOut   = 270.0*ones(1,layout.nTurbs);
pitchAngleArrayOut = 0.0*ones(1,layout.nTurbs);
controlTimeInterval  = 600; % Time to wait after applying control signal
measurementSignalAvgTime = 300; % Time to average data. Needs to be >= controlTimeInterval
sigma_WD = deg2rad(6)  % 6 deg for 8 m/s by M. Bertele, WES
dataSend = setupZmqSignal(yawAngleArrayOut,pitchAngleArrayOut);

disp(['Entering wind farm controller loop...']);
timeLastControl = 20e3; % -Inf: optimize right away. 0: optimize after controlTimeInterval (no prec), 20e3: optimize after controlTimeInterval (with prec)
measurementVector = [];
while 1
    % Receive information from SOWFA
    dataReceived = zmqServer.receive();
    currentTime  = dataReceived(1,1);
    measurementVector = [measurementVector;dataReceived(1,2:end)];
    
    % Optimize periodically
    if currentTime-timeLastControl >= controlTimeInterval
        disp([datestr(rem(now,1)) '__ Optimizing control at timestamp ' num2str(currentTime) '.']);
        
        % Time-average measurements
        if currentTime > controlTimeInterval % Past 1st iteration
            avgMeasurementVector = mean(measurementVector(end-measurementSignalAvgTime+1:end,:),1);
        end
        
        % Estimation
        measurementSet = struct();
        measurementSet.P = struct('values',avgMeasurementVector(1:3:end),'stdev',[1 1 1 2 2 2 3 3 3]);
        measurementSet.estimParams = {'TI0','Vref'}
        disp(measurementSet.P)
        
        disp([datestr(rem(now,1)) '__    Doing estimation cycle.']);
        
        florisRunner.clearOutput;
        estTool = estimator({florisRunner},{measurementSet});
        xopt = estTool.gaEstimation([0.0 6.0],[0.40 10.0]) % Estimate
        
        WD_measurements = 0.+sigma_WD*randn(1,9); % U=8m/s -> std = 6 deg. according to https://www.wind-energ-sci.net/2/615/2017/wes-2-615-2017.pdf
        florisRunner.layout.ambientInflow.windDirection = mean(WD_measurements);
        florisRunner.layout.ambientInflow.TI0  = xopt(1);
        florisRunner.layout.ambientInflow.Vref = xopt(2);
        clear xopt
        
        % Optimization
        disp([datestr(rem(now,1)) '__    Doing optimization cycle.']);
        [xopt,P_bl,P_opt] = optimizeControlSettingsRobust(florisRunner,'yawOpt',true,'pitchOpt',false,'axOpt',false,sigma_WD/3,5,true);
        yawAngleArray   = florisRunner.controlSet.yawAngleArray;
        pitchAngleArray = florisRunner.controlSet.pitchAngleArray;
                
        % Update message string
        yawAngleArrayOut   = round(270.-rad2deg(yawAngleArray),1);
        pitchAngleArrayOut = round(rad2deg(pitchAngleArray),1);        
        disp([datestr(rem(now,1)) '__    Synthesizing message string.']);
        dataSend = setupZmqSignal(yawAngleArrayOut,pitchAngleArrayOut);
        
        % Update time stamp
        timeLastControl = currentTime;
        measurementVector = [];
    end
    
    % Send a message (control action) back to SOWFA
    zmqServer.send(dataSend);
end
% Close connection
zmqServer.disconnect()

function [dataOut] = setupZmqSignal(yawAngles,pitchAngles)
    for i = 1:length(yawAngles)
        dataOut = [yawAngles(i) pitchAngles(i)];
    end
end