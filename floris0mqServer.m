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
javaaddpath('/home/bmdoekemeijer/OpenFOAM/zeroMQ/jeromq-0.4.4-SNAPSHOT.jar','-dpath');
addpath(genpath('FLORISSE_M'))
import org.zeromq.*

% disp(['Setting up the server...']);
context = zmq.Ctx();
socket = context.createSocket(ZMQ.REP);
socket.bind('tcp://*:5553');

% Model set-up
layout = sowfa_9_turb;
layout.ambientInflow = ambient_inflow_log('WS', 8,'HH', 90.0,'WD', 0,'TI0', .05);
controlSet = control_set(layout, 'pitch');
subModels = model_definition('','rans','','selfSimilar','','quadraticRotorVelocity','', 'crespoHernandez');                    
florisRunner = floris(layout, controlSet, subModels);

% Initial control settings
yawAngleArrayOut   = 270.0*ones(1,layout.nTurbs);
pitchAngleArrayOut = 0.0*ones(1,layout.nTurbs);
controlTimeInterval  = 600; % Time to wait after applying control signal
measurementSignalAvgTime = 300; % Time to average data. Needs to be >= controlTimeInterval
sigma_WD = deg2rad(6)  % 6 deg for 8 m/s by M. Bertele, WES
str_send = createMessage(yawAngleArrayOut,pitchAngleArrayOut);

disp(['Entering loop for collecting and sending data...']);
timeLastControl = 20e3; % -Inf: optimize right away. 0: optimize after controlTimeInterval (no prec), 20e3: optimize after controlTimeInterval (with prec)
jTemp = 1;
measurementVector = [];
timeoutTimer = tic;
while 1

	%% Receive information from SOWFA
    message = socket.recv(1); % Do not wait for transmitter
    if length(message) <= 0
        pause(0.01); % wait 10 ms
		if toc(timeoutTimer) > 300 % 5-minute time out (no response from OpenFOAM)
			break % exit control loop
		end
    else
        json_data = native2unicode(message.data)'; % Received message

		% cut down json_data to remove all non-used bytes
		sortedFloats = textscan( json_data, '%f', 'Delimiter',' ' );
		inputsToSSC  = sortedFloats{1}'; % row vector with data
        currentTime = inputsToSSC(1,1);
        measurementVector = [measurementVector;inputsToSSC(1,2:end)];
        
		% disp([datestr(rem(now,1)) '__ Message received: [' num2str(inputsToSSC) '].']);		
        
        
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
%             florisRunner.layout.ambientInflow.Vref = 8.0+1*randn;
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
            [xopt,P_bl,P_opt] = optimizeControlSettingsRobust(florisRunner,'yawOpt',true,'pitchOpt',false,'axOpt',false,sigma_WD,5,true);
            yawAngleArray   = florisRunner.controlSet.yawAngleArray;
            pitchAngleArray = florisRunner.controlSet.pitchAngleArray;

			yawAngleArrayOut   = round(270.-rad2deg(yawAngleArray),1);
			pitchAngleArrayOut = round(rad2deg(pitchAngleArray),1);
			
            % Update message string
            disp([datestr(rem(now,1)) '__    Synthesizing message string.']);
            str_send = createMessage(yawAngleArrayOut,pitchAngleArrayOut);
            
            % Update time stamp
            timeLastControl = currentTime;
            measurementVector = [];
			timeoutTimer = tic;
        end
		
        % Send a message (control action) back to SOWFA
        disp([datestr(rem(now,1)) '__ Sending control message: [' str_send ']']);
        message = zmq.Msg(length(str_send));
        message.put(unicode2native(str_send));
        socket.send(message, 0);
    end
end
% Close connection
socket.close()

function [str_send] = createMessage(yawAngles,pitchAngles)
    % Initial string with control settings
    str_send = [num2str(yawAngles(1)) ' ' num2str(pitchAngles(1))];
    for i = 2:length(yawAngles)
        str_send = [str_send ' ' num2str(yawAngles(i)) ' ' num2str(pitchAngles(i))];
    end
end