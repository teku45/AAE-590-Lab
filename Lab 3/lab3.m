%% tell them about this is the requirements, input is, output is
%% Starting Code 
delete(instrfind);
clear all;clc; close all
% mode_1 = heading control; mode_2 = speed control; mode_3 = waypoint
mode_1 = 0; mode_2 = 0; mode_3 = 1;
% Input the speed of the rover
wr.forward_spd = 100;
% loading vehicle parameters
vhc_param;
% Setup Serial and QTM communication
% Connect to QTM
QCM('connect', '127.0.0.1', '3D')

% Connect to Serial for wireless control
info = instrhwinfo('serial');
if isempty(info.AvailableSerialPorts)
   error('No ports free!');
end
s = serial(info.AvailableSerialPorts{end}, 'Baudrate', 57600, 'DataBits', 8, 'Timeout', 1);
% manual selection
% s = serial('com3', 'Baudrate', 9600, 'DataBits', 8, 'Timeout', 1);
fopen(s);

% Logfile preparation
thismoment = clock;
date_time = '';
for i = 1:5
    date_time = [date_time, num2str(thismoment(i), '%02d')];
    if i == 3
        date_time = [date_time, '_'];
    end
end

if exist('lab_data', 'dir') ~= 7
    mkdir('lab_data');
end
file_name = ['lab_data/data_',date_time,'.csv'];
fid = fopen(file_name,'w');

%% Read QTM and WP

% Retrieve current position data
labels3d = QCM('3dlabels'); % saves name of tracked objects
num_marker = size(labels3d, 2);
for i = 1:num_marker
    switch string(labels3d(i))
        case 'want_target'
            target.index = i;
        case 'tank_12_front'
            % front index
            wr.findex = i;
        case 'tank_12_back'
            % back index
            wr.bindex = i;
        case 'obstacle_one'
            % first obstacle index
            obs_1index = i;
        case 'obstacle_two'
            % second obstacle index
            obs_2index = i;
        otherwise
    end
end


nun_usable_marker = 5;
% Read initial data from QTM
rawData = zeros(nun_usable_marker, 5);
rData = QCM;
loop = 1;
disable = 0;
% reading waypoints frrm excel file
wr.WP = readmatrix("Rover_wp_lab3.xlsx", 'sheet','white_rover');
wr.numWP = size(wr.WP,1);
%% Command for rover 
header = 4;
command = 65;   % A = 65, (Autonomous)
roverID = 12;
% Communication
PWMV = 0; % For blimp
PWML = 0;
PWMR = 0;
DIRL = 1; % Direction (Left)
DIRR = 1; % Direction (Right)
%% Kill switch code
% disable is kill switch, ex) one loop period: 0.03s, then 0.03*50 = 0.15s
% want.goal_posz = 500mm from the ground
time.old = 0;
% reading the positional data to initialize parameters
rData = QCM;
% location of the front and back markers 
wr.front_marker = rData(wr.findex,1:2); 
wr.back_marker = rData(wr.bindex,1:2); 
wr.pos_old = (wr.front_marker + wr.back_marker)/2;
% center location of the obstacle
obs_first_marker = rData(obs_1index, 1:3);
obs_second_marker = rData(obs_2index, 1:3);
obs_pos_old = (obs_first_marker + obs_second_marker)/2;
tic
while(disable < 1)
    wr.disable = disable; 
    time.curr = toc;  
    time.dt = time.curr - time.old;
    wr.i2 = wr.i2 + 1;
    wr.i1 = 1;
    if wr.i2>5
        wr.i1 = wr.i2 - 4; 
    end
    % rData structure: x,y,z
    rData = QCM;
    % extracting the front and back marker positions
    wr.front_marker = rData(wr.findex,1:2); 
    wr.back_marker = rData(wr.bindex,1:2); 
    % calculating position of the rover center
    wr.pos = (wr.front_marker + wr.back_marker)/2;
    % storing the current position vector
    % targetz is the one marker on the wand
    targetz = rData(target.index, 3);
    % heading angle is calculated from 2 marker positions
    % one in the front and one in the back
    wr.heading_vec = wr.front_marker - wr.back_marker;
    % grabbing obstacle position data 
    obs_first_marker = rData(obs_1index, 1:3);
    obs_second_marker = rData(obs_2index, 1:3);
    obs_pos = (obs_first_marker + obs_second_marker)/2;
    % reference vector, in this case will be the x axis in the fixed frame
    % heading error (e_heading) is equal to theta meaning: 
    % if heading error > 0, heading error is (+)1
    % if heading error < 0, heading error is (-)

    % calling the controller (Task 1-3)
    if mode_1
        wr.heading_dir = [1,0];
        wr = wr_control_heading(wr, time);
        disp('Running heading test'); 
    elseif mode_2
        wr = wr_control_spd(wr, time);
        disp('Running speed test');
    else
        wr = wr_control_wp(wr, time, obs_pos);
        %disp('Running waypoint navigation');
    end



    % check if the last waypoint is reached by the rover
    if wr.curWP > wr.numWP
        disable = 99999;
    end

    % sending commands based on controller 
    wr.scommand = [header command PWMV wr.PWML wr.PWMR wr.DIRL wr.DIRR wr.ID 0 13];
    checksum = 0;
    for i = 2:1:8
        checksum = bitxor(checksum, wr.scommand(i));
    end
    wr.scommand(9) = checksum;
    % actrual command sent
    fwrite(s, wr.scommand);
    wr.scommand = 0;
    if targetz < 500
        disable = disable + 1;
    end
    %% Logging
    % data saving, more data can be added to here
    % Add data to data_to_log and redefine the first row of csv file
    %----------------------------------------------------------------------
    data_to_log = [double(time.curr), double(wr.pos(1)), double(wr.pos(2)),double(wr.heading_vec(1)),double(wr.heading_vec(2))];
    if loop == 1
        fprintf(fid, ['time[s], wr_x[mm], wr_y[mm], x_heading, y_heading \n']);
    %----------------------------------------------------------------------
        format_string = '';
        for i=1:1:length(data_to_log)
            format_string = strcat(format_string, '%3.4f');
            if i < length(data_to_log)
                format_string = strcat(format_string, ',');
            else
                format_string = strcat(format_string, '\n');
            end
        end
    end
    fprintf(fid, format_string, data_to_log);
    loop = loop + 1;
    time.old = time.curr;
end

%% Settings for terminating robot movement 
PWMV = 0; % For blimp
PWML = 0; % Speed (Left)
PWMR = 0; % Speed (Right)
DIRL = 1; % Direction (Left)
DIRR = 1; % Direction (Right)
scommand = [header command PWMV PWML PWMR DIRL DIRR roverID 0 13];
checksum = 0;
for i = 2:1:8
    checksum = bitxor(checksum, scommand(i));
end
scommand(9) = checksum;
fwrite(s, scommand);
scommand = 0; 

%% end of file, close all the connections 
disp('terminated');
fclose(fid);
fclose(s);
delete(instrfind);
QCM('disconnect');