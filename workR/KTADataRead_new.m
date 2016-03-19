%% Data input and interpolation script
% Reads the knee and ankle winter data and organizes into the correct forms
% and interpolates data for a finer resolution.

%% Load the Winter Ankle and Knee Data
% stride|Mean Angle|St. Dev|Moment|St. Dev|


global MBS_user;

CGA_Ankle
CGA_Knee
CGA_Hip
MBS_user.m = 80.0; % Mass of the body

% Length of divisions for interpolation
divisions = .1;

%% Kinematic data of Ankle (And Interpolated set from Winter data)
    % Read in raw data
    MBS_user.ankle.stride_raw = MBS_user.CGA_Ankle_Data(:,1);    % Winter Data
    MBS_user.ankle.theta_raw = MBS_user.CGA_Ankle_Data(:,2);     % Winter Data
    MBS_user.ankle.T_raw = -MBS_user.CGA_Ankle_Data(:,4)*MBS_user.m;       % Winter Data, times the Mass of Body
    % Determine time between steps
    MBS_user.ankle.stridetime = 1; % sec, Time for one stride (2 steps)
    MBS_user.ankle.timestep_raw = MBS_user.ankle.stridetime/(length(MBS_user.ankle.stride_raw)-1); % Time between each sample 
    % Determine Joint Velocity
    MBS_user.ankle.dtheta_raw = [0;diff(MBS_user.ankle.theta_raw)];           % Joint Velocity
    % Work and Power Calculations
    MBS_user.ankle.W_raw = MBS_user.ankle.T_raw.*(MBS_user.ankle.dtheta_raw*pi/180);   % Angular displacements in Radians
    MBS_user.ankle.P_raw = MBS_user.ankle.W_raw/MBS_user.ankle.timestep_raw;

    % Interpolated Winter data
    MBS_user.ankle.divisions = divisions;
    MBS_user.ankle.xi = (1:MBS_user.ankle.divisions:length(MBS_user.ankle.stride_raw))';
    MBS_user.ankle.stride = interp1(MBS_user.ankle.stride_raw,MBS_user.ankle.xi,'spline');
    MBS_user.ankle.theta = interp1(MBS_user.ankle.theta_raw,MBS_user.ankle.xi,'spline');
    MBS_user.ankle.dtheta = [0;diff(MBS_user.ankle.theta)]; %interp1(dtheta_raw,xi,'spline');
    MBS_user.ankle.T = interp1(MBS_user.ankle.T_raw,MBS_user.ankle.xi,'spline');

    MBS_user.ankle.timestep = MBS_user.ankle.stridetime/(length(MBS_user.ankle.stride)-1); % Time between each sample 

    MBS_user.ankle.W = MBS_user.ankle.T.*(MBS_user.ankle.dtheta*pi/180);    % Joint work over stride (= power at 1 stride/sec)
    MBS_user.ankle.Joules = trapz(MBS_user.ankle.W);
    MBS_user.ankle.P = MBS_user.ankle.W/MBS_user.ankle.timestep;   % Joint Power

%% Kinematic Data of Knee (And Interpolated set from Winter data)
MBS_user.knee.stride_raw = MBS_user.CGA_Knee_Data(:,1);       % Winter Data
MBS_user.knee.theta_raw = MBS_user.CGA_Knee_Data(:,2);     % Winter Data, modified to match coordinates we are using
MBS_user.knee.dtheta_raw = [0;diff(MBS_user.knee.theta_raw)];           % Joint Velocity
MBS_user.knee.T_raw = -MBS_user.CGA_Knee_Data(:,4)*MBS_user.m;         % Winter Data, modified to match coordinates/masses we are using

MBS_user.knee.W_raw = -(MBS_user.knee.T_raw.*(MBS_user.knee.dtheta_raw*pi/180));   % Angular displacements in Radians

MBS_user.knee.stridetime = 1; % sec, Time for one stride (2 steps)
MBS_user.knee.timestep_raw = MBS_user.knee.stridetime/(length(MBS_user.knee.stride_raw)-1); % Time between each sample 

MBS_user.knee.P_raw = MBS_user.knee.W_raw/MBS_user.knee.timestep_raw;

% Interpolated Winter data
MBS_user.knee.divisions = divisions;
MBS_user.knee.xi = (1:MBS_user.knee.divisions:length(MBS_user.knee.stride_raw))';
MBS_user.knee.stride = interp1(MBS_user.knee.stride_raw,MBS_user.knee.xi,'spline');
MBS_user.knee.theta = interp1(MBS_user.knee.theta_raw,MBS_user.knee.xi,'spline');
MBS_user.knee.dtheta = [0;diff(MBS_user.knee.theta)];
MBS_user.knee.ddtheta = [0;diff(MBS_user.knee.dtheta)];%interp1(dtheta_raw,xi,'spline');
MBS_user.knee.T = interp1(MBS_user.knee.T_raw,MBS_user.knee.xi,'spline');

MBS_user.knee.timestep = MBS_user.knee.stridetime/(length(MBS_user.knee.stride)-1); % Time between each sample 

MBS_user.knee.W = -(MBS_user.knee.T.*(MBS_user.knee.dtheta*pi/180));    % Joint work over stride (= power at 1 stride/sec)
MBS_user.knee.P = MBS_user.knee.W/MBS_user.knee.timestep;   % Joint Power

%% Kinematic Data of Hip (And Interpolated set from Winter data)
MBS_user.hip.stride_raw = MBS_user.CGA_Hip_Data(:,1);       % Winter Data
MBS_user.hip.theta_raw = MBS_user.CGA_Hip_Data(:,2);     % Winter Data, modified to match coordinates we are using
MBS_user.hip.dtheta_raw = [0;diff(MBS_user.hip.theta_raw)];           % Joint Velocity
MBS_user.hip.T_raw = MBS_user.CGA_Hip_Data(:,4)*MBS_user.m;         % Winter Data, modified to match coordinates/masses we are using

MBS_user.hip.W_raw = (MBS_user.hip.T_raw.*(MBS_user.hip.dtheta_raw*pi/180));   % Angular displacements in Radians

MBS_user.hip.stridetime = 1; % sec, Time for one stride (2 steps)
MBS_user.hip.timestep_raw = MBS_user.hip.stridetime/(length(MBS_user.hip.stride_raw)-1); % Time between each sample 

MBS_user.hip.P_raw = MBS_user.hip.W_raw/MBS_user.hip.timestep_raw;

% Interpolated Winter data
MBS_user.hip.divisions = divisions;
MBS_user.hip.xi = (1:MBS_user.hip.divisions:length(MBS_user.hip.stride_raw))';
MBS_user.hip.stride = interp1(MBS_user.hip.stride_raw,MBS_user.hip.xi,'spline');
MBS_user.hip.theta = interp1(MBS_user.hip.theta_raw,MBS_user.hip.xi,'spline');
MBS_user.hip.dtheta = [0;diff(MBS_user.hip.theta)]; %interp1(dtheta_raw,xi,'spline');
MBS_user.hip.T = interp1(MBS_user.hip.T_raw,MBS_user.hip.xi,'spline');

MBS_user.hip.timestep = MBS_user.hip.stridetime/(length(MBS_user.hip.stride)-1); % Time between each sample 

MBS_user.hip.W = -(MBS_user.hip.T.*(MBS_user.hip.dtheta*pi/180));    % Joint work over stride (= power at 1 stride/sec)
MBS_user.hip.P = MBS_user.hip.W/MBS_user.hip.timestep;   % Joint Power

%% Plotting Section

% figure(200)
% plot(MBS_user.ankle.stride,MBS_user.ankle.theta)
% hold on
% plot(MBS_user.knee.stride,MBS_user.knee.theta,'r')
% plot(MBS_user.hip.stride,MBS_user.hip.theta,'g')
% title('Knee and Ankle Angles (deg)')
% 
% figure(201)
% plot(MBS_user.ankle.stride,MBS_user.ankle.T)
% hold on
% plot(MBS_user.knee.stride,MBS_user.knee.T,'r')
% plot(MBS_user.hip.stride,MBS_user.hip.T,'g')
% title('Knee and Ankle Torques (Nm)')
% % 
% figure(202)
% plot(MBS_user.ankle.stride,MBS_user.ankle.P)
% hold on
% plot(MBS_user.knee.stride,MBS_user.knee.P,'r')
% plot(MBS_user.hip.stride,MBS_user.hip.P,'g')
% title('Knee and Ankle Power (W)')
