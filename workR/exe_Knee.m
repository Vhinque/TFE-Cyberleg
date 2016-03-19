%--------------------------------------------------------------------------
%   Université catholique de Louvain
%   CEREM : Centre for research in mechatronics
%   http://www.robotran.be
%   Contact : robotran@prm.ucl.ac.be
%   Version : ROBOTRAN $Version$
%
%   Project : Knee prosthesis
%   Author :  Charles Boussemaere & Virgil Hinque
%   Date :    10/12/2015
%--------------------------------------------------------------------------

%% 1. Initialization and Project Loading [mbs_load]
%--------------------------------------------------------------------------
close all; clear variables; clc;                                            % Cleaning of the Matlab workspace
global MBS_user;                                                            % Declaration of the global user structure
MBS_user.process = '';                                                      % Initialisation of the user field "process"
MBS_user.step = 1;


% Project loading
prjname = 'Knee';
[mbs_data, mbs_info] = mbs_load(prjname,'default');                         % Option 'default': automatic loading of "$project_name$.mbs"
mbs_data_ini = mbs_data;                                                    % Backup of the initial multibody data structure

KTADataRead_new;                                                            % Load all Winter data
MBS_user.T_STRIDE = 1.0;


%% 2. Coordinate partitioning [mbs_exe_part]                                % For constrained MBS only
%--------------------------------------------------------------------------
MBS_user.process = 'part';
MBS_user.step = 2;


opt.part = {'rowperm','yes','threshold',1e-9,'verbose','yes'};
% Options for the coordinate partitioning.

[mbs_part,mbs_data] = mbs_exe_part(mbs_data,opt.part);
% Run the coordinate partitioning process


% Coordinate partitioning results
disp('Coordinate partitioning results');
disp(['Sorted independent variables = ', mat2str(mbs_part.ind_u)]);
disp(['Permutated dependent variables = ', mat2str(mbs_part.ind_v)]);
disp(['Permutated independent constraints = ', mat2str(mbs_part.hu)]);
disp(['Redundant constraints = ', mat2str(mbs_part.hv)]);



%% 3. Equilibrium [mbs_exe_equil]
%--------------------------------------------------------------------------
% MBS_user.process = 'equil';
% MBS_user.step = 3;
% 
% 
% opt.equil = {'solvemethod','fsolvepk',...
%     'relax',1.0,'itermax',30,'verbose','no'};
% %other options : 'smooth', 'xeqchoice', 'visualize', 'clearmbsglobal'      % Help about options on www.robotran.be
% %                'senstol', 'equitol', 'static'
% 
% MBS_user.index_stride_equil = 1;
% 
% 
% for i = 1:501
%     
%     [mbs_equil,mbs_data] = mbs_exe_equil(mbs_data,opt.equil);                   % Equilibrium process
%     
%     MBS_user.equil.phi_knee(i) = mbs_data.q(1);
%     MBS_user.equil.x_slider(i) = mbs_data.q(4);
%     MBS_user.equil.x_carriage(i) = mbs_data.q(3);
%     
%     MBS_user.equil.F_carriage(i) = mbs_data.Qq(3);
%     MBS_user.equil.F_slider(i) = mbs_data.Qq(4);
%     MBS_user.equil.T_knee(i) = mbs_data.Qq(1);
%     
%     MBS_user.index_stride_equil =  MBS_user.index_stride_equil + 1;
%     
%     
% end
% 
% % Equilibrium results
% disp('Equilibrium results');
% disp(' ');
% 
% disp(['Angle genou = ', num2str(mbs_data.q(1))]);                                  % Units : translation [m]
% disp(['Position chariot = ', num2str(mbs_data.q(3))]);                                  %         rotation    [rad]
% disp(['Position slider = ', num2str(mbs_data.q(4))]);
% disp(' ');
% 
% disp(['Couple genou = ', num2str(mbs_data.Qq(1))]);                                  % Units : translation [m]
% disp(['Force rapportée Moteur = ', num2str(mbs_data.Qq(3))]);                                  %         rotation    [rad]
% disp(['Force ressort slider = ', num2str(mbs_data.Qq(4))]);
% disp(' ');



% %% 4. direct dynamics [mbs_exe_dirdyn]
% %--------------------------------------------------------------------------
% MBS_user.process = 'dirdyn';
% MBS_user.step = 4;
% 
% 
% % set options
% opt.dirdyn = {'time',0:(MBS_user.T_STRIDE/50000):MBS_user.T_STRIDE,'motion','simulation',...
%     'odemethod','ode45','save2file','yes','framerate',1000,...
%     'renamefile','no','verbose','yes'};
% % other options : 'visualize', 'save2file', 'depinteg', 'dtmax', 'dtinit',
% %                 'reltol', 'abstol', 'clearmbsglobal'                      % Help about options on www.robotran.be
% 
% % launch simulation
% 
% 
% % for i = 1:2
% %     [mbs_dirdyn,mbs_data] = mbs_exe_dirdyn(mbs_data,opt.dirdyn);                % Direct dynamics process (time simulation)
% %    MBS_user.DAMPING_PARAMETER = MBS_user.DAMPING_PARAMETER + 1;
% %    
% %     MBS_user.dirdyn.phi_knee = [MBS_user.dirdyn.phi_knee mbs_dirdyn.q(:,1)];
% %     MBS_user.dirdyn.x_slider(:,i) = mbs_dirdyn.q(:,4);
% %     MBS_user.dirdyn.x_carriage(:,i) = mbs_dirdyn.q(:,3);
% %     
% %     MBS_user.dirdyn.F_carriage(:,i) = mbs_dirdyn.Qq(:,3);
% %     MBS_user.dirdyn.F_slider(:,i) = mbs_dirdyn.Qq(:,4);
% %     MBS_user.dirdyn.T_knee(:,i) = mbs_dirdyn.Qq(:,1);
% 
% %end


%% 5. Inverse dynamics [mbs_exe_invdyn]
%--------------------------------------------------------------------------
MBS_user.process = 'invdyn';
MBS_user.step = 5;

%Joint_id = mbs_get_joint_id(mbs_info,{'joint1' 'joint2' 'joint3'});
%mbs_data = mbs_set_qa(mbs_data,Joint_id);                                   % Set variables [Joint_id] as actuated

opt.invdyn = {'motion','trajectory','time',0:(MBS_user.T_STRIDE/50000):MBS_user.T_STRIDE,'framerate',1000,...
    'save2file','yes','renamefile','no','verbose','yes'};
% other options : 'time', 'visualize', 'clearmbsglobal'                     % Help about options on www.robotran.be

[mbs_invdyn,mbs_data] = mbs_exe_invdyn(mbs_data,opt.invdyn);                % Inverse dynamics process


%% 6. Graphical Results
%--------------------------------------------------------------------------


figure(1)
%plot(linspace(0,100,501),MBS_user.knee.T);
hold on
%plot(linspace(0,100,501),-MBS_user.equil.T_knee);
plot(mbs_invdyn.tsim*100 / MBS_user.T_STRIDE,-mbs_invdyn.qdd(:,2),'g') ;

% % plot(linspace(0,100,501),MBS_user.Fb/20);
% 
% % plot(mbs_dirdyn.tsim*100 / MBS_user.T_STRIDE,-MBS_user.dirdyn.T_knee(:,1),'g');
% % plot(mbs_dirdyn.tsim*100 / MBS_user.T_STRIDE,-MBS_user.dirdyn.T_knee(:,2),'g');
% % plot(mbs_dirdyn.tsim*100 / MBS_user.T_STRIDE,-MBS_user.dirdyn.T_knee(:,3),'g');
% % plot(mbs_dirdyn.tsim*100 / MBS_user.T_STRIDE,-MBS_user.dirdyn.T_knee(:,4),'g');
% 
xlabel('Stride [%]');
ylabel('Couple genou [Nm]');
grid on
