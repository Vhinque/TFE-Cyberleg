function [Qq] = user_JointForces(mbs_data,tsim);
% --------------------------
% UCL-CEREM-MBS
%
% @version MBsysLab_m 1.7.a
%
% Creation : 2006
% Last update : 30/09/2008
% -------------------------
%
%[Qq] = user_JointForces(mbs_data.tsim);
%
% mbs_data : multibody data structure
% tsim : current time
%
% Qq : joint generalized force/torque (for all joints)
% Qq(i) : joint force/torque in joint (i) along its joint axis
%
% this function may use a global structure called MBS_user

global MBS_user MBS_info

MBS_user.index_stride_dirdyn  = round(tsim * 500 / MBS_user.T_STRIDE)+1;
load('desired_torque.mat')

Qq = mbs_data.Qq;

%/*-- Begin of user code --*/

% Knee spring force
id = mbs_get_joint_id(MBS_info,'EX_BL_springs');


x = mbs_data.q(id);
xp = mbs_data.qd(id);

K_Ex = 2 * 89000; % [N/m]
K_Bl = 2 * 10700; % [N/m]

B_Ex = 1780;% + 10*MBS_user.DAMPING_PARAMETER; %[Ns/m]
B_Bl = 200;% + 10*MBS_user.DAMPING_PARAMETER; %[Ns/m]

L0_Ex = 0.032;
L0_Bl = 0.064;

L_total = 0.094;


if (x > -(L_total - L0_Bl)) % -0.03
    Qq(id) = - K_Ex * (x + L0_Ex) - B_Ex * xp; %0.032    
elseif (x < -L0_Ex) % -0.032
    Qq(id) = - K_Bl * (x + (L_total - L0_Bl)) - B_Bl * xp ;
else
    Qq(id) = - K_Bl * (x + (L_total - L0_Bl)) - K_Ex * (x + L0_Ex) - (B_Ex + B_Bl) * xp;
end


% 
% % Knee spring force
% 
% id = mbs_get_joint_id(MBS_info,'Carriage_position');
% 
% if (MBS_user.step == 3)
%     
% %    Qq(id) =  mbs_invdyn.Qq(MBS_user.index_stride_equil,3);
% %     
% % elseif (MBS_user.step == 4)
% %     
% %     Qq(id) =  mbs_invdyn.Qq(MBS_user.index_stride_dirdyn,3);
% %     
% % else
% %     
% %     Qq(id) =  0;
% %     
%  end


%/*-- End of user code --*/

return
