function Flink = user_LinkForces(Z,Zd,mbs_data,tsim,ilnk)
% --------------------------
% UCL-CEREM-MBS
%
% @version MBsysLab_m 1.7.a
%
% Creation : 2005
% Last update : 30/09/2008
% -------------------------
%
% Flink = user_LinkForces(Z,Zd,mbs_data,tsim,ilnk)
%
% Z : position of link-body 2 with respect to link-body 1
% Zd : velocity of link-body 2 with respect to link-body 1
% NB :  Z and Zd are automatically computed in the symbolic file
%       associated with the links that the user has introduced in its MBS
% mbs_data : multibody data structure
% tsim : current time
% ilnk : link index (can be obtained via the 'mbs_get_link_id' function)
%
% Flink : force applied to link-body 1 from link-body 2 (link "ilnk")
%   NB :
%     - For a spring/damper system, the Flink force has the SAME sign as Z, Zd
%       (contrary to joint forces) and thus : Flink = + ... Z + ... Zd
%     - The reaction "-Flink" is automatically taken into accounbt by MBsyslab
%
% this function may use a global structure called MBS_user

global MBS_user MBS_info

MBS_user.index_stride_dirdyn  = round(tsim * 500 / MBS_user.T_STRIDE)+1;

Flink = 0;
q = mbs_data.q;



%/*-- Begin of user code --*/
L1 = mbs_get_link_id(MBS_info,'WA_link');
%
switch(ilnk)
    case L1
        
        id = mbs_get_joint_id(MBS_info,'Knee_joint');
        
        if (MBS_user.step == 3)
            
            if (MBS_user.index_stride_equil > 158)
                WA_active = 0;
            else
                WA_active = 1;
            end
            
        elseif (MBS_user.step == 4)
            if (MBS_user.index_stride_dirdyn > 158)
                WA_active = 0;
            else
                WA_active = 1;
            end
            
        elseif (MBS_user.step == 5)
            if (MBS_user.index_stride_dirdyn > 158)
                WA_active = 0;
            else
                WA_active = 1;
            end
            
        else
            WA_active = 0;
        end
        
        
        
        K = 200000; %mbs_data.user_model.WA_Spring.K;
        Z0 = 0.15134; %mbs_data.user_model.WA_Spring.L0;
        
        if q(1) < 0.17453
            Flink = 0;
        else
            Flink = WA_active * K*(Z-Z0);
        end
        
        
    case L2
        % instructions for case 2, if any
        %
    case L3
        % instructions for case 3, if any
end

%/*-- End of user code --*/

return
