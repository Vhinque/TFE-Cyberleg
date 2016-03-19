function [q,qd,qdd] = user_DrivenJoints(mbs_data,tsim)
% --------------------------
% UCL-CEREM-MBS
%
% @version MBsysLab_m 1.7.a
%
% Creation : 2005
% Last update : 30/09/2008
% -------------------------
%
%[q,qd,qdd] = user_DrivenJoints(mbs_data,tsim)
%
% mbs_data : multibody data structure
% tsim : current time
%
% q, qd, qdd : updated column vectors of generalized coordinates
%
%
% mbs_data.q : generalized coordinates [column vector]
% mbs_data.qd : generalized velocities [column vector]
% mbs_data.qdd : generalized accelerations [column vector]
% mbs_data.nqc : number of driven variables
% mbs_data.qc : indices of driven variables [column vector]

global MBS_user MBS_info

q   = mbs_data.q;
qd  = mbs_data.qd;
qdd = mbs_data.qdd;

MBS_user.index_stride_dirdyn  = round(tsim * 500 / MBS_user.T_STRIDE)+1;
load('carriage_position.mat');
dcarriage_position = [0;diff(carriage_position)];
ddcarriage_position = [0;diff(dcarriage_position)];



%/*-- Begin of user code --*/

% Fix the inclination angle to 5°
id = mbs_get_joint_id(MBS_info,'Inclination_angle');

angle = -0.087;

q(id) = angle;
qd(id) = 0;
qdd(id) = 0;

% Drive the knee angle
id = mbs_get_joint_id(MBS_info,'Carriage_position');

if (MBS_user.step == 3)
    
%     q(id) = carriage_position(MBS_user.index_stride_equil) * 2 * pi / 360;
%     qd(id) = 0;
%     qdd(id) = 0;
    
elseif (MBS_user.step == 4)
    
%     q(id) = carriage_position(MBS_user.index_stride_dirdyn) * 2 * pi / 360;
%     qd(id) = 0;
%     qdd(id) = 0;
    
elseif (MBS_user.step == 5)
    
    q(id) = -0.129 + MBS_user.index_stride_dirdyn * 0.00008;%-0.129 + carriage_position(MBS_user.index_stride_dirdyn);
    qd(id) = dcarriage_position(MBS_user.index_stride_dirdyn);
    qdd(id) = ddcarriage_position(MBS_user.index_stride_dirdyn);
    
    
else
    
    q(id) = 0;
    qd(id) = 0;
    qdd(id) = 0;
    
end

% Drive the knee angle
id = mbs_get_joint_id(MBS_info,'Knee_joint');

if (MBS_user.step == 3)
    
    q(id) = MBS_user.knee.theta(MBS_user.index_stride_equil) * 2 * pi / 360;
    qd(id) = 0;
    qdd(id) = 0;
    
elseif (MBS_user.step == 4)
    
    q(id) = MBS_user.knee.theta(MBS_user.index_stride_dirdyn) * 2 * pi / 360;
    qd(id) = MBS_user.knee.dtheta(MBS_user.index_stride_dirdyn) * 2 * pi / 360;
    qdd(id) = 0;
    
elseif (MBS_user.step == 5)
    
    q(id) = MBS_user.knee.theta(MBS_user.index_stride_dirdyn) * 2 * pi / 360;
    qd(id) = MBS_user.knee.dtheta(MBS_user.index_stride_dirdyn) * 2 * pi / 360;
    qdd(id) = MBS_user.knee.ddtheta(MBS_user.index_stride_dirdyn) * 2 * pi / 360;;
    
else
    
    q(id) = 0;
    qd(id) = 0;
    qdd(id) = 0;
    
end

%/*-- End of user code --*/

return
