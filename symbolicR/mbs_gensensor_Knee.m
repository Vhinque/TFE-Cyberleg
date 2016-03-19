%
%-------------------------------------------------------------
%
%	ROBOTRAN - Version 6.6 (build : february 22, 2008)
%
%	Copyright 
%	Universite catholique de Louvain 
%	Departement de Mecanique 
%	Unite de Production Mecanique et Machines 
%	2, Place du Levant 
%	1348 Louvain-la-Neuve 
%	http://www.robotran.be// 
%
%	==> Generation Date : Fri Mar 11 08:19:20 2016
%
%	==> Project name : Knee
%	==> using XML input file 
%
%	==> Number of joints : 4
%
%	==> Function : F 6 : Sensors Kinematical Informations (sens) 
%	==> Flops complexity : 87
%
%	==> Generation Time :  0.000 seconds
%	==> Post-Processing :  0.000 seconds
%
%-------------------------------------------------------------
%
function [sens] = gensensor(s,tsim,usrfun,isens)

 sens.P = zeros(3,1);
 sens.R = zeros(3,3);
 sens.V = zeros(3,1);
 sens.OM = zeros(3,1);
 sens.A = zeros(3,1);
 sens.OMP = zeros(3,1);
 sens.J = zeros(6,4);

q = s.q; 
qd = s.qd; 
qdd = s.qdd; 
frc = s.frc; 
trq = s.trq; 

% === begin imp_aux === 

% === end imp_aux === 

% ===== BEGIN task 0 ===== 
 
% Sensor Kinematics 



% = = Block_0_0_0_0_0_1 = = 
 
% Trigonometric Variables  

  C1 = cos(q(1));
  S1 = sin(q(1));

% = = Block_0_0_0_0_0_2 = = 
 
% Trigonometric Variables  

  C2 = cos(q(2));
  S2 = sin(q(2));

% ====== END Task 0 ====== 

% ===== BEGIN task 1 ===== 
 
switch isens

 
% 
case 1, 


% = = Block_1_0_0_1_1_0 = = 
 
% Symbolic Outputs  

    sens.R(1,1) = C1;
    sens.R(1,3) = -S1;
    sens.R(2,2) = (1.0);
    sens.R(3,1) = S1;
    sens.R(3,3) = C1;
    sens.OM(2) = qd(1);
    sens.OMP(2) = qdd(1);
 
% 
case 2, 


% = = Block_1_0_0_2_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = s.dpt(1,1);
    sens.R(1,1) = C2;
    sens.R(1,3) = -S2;
    sens.R(2,2) = (1.0);
    sens.R(3,1) = S2;
    sens.R(3,3) = C2;
    sens.OM(2) = qd(2);
    sens.OMP(2) = qdd(2);
 
% 
case 3, 


% = = Block_1_0_0_3_0_2 = = 
 
% Sensor Kinematics 


    RLcp2_13 = q(3)*S2;
    RLcp2_33 = q(3)*C2;
    POcp2_13 = RLcp2_13+s.dpt(1,1);
    ORcp2_13 = RLcp2_33*qd(2);
    ORcp2_33 = -RLcp2_13*qd(2);
    VIcp2_13 = ORcp2_13+qd(3)*S2;
    VIcp2_33 = ORcp2_33+qd(3)*C2;
    ACcp2_13 = RLcp2_33*qdd(2)+qdd(3)*S2+qd(2)*(ORcp2_33+(2.0)*qd(3)*C2);
    ACcp2_33 = -(RLcp2_13*qdd(2)-qdd(3)*C2+qd(2)*(ORcp2_13+(2.0)*qd(3)*S2));

% = = Block_1_0_0_3_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp2_13;
    sens.P(3) = RLcp2_33;
    sens.R(1,1) = C2;
    sens.R(1,3) = -S2;
    sens.R(2,2) = (1.0);
    sens.R(3,1) = S2;
    sens.R(3,3) = C2;
    sens.V(1) = VIcp2_13;
    sens.V(3) = VIcp2_33;
    sens.OM(2) = qd(2);
    sens.A(1) = ACcp2_13;
    sens.A(3) = ACcp2_33;
    sens.OMP(2) = qdd(2);
 
% 
case 4, 


% = = Block_1_0_0_4_0_2 = = 
 
% Sensor Kinematics 


    RLcp3_13 = q(3)*S2;
    RLcp3_33 = q(3)*C2;
    ORcp3_13 = RLcp3_33*qd(2);
    ORcp3_33 = -RLcp3_13*qd(2);
    RLcp3_14 = q(4)*S2;
    RLcp3_34 = q(4)*C2;
    POcp3_14 = RLcp3_13+RLcp3_14+s.dpt(1,1);
    POcp3_34 = RLcp3_33+RLcp3_34;
    ORcp3_14 = RLcp3_34*qd(2);
    ORcp3_34 = -RLcp3_14*qd(2);
    VIcp3_14 = ORcp3_13+ORcp3_14+qd(3)*S2+qd(4)*S2;
    VIcp3_34 = ORcp3_33+ORcp3_34+qd(3)*C2+qd(4)*C2;
    ACcp3_14 = qdd(2)*(RLcp3_33+RLcp3_34)+qdd(3)*S2+qdd(4)*S2+qd(2)*(ORcp3_33+(2.0)*qd(3)*C2)+qd(2)*(ORcp3_34+(2.0)*qd(4)*C2);
    ACcp3_34 = -(qdd(2)*(RLcp3_13+RLcp3_14)-qdd(3)*C2-qdd(4)*C2+qd(2)*(ORcp3_13+(2.0)*qd(3)*S2)+qd(2)*(ORcp3_14+(2.0)*qd(4)*S2));

% = = Block_1_0_0_4_1_0 = = 
 
% Symbolic Outputs  

    sens.P(1) = POcp3_14;
    sens.P(3) = POcp3_34;
    sens.R(1,1) = C2;
    sens.R(1,3) = -S2;
    sens.R(2,2) = (1.0);
    sens.R(3,1) = S2;
    sens.R(3,3) = C2;
    sens.V(1) = VIcp3_14;
    sens.V(3) = VIcp3_34;
    sens.OM(2) = qd(2);
    sens.A(1) = ACcp3_14;
    sens.A(3) = ACcp3_34;
    sens.OMP(2) = qdd(2);

end


% ====== END Task 1 ====== 

  

