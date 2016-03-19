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
%	==> Generation Date : Fri Mar 11 08:19:19 2016
%
%	==> Project name : Knee
%	==> using XML input file 
%
%	==> Number of joints : 4
%
%	==> Function : F18 : Constraints Quadratic Velocity Terms (Jdqd)
%	==> Flops complexity : 70
%
%	==> Generation Time :  0.000 seconds
%	==> Post-Processing :  0.000 seconds
%
%-------------------------------------------------------------
%
function [Jdqd] = cons_jdqd(s,tsim,usrfun)

 Jdqd = zeros(1,1);

q = s.q; 
qd = s.qd; 
qdd = s.qdd; 
frc = s.frc; 
trq = s.trq; 

% === begin imp_aux === 

% === end imp_aux === 

% ===== BEGIN task 0 ===== 

% = = Block_0_0_0_0_0_1 = = 
 
% Trigonometric Variables  

  C1 = cos(q(1));
  S1 = sin(q(1));

% = = Block_0_0_0_0_0_2 = = 
 
% Trigonometric Variables  

  C2 = cos(q(2));
  S2 = sin(q(2));

% = = Block_0_1_0_0_0_1 = = 
 
% Constraints and Constraints Jacobian 

%
  RL_0_15 = s.dpt(1,3)*C1+s.dpt(3,3)*S1;
  RL_0_35 = -(s.dpt(1,3)*S1-s.dpt(3,3)*C1);

% = = Block_0_1_0_0_0_2 = = 
 
% Constraints and Constraints Jacobian 

%
  RL_1_13 = q(3)*S2;
  RL_1_33 = q(3)*C2;
  RL_1_14 = q(4)*S2;
  RL_1_34 = q(4)*C2;

% = = Block_0_2_0_0_0_0 = = 
 
% Constraints Quadratic Terms 

%
  OR_0_15 = RL_0_35*qd(1);
  OR_0_35 = -RL_0_15*qd(1);
%
  OR_1_13 = RL_1_33*qd(2);
  OR_1_33 = -RL_1_13*qd(2);
  OR_1_14 = RL_1_34*qd(2);
  OR_1_34 = -RL_1_14*qd(2);
  VI_1_14 = OR_1_13+OR_1_14+qd(3)*S2+qd(4)*S2;
  VI_1_34 = OR_1_33+OR_1_34+qd(3)*C2+qd(4)*C2;

% = = Block_0_2_0_0_0_1 = = 
 
% Constraints Quadratic Terms 

%
  jdqd1 = (OR_0_15-VI_1_14)*(OR_0_15-VI_1_14)+(OR_0_35-VI_1_34)*(OR_0_35-VI_1_34)-(OR_0_15*qd(1)-qd(2)*(OR_1_13+OR_1_14+(2.0)*qd(3)*S2+(2.0)*qd(4)*S2))*(...
 RL_0_35-RL_1_33-RL_1_34)+(OR_0_35*qd(1)-qd(2)*(OR_1_33+OR_1_34+(2.0)*qd(3)*C2+(2.0)*qd(4)*C2))*(RL_0_15-RL_1_13-RL_1_14-s.dpt(1,1));

% = = Block_0_3_0_0_0_0 = = 
 
% Symbolic Outputs  

  Jdqd(1) = jdqd1;

% ====== END Task 0 ====== 

  

