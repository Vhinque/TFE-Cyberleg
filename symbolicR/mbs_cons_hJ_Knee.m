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
%	==> Function : F 8 : Constraints Vector (h) and Jacobian Matrix (Jac) 
%	==> Flops complexity : 43
%
%	==> Generation Time :  0.000 seconds
%	==> Post-Processing :  0.000 seconds
%
%-------------------------------------------------------------
%
function [h,Jac] = cons_hJ(s,tsim,usrfun)

 h = zeros(1,1);
 Jac = zeros(1,4);

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

% = = Block_0_1_0_0_1_0 = = 
 
% Constraints and Constraints Jacobian 

%
  Plp11 = RL_0_15-RL_1_13-RL_1_14-s.dpt(1,1);
  Plp31 = RL_0_35-RL_1_33-RL_1_34;
  h_1 = (0.50)*(Plp11*Plp11+Plp31*Plp31-s.lrod(1)*s.lrod(1));
%
  Jacu_1_1 = Plp11*RL_0_35-Plp31*RL_0_15;
  Jacu_1_2 = -(Plp11*(RL_1_33+RL_1_34)-Plp31*(RL_1_13+RL_1_14));
  Jacu_1_3 = -(Plp11*S2+Plp31*C2);
  Jac_1_4 = -(Plp11*S2+Plp31*C2);

% = = Block_0_3_0_0_0_0 = = 
 
% Symbolic Outputs  

  h(1) = h_1;
  Jac(1,1) = Jacu_1_1;
  Jac(1,2) = Jacu_1_2;
  Jac(1,3) = Jacu_1_3;
  Jac(1,4) = Jac_1_4;

% ====== END Task 0 ====== 

  

