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
%	==> Function : F 1 : Direct Dynamics (Semi-Explicit formulation) : RNEA
%	==> Flops complexity : 53
%
%	==> Generation Time :  0.000 seconds
%	==> Post-Processing :  0.000 seconds
%
%-------------------------------------------------------------
%
function [M,c] = dirdyna(s,tsim,usrfun)

 M = zeros(4,4);
 c = zeros(4,1);

q = s.q; 
qd = s.qd; 
qdd = s.qdd; 
frc = s.frc; 
trq = s.trq; 

% === begin imp_aux === 

% === end imp_aux === 

% ===== BEGIN task 0 ===== 

% = = Block_0_0_0_0_0_2 = = 
 
% Trigonometric Variables  

  C2 = cos(q(2));
  S2 = sin(q(2));

% = = Block_0_1_0_0_0_2 = = 
 
% Forward Kinematics 

  BS93 = -qd(2)*qd(2);
  AlF13 = (2.0)*qd(2)*qd(3)+s.g(3)*S2;
  AlF33 = -(q(3)*qd(2)*qd(2)+s.g(3)*C2);

% = = Block_0_2_0_1_0_2 = = 
 
% Backward Dynamics 

  FA14 = -(s.frc(1,4)-s.m(4)*(AlF13+(2.0)*qd(2)*qd(4)));
  FA34 = -(s.frc(3,4)-s.m(4)*(AlF33+q(4)*BS93));
  FB14_2 = s.m(4)*(q(3)+q(4));
  FA13 = -(s.frc(1,3)-s.m(3)*AlF13);
  FF33 = -(s.frc(3,3)-FA34-s.m(3)*(AlF33+BS93*s.l(3,3)));
  FB13_2 = s.m(3)*(q(3)+s.l(3,3));
  FM33_3 = s.m(3)+s.m(4);
  CF2_23 = -(s.trq(2,3)+s.trq(2,4)-q(3)*(FA13+FA14)-q(4)*FA14-FA13*s.l(3,3));
  CM22_23 = q(3)*(FB13_2+FB14_2)+q(4)*FB14_2+FB13_2*s.l(3,3);

% = = Block_0_3_0_0_0_0 = = 
 
% Symbolic Outputs  

  M(1,1) = s.In(5,1);
  M(2,2) = CM22_23;
  M(3,3) = FM33_3;
  M(3,4) = s.m(4);
  M(4,3) = s.m(4);
  M(4,4) = s.m(4);
  c(1) = -s.trq(2,1);
  c(2) = CF2_23;
  c(3) = FF33;
  c(4) = FA34;

% ====== END Task 0 ====== 

  

