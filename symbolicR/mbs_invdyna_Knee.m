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
%	==> Function : F 2 : Inverse Dynamics : RNEA
%	==> Flops complexity : 52
%
%	==> Generation Time :  0.000 seconds
%	==> Post-Processing :  0.000 seconds
%
%-------------------------------------------------------------
%
function [Qq] = invdyna(s,tsim,usrfun)

 Qq = zeros(4,1);

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

% = = Block_0_1_0_0_0_0 = = 
 
% Forward Kinematics 

  BS93 = -qd(2)*qd(2);
  ALPHA13 = q(3)*qdd(2)+(2.0)*qd(2)*qd(3)+s.g(3)*S2;
  ALPHA33 = qdd(3)-q(3)*qd(2)*qd(2)-s.g(3)*C2;
 
% Backward Dynamics 

  Fs14 = -(s.frc(1,4)-s.m(4)*(ALPHA13+q(4)*qdd(2)+(2.0)*qd(2)*qd(4)));
  Fs34 = -(s.frc(3,4)-s.m(4)*(qdd(4)+ALPHA33+q(4)*BS93));
  Fs13 = -(s.frc(1,3)-s.m(3)*(ALPHA13+qdd(2)*s.l(3,3)));
  Fq33 = -(s.frc(3,3)-Fs34-s.m(3)*(ALPHA33+BS93*s.l(3,3)));
  Cq22 = -(s.trq(2,3)+s.trq(2,4)-q(3)*(Fs13+Fs14)-q(4)*Fs14-Fs13*s.l(3,3));
  Cq21 = -(s.trq(2,1)-qdd(1)*s.In(5,1));

% = = Block_0_2_0_0_0_0 = = 
 
% Symbolic Outputs  

  Qq(1) = Cq21;
  Qq(2) = Cq22;
  Qq(3) = Fq33;
  Qq(4) = Fs34;

% ====== END Task 0 ====== 

  

