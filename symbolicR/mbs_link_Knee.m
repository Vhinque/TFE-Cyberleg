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
%	==> Function : F 7 : Point to point Link Forces (frc,trq,Flnk) 
%	==> Flops complexity : 28
%
%	==> Generation Time :  0.000 seconds
%	==> Post-Processing :  0.000 seconds
%
%-------------------------------------------------------------
%
function [frc,trq,Flnk,Z,Zd] = link(s,tsim,usrfun)

 frc = zeros(3,4);
 trq = zeros(3,4);
 Flnk = zeros(1,1);
 Z = zeros(1,1);
 Zd = zeros(1,1);

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

% = = Block_0_1_0_0_1_1 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  RLlnk0_17 = s.dpt(1,4)*C1;
  RLlnk0_37 = -s.dpt(1,4)*S1;

% = = Block_0_1_0_1_1_2 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk11 = -(RLlnk0_17-s.dpt(1,2));
  Plnk31 = -(RLlnk0_37-s.dpt(3,2));
  Z1 = sqrt(Plnk11*Plnk11+Plnk31*Plnk31);
  e11 = Plnk11/Z1;
  e31 = Plnk31/Z1;
  Zd1 = qd(1)*(RLlnk0_17*e31-RLlnk0_37*e11);
 
% Link Force Computation 

  Flink1 = usrfun.flink(Z1,Zd1,s,tsim,1);

% = = Block_0_1_0_2_2_1 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk11 = Flink1*(e11*C1-e31*S1);
  fPlnk31 = Flink1*(e11*S1+e31*C1);
  frc(1,1) = s.frc(1,1)+fPlnk11;
  frc(3,1) = s.frc(3,1)+fPlnk31;
  trq(2,1) = s.trq(2,1)-fPlnk31*s.dpt(1,4);

% = = Block_0_2_0_0_0_0 = = 
 
% Symbolic Outputs  

  frc(2,1) = s.frc(2,1);
  frc(1,3) = s.frc(1,3);
  frc(2,3) = s.frc(2,3);
  frc(3,3) = s.frc(3,3);
  frc(1,4) = s.frc(1,4);
  frc(2,4) = s.frc(2,4);
  frc(3,4) = s.frc(3,4);
  trq(1,1) = s.trq(1,1);
  trq(3,1) = s.trq(3,1);
  trq(1,3) = s.trq(1,3);
  trq(2,3) = s.trq(2,3);
  trq(3,3) = s.trq(3,3);
  trq(1,4) = s.trq(1,4);
  trq(2,4) = s.trq(2,4);
  trq(3,4) = s.trq(3,4);
  Flnk(1) = Flink1;
  Z(1) = Z1;
  Zd(1) = Zd1;

% ====== END Task 0 ====== 

  

