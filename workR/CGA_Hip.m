% CGA data Hip Natural Cadence, , Winter
%stride|Mean Angle|St. Dev|Moment|St. Dev|

% Hip actuators on thigh link!

global MBS_user;


MBS_user.CGA_Hip_Data=...
[0	19.33	5.64	0.249	0.222;	
2	18.92	5.79	0.6     0.317;	
4	18.45	5.77	0.556	0.334;	
6	17.94	5.64	0.416	0.402;	
8	17.3	5.56	0.359	0.497;	
10	16.4	5.63	0.305	0.555;	
12	15.18	5.81	0.245	0.541;	
14	13.67	5.94	0.159	0.491;	
16	11.97	5.9     0.084	0.438;	
18	10.21	5.73	0       0.364;	
20	8.48	5.57   -0.064	0.31;	
22	6.74	5.47   -0.092	0.291;	
24	4.94	5.38   -0.098	0.281;	
26	3.13	5.36   -0.092	0.266;	
28	1.42	5.51   -0.085	0.267;	
30	-0.13	5.75	-0.088	0.264;	
32	-1.54	5.96	-0.1	0.272;	
34	-2.87	6.14	-0.13	0.272;	
36	-4.12	6.34	-0.168	0.287;	
38	-5.3	6.58	-0.199	0.312;	
40	-6.4	6.86	-0.231	0.349;	
42	-7.43	7.14	-0.269	0.376;	
44	-8.39	7.4     -0.312	0.384;	
46	-9.27	7.68	-0.364	0.385;	
48	-10.02	7.97	-0.401	0.394;	
50	-10.61	8.25	-0.404	0.413;	
52	-10.95	8.51	-0.356	0.389;	
54	-10.91	8.71	-0.262	0.335;	
56	-10.31	8.81	-0.251	0.265;	
58	-9      8.72	-0.31	0.168;	
60	-6.95	8.39	-0.344	0.137;	
62	-4.25	7.84	-0.295	0.125;	
64	-1.05	7.17	-0.228	0.108;	
66	2.42	6.47	-0.169	0.122;	
68	5.93	5.8     -0.126	0.082;	
70	9.22	5.22	-0.089	0.066;	
72	12.11	4.75	-0.069	0.087;	
74	14.55	4.44	-0.057	0.063;	
76	16.53	4.35	-0.044	0.051;	
78	18.13	4.43	-0.026	0.053;	
80	19.45	4.59	-0.009	0.05;	
82	20.54	4.76	0.008	0.047;	
84	21.38	4.84	0.029	0.053;	
86	21.84	4.83	0.06	0.069;	
88	21.87	4.75	0.106	0.085;	
90	21.5	4.68	0.17	0.105;	
92	20.84	4.72	0.242	0.122;	
94	20.09	4.84	0.296	0.139;	
96	19.5	5.02	0.301	0.144;	
98	19.18	5.23	0.237	0.139;	
100	19.01	5.43	0.118	0.123];

MBS_user.CGA_Hip_Data(:,4)=-MBS_user.CGA_Hip_Data(:,4); %conversion biom -> mech >>>
%Cbiom=Cmech, see convention WINTER

MBS_user.CGA_Hip_Data(:,2)=MBS_user.CGA_Hip_Data(:,2);

% A=[ CGA_Hip_Data(:,2)+1.65*CGA_Hip_Data(:,3);   CGA_Hip_Data(:,2)+1.65*CGA_Hip_Data(:,3);  CGA_Hip_Data(:,2)-1.65*CGA_Hip_Data(:,3); CGA_Hip_Data(:,2)-1.65*CGA_Hip_Data(:,3) ]';
% 
% T=[ CGA_Hip_Data(:,4)+1.65*CGA_Hip_Data(:,5); CGA_Hip_Data(:,4)-1.65*CGA_Hip_Data(:,5); CGA_Hip_Data(:,4)+1.65*CGA_Hip_Data(:,5); CGA_Hip_Data(:,4)-1.65*CGA_Hip_Data(:,5)]';

MBS_user.HipAngle=MBS_user.CGA_Hip_Data(:,2)';
MBS_user.TotalHipMOF=MBS_user.CGA_Hip_Data(:,4)';

% size_A=size(A,2);
% 
% count_pos=0;
% count_neg=0;
% 
% for i=1:size_A
%     if (T(i)>=0)
%         count_pos=count_pos+1;
%         A_required_pos(count_pos)=A(i);
%         T_required_pos(count_pos)=T(i);
%     else
%         count_neg=count_neg+1;
%         A_required_neg(count_neg)=A(i);
%         T_required_neg(count_neg)=T(i);     
%     end;
% end;

MBS_user.HipAngleRiener=MBS_user.HipAngle;%sign conform with definition by Riener (passive elastic joint moments)

% figure;
% 
% plot(A_required_pos, T_required_pos, '*');
% 
% figure;
% 
% plot(A_required_neg, T_required_neg,'*');


% figure;
% 
% hold on;
% 
% %---90% range for x : [-1.65s, 1.65s]-----%
% 
% plot(CGA_Hip_Data(:,2),CGA_Hip_Data(:,4),'r')
% plot(CGA_Hip_Data(:,2)+1.65*CGA_Hip_Data(:,3),CGA_Hip_Data(:,4)+1.65*CGA_Hip_Data(:,5),'g')
% plot(CGA_Hip_Data(:,2)+1.65*CGA_Hip_Data(:,3),CGA_Hip_Data(:,4)-1.65*CGA_Hip_Data(:,5),'b')
% plot(CGA_Hip_Data(:,2)-1.65*CGA_Hip_Data(:,3),CGA_Hip_Data(:,4)+1.65*CGA_Hip_Data(:,5),'c')
% plot(CGA_Hip_Data(:,2)-1.65*CGA_Hip_Data(:,3),CGA_Hip_Data(:,4)-1.65*CGA_Hip_Data(:,5),'m')
% 
% legend('Mean vs Mean','Max vs Max','Min vs Max','Max vs Min','Min vs Min')