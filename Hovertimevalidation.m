%Hover Validation
clear
clc

%fprintf('Power req validation \n');% cal at hover condition
fprintf('Power req validation w/ factor\n');% cal at hover condition
AUW = 4.112;%kg
Pdia = 13;%inch
Ppicth = 8;
S = 0.589;
AR = 2.95;
c = sqrt(S/AR);%m
b = S/c;
Cl = 0.1;%mh45
Cd0 = 0.02;%mh45
Nm =2; %number of motors
%S_factor = 1.4;
V0 = 0;
Vc = 15;

% validate test data
% Thrust_total_test = 17.85;% N

Batt = importdata('Batteries_hovervalidate.txt'); %import batteries data from text file
Motor = importdata('MotorSpec_hovervalidate.txt'); %import Motors data from text file
Cellcheck = Batt.data(1);
Cap = (Batt.data (2))*0.8; %battery's Capacity  (mAh) *0.8 mean that max dischart = 80% of total capacity
Mi0 = Motor.data(5); % motor's No-load current (A)
MR = Motor.data(4)/1000; % motor's Internal Resistance (m-ohm to ohm)
v = Cellcheck*3.7 ; %Voltage from battery
kv = Motor.data(1);% Motor's kv
eff_factor = 1.12582688593886;

% call propeller data
filename = sprintf('%dx%d.txt',Pdia,Ppicth); %create propeller's file name
Prop = importdata(filename); %call propeller's file
Prop_databyrpm = size(Prop.data,1)/25; %calculate number of data group devide by RPM
RPM = zeros(1,Prop_databyrpm); %Preallocation array make a loop faster
Tv0 = zeros(1,Prop_databyrpm);
Pv0 = zeros(1,Prop_databyrpm);
Tvc = zeros(1,Prop_databyrpm);
Pvc = zeros(1,Prop_databyrpm);
V = Vc*2.23694; %velocity at hover and cruise (m/s to mph)
for i = 1:Prop_databyrpm %each RPM group

    RPM(i) = Prop.data(((i-1)*25)+1,1); %collect each rpm in this propeller's data
    for ii = 1:24
        X.v(ii) = Prop.data(((i-1)*25)+1+ii,1); %velocity (mph)
        X.T(ii) = Prop.data(((i-1)*25)+1+ii,8); %Thrust (lbf)
        X.P(ii) = Prop.data(((i-1)*25)+1+ii,6); %Power output(Hp)
    end
funTeachRPM = fit(X.v',X.T','linearinterp'); %fit curve velocity vs Thrust for each rpm
    funPeachRPM = fit(X.v',X.P','linearinterp');%fit curve velocity vs power for each rpm
    Tv0(i) = X.T(1); % thrust at v=0 (hover) for each rpm
    Tvc(i) = funTeachRPM(V);% thrust at v=vc (cruise) for each rpm
    Pv0(i) = X.P(1);% Power at v=0 (hover) for each rpm
    Pvc(i) = funPeachRPM(V);% Power at v=0 (cruise) for each rpm
end
T2RPM_v0 = fit(Tv0',RPM','linearinterp');
RPM2P_v0 = fit(RPM',Pv0','linearinterp');
T2RPM_vc = fit(Tvc',RPM','linearinterp');
RPM2P_vc = fit(RPM',Pvc','linearinterp');

% Hover part
T = (AUW*9.81/Nm);%*S_factor; %N
Exdrag_v0 = 0; %extra drag initial
Treq = T+Exdrag_v0;
new_T = Treq;
old_T = 0;
while abs((new_T-old_T)/new_T*100)>=10
    old_T = Treq;
    Thover = Treq; % Treq(N)
    [~,Exdrag_v0,~] = Momentumprop(Thover,Pdia,V0,c,Cl,Cd0);
    %Treq = T + Exdrag_v0;
    Treq = T + (Exdrag_v0*1.28470118686227);%correction factor
    new_T = Treq;
end
TT = Treq*0.224809;
RPM_h = T2RPM_v0(TT);
P_h = (RPM2P_v0(RPM_h))*745.7;
Eff_motor = inline('(1-((Mi0*MR/1000)/(v-(RPM/kv))))*(RPM/(v*kv))'); %motor's eff function
%Power_hover = (P_h/(Eff_motor(MR,Mi0,RPM_h,kv,v))); %each motor
Power_hover = (P_h/(Eff_motor(MR,Mi0,RPM_h,kv,v)*eff_factor)); %each motor w/ factor
Test_hover = 716.0088;% hover power from test

%Cruise part
CL = (AUW*9.81)/(0.5*1.225*Vc*Vc*S);
ar = [5 10 15 20];
eff = [0.935 0.91 0.885 0.86];
Oswald = fit(ar',eff','cubicinterp'); %Oswald's efficiency of rectangular wing that depend on AR
e = Oswald(AR) ;
CD = Cd0 + ((CL^2)/(pi*e*AR)); %3D Drag coefficient
Exdrag_vc = 0;
Drag = ((0.5*1.225*(Vc^2)*S*CD)/Nm)+Exdrag_vc; %Drag (N)
new_D = Drag;
old_D = 0;
while abs((new_D-old_D)/new_D*100)>=10
    old_D = Drag;
    Tcruise = Drag; % Treq(N)
    [~,Exdrag_vc,~] = Momentumprop(Tcruise,Pdia,Vc,c,Cl,CD);
    %Drag = ((0.5*1.225*(Vc^2)*S*CD)/Nm)+Exdrag_vc;  %Drag (N)
    Drag = ((0.5*1.225*(Vc^2)*S*CD)/Nm)+(Exdrag_vc*1.28470118686227);  %Drag (N)w/ correction factor
    new_D = Drag;
end
TTT = Drag*0.224809;
RPM_c = T2RPM_vc(TTT);
P_c = (RPM2P_vc(RPM_c))*745.7;
Eff_motor = inline('(1-((Mi0*MR/1000)/(v-(RPM/kv))))*(RPM/(v*kv))'); %motor's eff function
%Power_cruise = (P_c/(Eff_motor(MR,Mi0,RPM_c,kv,v)));%each motor
Power_cruise = (P_c/(Eff_motor(MR,Mi0,RPM_c,kv,v)*eff_factor));%each motor w/ factor
Test_cruise = 171.646875;%cruise power from test

%comparison
 diff_h = (Power_hover-Test_hover)/Test_hover*100;
 diff_c = (Power_cruise-Test_cruise)/Test_cruise*100;
 VarNames_h = {'Hover_Power_ref', 'Hover_power_cal', 'Difference_percentage'};
 Hover = table(Test_hover,Power_hover,diff_h, 'VariableNames',VarNames_h)
 VarNames_c = {'Cruise_Power_ref', 'Cruise_power_cal', 'Difference_percentage'};
 Cruise = table(Test_cruise,Power_cruise,diff_c, 'VariableNames',VarNames_c)