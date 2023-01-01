% D_ExD cal
clear all
%clc

AUW = 3000;
Bmax = 2;
Pdia = 13;
Ppicth = 6;
Nm =2; %number of motors
Vc = [12 14 16 18];
Bpc = 0;
Bcruise = Bmax-Bpc;

airfoil
Batt = importdata('Batteries_hovervalidate.txt'); %import batteries data from text file
Motor = importdata('MotorSpec_hovervalidate.txt'); %import Motors data from text file
Cellcheck = Batt.data(1);
Cap = (Batt.data (2))*0.8; %battery's Capacity  (mAh) *0.8 mean that max dischart = 80% of total capacity
Mi0 = Motor.data(5); % motor's No-load current (A)
MR = Motor.data(4)/1000; % motor's Internal Resistance (m-ohm to ohm)
v = Cellcheck*3.7 ; %Voltage from battery
kv = Motor.data(1);% Motor's kv

% call propeller data
filename = sprintf('%dx%d.txt',Pdia,Ppicth); %create propeller's file name
Prop = importdata(filename); %call propeller's file
Prop_databyrpm = size(Prop.data,1)/25; %calculate number of data group devide by RPM
RPM = zeros(1,Prop_databyrpm); %Preallocation array make a loop faster
Tv0 = zeros(1,Prop_databyrpm);
Pv0 = zeros(1,Prop_databyrpm);
Tvc = zeros(1,Prop_databyrpm);
Pvc = zeros(1,Prop_databyrpm);
for k = 1:4
V = Vc(k)*2.23694; %velocity at hover and cruise (m/s to mph)
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

c = 0.1; %chord initial
thickness = 0; %wing's thickness initial
Exlift_vc = 0; %extra lift initial
Exdrag_v0 = 0; %extra drag initial
Exdrag_vc = 0; %extra drag initial
S =0;
S_old=100;
while abs((S-S_old)/S)*100 >= 0.1
    S_old = S;
    [S,c,Drag,CL,CD,thickness] = wingdesign(AUW,Vc(k),Bcruise,Exlift_vc,Exdrag_vc); %S=wing's area (m^2), c = chord mean(m),Drag (N),CL = 3D lift coefficient,CD = 3D Drag coefficient
    [Exlift_vc,Exdrag_vc,Bpc] = Momentumprop(Drag,Pdia,Vc(k),c,CL,CD);
    Bcruise = Bmax-Bpc;
end

ar = [5 10 15 20];
eff = [0.935 0.91 0.885 0.86];
Oswald = fit(ar',eff','cubicinterp'); %Oswald's efficiency of rectangular wing that depend on AR
AR = Bmax/c;
e = Oswald(AR) ;
CD = Cd0 + ((CL^2)/(pi*e*AR)); %3D Drag coefficient

Drag_perside = ((0.5*1.225*(Vc(k)^2)*S*CD)/Nm)+Exdrag_vc; %Drag (N)
new_D = Drag_perside*2;
old_D = 0;
while abs((new_D-old_D)/new_D*100)>=10
    old_D = Drag_perside*2;
    Tcruise = Drag_perside; % Treq(N)
    [~,Exdrag_v0,~] = Momentumprop(Tcruise,Pdia,Vc(k),c,Cl,CD);
    Drag_wing = ((0.5*1.225*(Vc(k)^2)*S*CD)/Nm);  %Drag (N) per side
    Drag_perside = Drag_wing+Exdrag_vc;
    new_D = Drag_perside*2;
end
S_wing(k) = S;
Drag_Perside(k) = Drag_perside;
Exdrag(k) =Exdrag_vc;
TTT = Drag_perside*0.224809;
RPM_c = T2RPM_vc(TTT);
P_c = (RPM2P_vc(RPM_c))*745.7;
Eff_motor = inline('(1-((Mi0*MR/1000)/(v-(RPM/kv))))*(RPM/(v*kv))'); %motor's eff function
Power_cruise(k) = (P_c/(Eff_motor(MR,Mi0,RPM_c,kv,v)));%each motor

end

VarNames_c = {'Cruise_Velocity', 'Wing_area', 'Drag_total', 'Drag_extra','Power'};
Cruise = table(Vc',S_wing',Drag_Perside',Exdrag',Power_cruise', 'VariableNames',VarNames_c)








