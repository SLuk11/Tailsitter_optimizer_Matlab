function[f_TRpm_v0,f_RpmP_v0,f_TRpm_vc,f_RpmP_vc,Max_Tv0,Max_Tvc,Prop_MaxRPM] = fitprop(Pdia,Ppicth,Vc)
filename = sprintf('%dx%d.txt',Pdia,Ppicth); %create propeller's file name
Prop_MaxRPM = 190000/Pdia;
Prop = importdata(filename); %call propeller's file
Prop_databyrpm = size(Prop.data,1)/25; %calculate number of data group devide by RPM
RPM = zeros(1,Prop_databyrpm); %Preallocation array make a loop faster
Tv0 = zeros(1,Prop_databyrpm);
Tvc = zeros(1,Prop_databyrpm);
Pv0 = zeros(1,Prop_databyrpm);
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
Rpm2T_v0 = fit(RPM',Tv0','linearinterp');
Rpm2T_vc = fit(RPM',Tvc','linearinterp');
f_TRpm_v0 = fit(Tv0',RPM','linearinterp');
f_RpmP_v0 = fit(RPM',Pv0','linearinterp');
f_TRpm_vc = fit(Tvc',RPM','linearinterp');
f_RpmP_vc = fit(RPM',Pvc','linearinterp');
Max_Tv0 = Rpm2T_v0(Prop_MaxRPM); %Max thrust at v=0 (lbf)
Max_Tvc = Rpm2T_vc(Prop_MaxRPM); %Max thrust at v=Vc (lbf)

