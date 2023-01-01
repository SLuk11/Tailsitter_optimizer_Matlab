clear
%clc

fprintf('Motor eff validation\n');% cal at hover condition
RPM = input('RPM = '); 
Tq = input('Torque(N.m) = ');
% Q=0.028 RPM = 3596
% Q=0.108 RPM=6300
% Q=0.241 RPM=7064
Mission = 0;

for Mission = 1:2

if Mission == 1
    fprintf('\nMotor No.1: T motor - U3 700kv\n');
    kv = 700;
    cell = 4;
    MR = 50;%m-ohm
    Mi0 = 0.5;%A
    % U3 700kv Power eq from E1317 test data
    PowerEq = @(RPM,Tq) (-0.8251+(0.0045*RPM)+(-73.5362*Tq)+(-0.0000*RPM^2)+(0.1093*RPM*Tq)+(694.1561*Tq^2));
elseif Mission == 2
    fprintf('\nMotor No.2: T motor - U5 400kv\n');
    kv = 400;
    cell = 6;
    MR = 116;%m-ohm
    Mi0 = 0.3;%A
    % U5 400kv Power eq from E1317 test data
    PowerEq = @(RPM,Tq) (-0.4125+(0.0033*RPM)+(25.3207*Tq)+(-0.0000*RPM^2)+(0.0978*RPM*Tq)+(130.1791*Tq^2));
end

Vangular = (2*pi*RPM/60);

%cal by E1317 test data
Power_out = Tq*Vangular;
Power_in = PowerEq(RPM,Tq);
Eff_Test =  Power_out/Power_in;

%cal by MIT Lab
v = cell*3.7;
Eff_cal = inline('(1-((Mi0*MR/1000)/(v-(RPM/kv))))*(RPM/(v*kv))'); %motor's eff function
Eff_Lab = Eff_cal(MR,Mi0,RPM,kv,v);

%comparison
diff =  (Eff_Lab-Eff_Test)/Eff_Test*100;
Data = [Eff_Test Eff_Lab diff];
VarNames = {'Eff_ref', 'Eff_cal', 'Difference_percentage'};
Table = table(Data(1),Data(2),Data(3), 'VariableNames',VarNames)
end