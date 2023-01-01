%momentum validate
clc
clear

fprintf('Prop-wing interact validation\n');
Pdia = 12; %inch
A = pi*(Pdia*0.0254/2)^2;
%test result
PWM = [1400 1550 1600];
T = [3.7 7 9.7] ;
Vexit = [9.849895 14.40213 15.79447];
Drag_test= [0.079 0.165 0.302];
Vi = 0;
c = 0.3;
b = 0.9;
%airfoil
airfoil
AR = b/c;
CL = cl_a(0)*(AR/(AR+2));
ar = [5 10 15 20];
eff = [0.935 0.91 0.885 0.86];
Oswald = fit(ar',eff','cubicinterp'); %Oswald's efficiency of rectangular wing that depend on AR
e = Oswald(AR) ;
CD = cd_a(0) + ((CL^2)/(pi*e*AR)); %3D Drag coefficient

%cal part
Vj = zeros(1,3);
Exdrag = zeros(1,3);
for i=1:3
Vj(i) = sqrt((2*T(i)/(1.225*A))+(Vi^2)); %find a jet stream from momentum theory
r = Vi/Vj(i); %ratio berween free stream and jet stream from actuator theory
Bpc = (Pdia*0.0254)*sqrt((1+r)/2); %Bpc is the fully developed contracted slipstream diameter (m)
Exdrag(i) = 0.5*1.225*(Vj(i)^2)*(Bpc*c)*CD; %extra drag (N)
end

%compare
diff_V = ((Vj -Vexit )*100)./Vexit;
diff_D =  ((Exdrag-Drag_test)*100)./Drag_test;
VarNames = {'V_ref', 'V_cal', 'V_Difference_percentage','Drag_ref', 'Drag_cal', 'Drag_Difference_percentage'};
Table = table(Vexit',Vj',diff_V' ,Drag_test',Exdrag',diff_D' ,'VariableNames',VarNames)







