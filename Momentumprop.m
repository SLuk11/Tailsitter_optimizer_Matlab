function [Exlift,Exdrag,Bpc] = Momentumprop(T,Pdia,Vi,c,CL,CD)
A = pi*(Pdia*0.0254/2)^2; % Propeller area
Vj = sqrt((2*T/(1.225*A))+(Vi^2)); %find a jet stream from momentum theory
r = Vi/Vj; %ratio berween free stream and jet stream from actuator theory
Bpc = (Pdia*0.0254)*sqrt((1+r)/2); %Bpc is the fully developed contracted slipstream diameter (m)
Exlift = 0.5*1.225*(Vj^2)*(Bpc*c)*CL; %extra lift (N)
Exdrag = 0.5*1.225*(Vj^2)*(Bpc*c)*CD; %extra drag (N)