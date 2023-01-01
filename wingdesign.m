function[S,c,D,CL,CD,th] = wingdesign(AUW,Vc,b,Exlift,Exdrag_vc)
airfoil %call airfoil data
th = thickness/100; %covert thickness to percent
L = (AUW/1000*9.81) - (Exlift*2); %lift req. (N)
CL = Cl;
S = 2*L/(1.225*(Vc^2)*CL);% calculate wing area
ARnew = b^2/S;
% if ARnew<1.2
%     msg = 'Span too short.';
%     error(msg)
% end
ARold = 0 ;
while abs((ARnew-ARold)/ARnew)*100 >= 0.1
    ARold = ARnew;
    CL = Cl*(ARold/(ARold+2)); %calculate CL 3D from lifting line theory
    S = 2*L/(1.225*(Vc^2)*CL); %calculate wing's area
    ARnew = b^2/S;
    if ARnew < 1 %force AR always more than 1
        ARnew = 1;
    end
end
c = S/b;  %chord (m)
AR = b/c;
% if AR<1
%     msg = 'Span too short.';
%     error(msg)
% end
ar = [5 10 15 20];
eff = [0.935 0.91 0.885 0.86];
Oswald = fit(ar',eff','cubicinterp'); %Oswald's efficiency of rectangular wing that depend on AR
e = Oswald(AR) ;
CD = Cd0 + ((CL^2)/(pi*e*AR)); %3D Drag coefficient
D = (0.5*1.225*(Vc^2)*S*CD)+Exdrag_vc; %Drag (N)