%Power calculator for point plot
function [EPower,AUW,c] = Power_Calculation_forpointplot(Pdia,Ppicth,Wp,w_batt,w_motor,S_factor,Vc,N_m,Mi0,MR,M_maxrpm,Mpowermax,kv,v,b,hover_time,cruise_time,V0,Bcruise)

err = 0;

[T2RPM_v0,RPM2P_v0,T2RPM_vc,RPM2P_vc,Max_Tv0,~,~] = fitprop(Pdia,Ppicth,Vc);%find propeller data thru fitprop.m
old_auw = 0;
new_auw = 100;
Exlift_vc = 0; %extra lift initial
Exdrag_vc = 0;
Exdrag_v0 = 0; %extra drag initial
c = 0.1; %chord initial
thickness = 0; %wing's thickness initial
AUW = weightestimate(Wp,w_batt,w_motor,Pdia,c,b,thickness,old_auw); %calculate all up weight (g)
while (new_auw-old_auw)/new_auw*100>=0.1 %check error
    old_auw = AUW ;
    Tmax = (((AUW/N_m)/1000*9.81*0.224809)*S_factor)+(Exdrag_v0*0.224809); %N to lbf, this is a max thrust for one motor (SafetyFactor times of a Treq)
    RPMmax = T2RPM_v0(Tmax); %find a rpm for generate Thrust max
    Pmax = (RPM2P_v0(RPMmax))*745.7; %find a Max power from rpm max (Hp to W)
    if RPMmax >= M_maxrpm %check motor max rpm
        err = 1;
    elseif Tmax > Max_Tv0 %check Max Thrust
        err = 1;
    elseif Pmax > Mpowermax %check that can motor use with propeller
        err = 1;
    else
        err = 0;
    end
    if err == 1 %Set high value
        fprintf('\nError Prop: %d * %d\n',Pdia,Ppicth);
        EPower = Pmax *2;
        break
    else
        
        Treq_hover = ((AUW/N_m)/1000*9.81*0.224809)+(Exdrag_v0*0.224809) ;% N to lbf
        [~,c,Drag,CL,CD,thickness] = wingdesign(AUW,Vc,Bcruise,Exlift_vc,Exdrag_vc); %S=wing's area (m^2), c = chord mean(m),Drag (N),CL = 3D lift coefficient,CD = 3D Drag coefficient
        Thover = Treq_hover*4.44822; % Treq(lbf to N)
        [~,Exdrag_v0,~] = Momentumprop(Thover,Pdia,V0,c,CL,CD);%find extra drag while hover thru Momentumprop.m
        [Exlift_vc,Exdrag_vc,Bpc] = Momentumprop(Drag,Pdia,Vc,c,CL,CD);%find extra lift and drag while cruise thru Momentumprop.m
        AUW = weightestimate(Wp,w_batt,w_motor,Pdia,c,b,thickness,old_auw); %calculate all up weight (g)
        new_auw = AUW;
        Bcruise = b-Bpc;%span without Propeller
    end
end
if err ~= 1
    fprintf('\nProp: %d * %d\n',Pdia,Ppicth);
    Treq_hover = ((new_auw/N_m)/1000*9.81*0.224809)+(Exdrag_v0*0.224809); % N to lbf
    RPMhover = T2RPM_v0(Treq_hover);%find RPM for hover
    Phover = (RPM2P_v0(RPMhover))*745.7;%find propeller power for hover
    [~,c,Drag,~,~,~] = wingdesign(new_auw,Vc,b,Exlift_vc,Exdrag_vc);%find Chord and drag thru wingdesign.m
    RPMcruise = T2RPM_vc((Drag*0.224809)/2);%find RPM for cruise
    Pcruise = (RPM2P_vc(RPMcruise))*745.7;%find propeller power for cruise
    if RPMcruise >= M_maxrpm
        err = 2;
    elseif RPMmax >= M_maxrpm
        err = 2;
    end
    if err == 2 %Set high value
        fprintf('\nError Prop: %d * %d\n',Pdia,Ppicth);
        EPower = Pmax *2;
    else
        Eff_motor = inline('(1-((Mi0*MR/1000)/(v-(x/kv))))*(x/(v*kv))'); %motor's eff function
        x = RPMhover;
        EPower_hover = Phover/(Eff_motor(MR,Mi0,kv,v,x));%find eletrict power for hover
        x = RPMcruise;
        EPower_cruise = Pcruise/(Eff_motor(MR,Mi0,kv,v,x));%find eletrict power for cruise
        
        EPower = (EPower_hover*hover_time)+(EPower_cruise*cruise_time); %eletrict power for this config
    end
end
