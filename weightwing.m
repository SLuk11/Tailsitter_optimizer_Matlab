function[W_wing]= weightwing (Win,b,c,th_airfoil)
LF_max = 3;%load factor max
SF = 1.4;%safety factor
Lmax = (Win/1000)*9.81*LF_max*SF; %Max lift
Mb = (Lmax/2)*(b/4); %Max bending moment

thickness= c*th_airfoil;
%skin part

Dens_skin = 1600 ;%skin density cabon (kg/m^3)
%bending_stress_skin = 0.2*10^9;%(Pa)
tf_skin = 0.00015; %skin thickness of the flanges (m)
tw_skin = 0.003; %skin thickness of the web (m)
h_skin_factor = 0.04954; %web thickness factor
h_skin = h_skin_factor*c; %web thickness

I_skin = (c*h_skin^3/12)-((c-tw_skin)*((h_skin-(2*tf_skin))^3)/12);
W_skin = 2*b*c*tf_skin*Dens_skin;

%Rib part
Dens_rib = 160;%Rib density Balsa (kg/m^3)
t_rib = 0.003; %rib thickness (m)
N_rib = round(b/(c/5))+2;
A_rib_factor = 0.061025 ; %Area MH45 per unit factor (m^2)
A_rib = A_rib_factor*c*c; %Area MH45 rib
W_rib = A_rib*t_rib*Dens_rib*N_rib;
W_glue = N_rib*0.002;
%Spar part
Dens_spar_flange = 1600 ;%skin density cabon (kg/m^3)
bending_stress_spar = 1.5e+8; 
Dens_spar_web = 160 ;%skin density balsa (kg/m^3)
tf_spar = 0.002; %skin thickness of the flanges (m)
tw_spar = 0.003; %skin thickness of the web (m)
I_spar = (Mb*(thickness/2)/bending_stress_spar)-I_skin;
%if I>0 this wing need a I-beam spar else just use a balsa wood spar
if I_spar > 0
    B_flange = tw_spar+(12*I_spar/((thickness-(2*tf_spar))^3));
    W_spar = ((thickness-(2*tf_spar))*tw_spar*b*Dens_spar_web)+(2*B_flange *tf_spar*Dens_spar_flange);
    
    W_wing = (W_skin+W_rib+W_spar+W_glue)*1000 ;%kg to g
else
    W_spar = ((thickness-(2*tf_spar))*tw_spar*b*Dens_spar_web);
    W_wing = (W_skin+W_rib+W_spar+W_glue)*1000 ;
end
