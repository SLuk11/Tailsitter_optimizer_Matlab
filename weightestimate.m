function[AUW]= weightestimate(Wpaload,Wbatt,Wmotor,Dprop,c,b,thickness,W_in) %use this function to sum Equiment's Weight (Payload + Batt + Motor + Propeller + Fixed weight Wing's weight)

syms x ;

BoardCont = 38; %Pixhawk weight(g)
GPS = 18; %GPS and Compass module weight (g)
Pitot = 13; %Pitot tube and air speed sensor weight(g)
Tele = 23; %Telemetry module weight (g)
Receiv = 10; %RC receiver weight(g)
Power = 30; %Power module weight(g)
servo = 15 ; %Servo weight (g)
Body = 500; %body weight (g)
ESC = 26 ; 
[W_wing]= weightwing (W_in,b,c,thickness);%g
Psize=[5 6 9.5 10 11 12 13 14 15 16 18];%prop size
Pweight=[0.14 0.18 0.42 0.53 0.6 0.78 0.85 1.06 1.27 1.55 2.15];%prop wight(oz)
funProp =fit(Psize',Pweight','cubicinterp');
Wprop = 28.3495*funProp(Dprop); %propeller's weight (oz to g)
AUW = Wpaload + Wbatt + (2*Wmotor) + (2*Wprop) + BoardCont + GPS + Pitot + Tele + Receiv + Power + (2*servo) + ESC + W_wing +Body;
