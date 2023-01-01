function[Xplus,Yplus,Xminus,Yminus] = HJpoint(x,y,condition)

Prop_distribution_box
% x = row, y = column
if condition == 1
    plus = {@(x) x+0 ; @(y) y+1};
    minus = {@(x) x+0 ; @(y) y-1};
elseif condition == 2
    plus = {@(x) x+1 ; @(y) y+0};
    minus = {@(x) x-1 ; @(y) y+0};
end

xplus = plus{1}(x);
yplus = plus{2}(y);
xminus = minus{1}(x);
yminus = minus{2}(y);

if xplus <1 || xplus>16 || yplus<1 || yplus>15
    Pdia_plus = 1;
    xplus = 0;
    yplus = 0;
else
    Pdia_plus = size_box(xplus,yplus,1); %Propeller's diameter
end
if xminus <1 || xminus>16 || yminus<1 || yminus>15
    Pdia_minus = 1;
    xminus = 0;
    yminus = 0;
else
    Pdia_minus = size_box(xminus,yminus,1); %Propeller's diameter
end
while Pdia_plus == 0
     xplus = plus{1}(xplus);
     yplus = plus{2}(yplus);
     
     if xplus <1 || xplus>16 || yplus<1 || yplus>15
         xplus = 0;
         yplus = 0;
         break
     else
         Pdia_plus = size_box(xplus,yplus,1); %Propeller's diameter
     end 
end
 while Pdia_minus == 0
     xminus = minus{1}(xminus);
     yminus = minus{2}(yminus);
     
     if xminus <1 || xminus>16 || yminus<1 || yminus>15
         xminus = 0;
         yminus = 0;
         break
     else
         Pdia_minus = size_box(xminus,yminus,1); %Propeller's diameter
     end 
 end
 
 Xplus = xplus;
 Yplus = yplus;
 Xminus = xminus;
 Yminus = yminus;
 