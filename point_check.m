function [Row,Col] = point_check(R0,C0,condition)

Prop_distribution_box

if condition == 1
    step = {@(R0) R0+1 ; @(C0) C0+0};
elseif condition == 2
    step = {@(R0) R0+0 ; @(C0) C0+1};
elseif condition == 3
    step = {@(R0) R0-1 ; @(C0) C0+0};
elseif condition == 4
    step = {@(R0) R0+0 ; @(C0) C0-1};
end

R0 = step{1}(R0);
C0 = step{2}(C0);

if R0 <1 || R0>16 || C0<1 || C0>15
    Pdia = 1;
    R0 = 0;
    C0 = 0;
else
    Pdia = size_box(R0,C0,1); %Propeller's diameter
end


 while Pdia == 0
     R0 = step{1}(R0);
     C0 = step{2}(C0);
     
     if R0 <1 || R0>16 || C0<1 || C0>15
         R0 = 0;
         C0 = 0;
         break
     else
         Pdia = size_box(R0,C0,1); %Propeller's diameter
     end
     
 end
 
 Row = R0;
 Col = C0;
 
 
 