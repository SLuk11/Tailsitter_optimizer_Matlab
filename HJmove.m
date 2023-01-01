function[Xmove,Ymove] = HJmove(x0,y0,x2,y2)

Prop_distribution_box
%x,xp(1) = row , y,xp(2) = column
if x2>x0 && y2==y0
    skip = {@(x) x+1 ;@(y) y+0};
elseif x2>x0 && y2>y0
    skip = {@(x) x+1 ;@(y) y+1};
elseif x2==x0 && y2>y0
    skip = {@(x) x+0 ;@(y) y+1};
elseif x2<x0 && y2>y0
    skip = {@(x) x-1 ;@(y) y+1};
elseif x2<x0 && y2==y0
    skip = {@(x) x-1 ;@(y) y+0};
elseif x2<x0 && y2<y0
    skip = {@(x) x-1 ;@(y) y-1};
elseif x2==x0 && y2<y0
    skip = {@(x) x+0 ;@(y) y-1};
elseif x2>x0 && y2<y0
    skip = {@(x) x+1 ;@(y) y-1};
end

pattern = {@(x2,x0) (2*x2)-x0 ; @(y2,y0) (2*y2)-y0};
xp(1) = pattern{1} (x2,x0);
xp(2) = pattern{2} (y2,y0);

if xp(1) <1 || xp(1)>16 || xp(2)<1 || xp(2)>15
    Pdia = 1;
    xp(1) = 0;
    xp(2) = 0;
else
    Pdia = size_box(xp(1),xp(2),1); %Propeller's diameter
end

while Pdia == 0
    x2 = skip{1}(x2);
    x0 = skip{1}(x0);
    y2 = skip{2}(y2);
    y0 = skip{2}(y0);
    xp(1) = pattern{1} (x2,x0);
    xp(2) = pattern{2} (y2,y0);
    if xp(1) <1 || xp(1)>16 || xp(2)<1 || xp(2)>15
        Pdia = 1;
        xp(1) = 0;
        xp(2) = 0;
    else
        Pdia = size_box(xp(1),xp(2),1); %Propeller's diameter
    end
 
end

Xmove = xp(1);
Ymove = xp(2);