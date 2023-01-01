Propsize = importdata('Propsize.txt'); %import a number of propeller's data ,diameter and picth
Psize = size(Propsize.data,1); %number of propellers
Dia = zeros(1,Psize);
Picth = zeros(1,Psize);
Power_DiaPicth = zeros(1,Psize);
show = zeros(1,Psize);